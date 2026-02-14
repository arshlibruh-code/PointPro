//
//  ContentView.swift
//  PointPro
//

import SwiftUI
import ARKit
import UIKit

struct ContentView: View {
    @StateObject private var arManager = ARSessionManager()
    @State private var show3DViewer = false
    @State private var didAutoStart = false
    @State private var showStartupOverlay = true
    var autoStartScan: Bool = false
    var showBottomCaptureControls: Bool = true
    var showMetricsOnlyWhileScanning: Bool = false
    var newScanSignal: Int = 0
    var clearScanSignal: Int = 0
    var saveSnapshotSignal: Int = 0
    var startScanSignal: Int = 0
    var stopScanSignal: Int = 0
    var openViewerSignal: Int = 0
    var viewerSession: ScanSession? = nil
    var onSnapshotSaved: ((Data, Int, ScanCaptureMetadata?) -> Void)? = nil
    var onPrepareViewer: ((PointCloudEngine) -> Void)? = nil
    var onViewerDismissed: (() -> Void)? = nil
    var onViewerContinue: (() -> Void)? = nil
    var onPointCountChanged: ((Int) -> Void)? = nil
    var onScanningChanged: ((Bool) -> Void)? = nil
    var onCameraReady: (() -> Void)? = nil
    
    var body: some View {
        ZStack {
            // Layer 1: Live camera feed (RealityKit)
            // Note: ARViewContainer now simplified as it doesn't do mesh anymore
            ARViewContainer(session: arManager.session)
                .ignoresSafeArea()
            
            // Layer 2: Metal Point Cloud (Raw Metal Points)
            if let device = MTLCreateSystemDefaultDevice() {
                MetalPointCloudView(
                    device: device,
                    engine: arManager.pointCloudEngine,
                    session: arManager.session
                )
                .ignoresSafeArea()
                .allowsHitTesting(false) // Let touches pass through to ARView
            }
            
            // Layer 3: HUD Overlay
            HUDView(
                arManager: arManager,
                show3DViewer: $show3DViewer,
                showBottomCaptureControls: showBottomCaptureControls,
                showMetricsOnlyWhileScanning: showMetricsOnlyWhileScanning
            )

            if showStartupOverlay {
                ZStack {
                    Color(red: 0.96, green: 0.39, blue: 0.10).ignoresSafeArea()
                    Image("LaunchLogo")
                        .resizable()
                        .scaledToFit()
                        .frame(width: 220, height: 220)
                }
                .transition(.opacity)
            }
        }
        .onAppear {
            arManager.startSession()
        }
        .preferredColorScheme(.dark)
        .fullScreenCover(isPresented: $show3DViewer, onDismiss: {
            onViewerDismissed?()
        }) {
            if let device = MTLCreateSystemDefaultDevice() {
                let viewerCaptureMetadata = loadCaptureMetadata(for: viewerSession)
                PointCloudViewer(
                    engine: arManager.pointCloudEngine,
                    device: device,
                    session: viewerSession,
                    captureMetadata: viewerCaptureMetadata,
                    onExport: { session, format, measurements, progress, isCancelled in
                        func mapProgress(_ start: Double, _ end: Double) -> (Double, String) -> Void {
                            { fraction, message in
                                let clamped = min(max(fraction, 0), 1)
                                progress(start + ((end - start) * clamped), message)
                            }
                        }

                        guard let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return nil }
                        let snapshotURL = docs
                            .appendingPathComponent("scans", isDirectory: true)
                            .appendingPathComponent("\(session.id.uuidString).pcraw")
                        guard let snapshotData = try? Data(contentsOf: snapshotURL) else { return nil }
                        let captureMetadata = loadCaptureMetadata(for: session) ?? viewerCaptureMetadata
                        switch format {
                        case .laz:
                            guard var artifact = arManager.pointCloudEngine.exportLAZFile(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                suggestedName: session.name,
                                session: session,
                                captureMetadata: captureMetadata,
                                progress: mapProgress(0.02, 0.60),
                                isCancelled: isCancelled
                            ) else { return nil }
                            guard let reportURL = arManager.pointCloudEngine.generateExportReportPDF(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                session: session,
                                captureMetadata: captureMetadata,
                                exportFormat: format,
                                primaryURL: artifact.primaryURL,
                                sidecarURL: artifact.sidecarURL,
                                measurements: measurements,
                                progress: mapProgress(0.60, 0.97),
                                isCancelled: isCancelled
                            ) else { return nil }
                            artifact.additionalURLs.append(reportURL)
                            progress(0.985, "Preparing share sheet...")
                            return artifact
                        case .plyBinaryLittleEndian:
                            guard let url = arManager.pointCloudEngine.exportPLYFile(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                suggestedName: session.name,
                                format: .binaryLittleEndian,
                                progress: mapProgress(0.02, 0.55),
                                isCancelled: isCancelled
                            ) else { return nil }
                            var artifact = PointCloudEngine.ExportArtifact(primaryURL: url, sidecarURL: nil)
                            guard let reportURL = arManager.pointCloudEngine.generateExportReportPDF(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                session: session,
                                captureMetadata: captureMetadata,
                                exportFormat: format,
                                primaryURL: url,
                                sidecarURL: nil,
                                measurements: measurements,
                                progress: mapProgress(0.55, 0.97),
                                isCancelled: isCancelled
                            ) else { return nil }
                            artifact.additionalURLs.append(reportURL)
                            progress(0.985, "Preparing share sheet...")
                            return artifact
                        case .plyAscii:
                            guard let url = arManager.pointCloudEngine.exportPLYFile(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                suggestedName: session.name,
                                format: .ascii,
                                progress: mapProgress(0.02, 0.55),
                                isCancelled: isCancelled
                            ) else { return nil }
                            var artifact = PointCloudEngine.ExportArtifact(primaryURL: url, sidecarURL: nil)
                            guard let reportURL = arManager.pointCloudEngine.generateExportReportPDF(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                session: session,
                                captureMetadata: captureMetadata,
                                exportFormat: format,
                                primaryURL: url,
                                sidecarURL: nil,
                                measurements: measurements,
                                progress: mapProgress(0.55, 0.97),
                                isCancelled: isCancelled
                            ) else { return nil }
                                artifact.additionalURLs.append(reportURL)
                            progress(0.985, "Preparing share sheet...")
                            return artifact
                        case .pdfReport:
                            guard let reportURL = arManager.pointCloudEngine.exportReportPDFOnly(
                                fromSnapshot: snapshotData,
                                pointCount: session.pointCount,
                                suggestedName: session.name,
                                session: session,
                                captureMetadata: captureMetadata,
                                measurements: measurements,
                                progress: mapProgress(0.05, 0.97),
                                isCancelled: isCancelled
                            ) else { return nil }
                            progress(0.985, "Preparing share sheet...")
                            return .init(primaryURL: reportURL, sidecarURL: nil)
                        }
                    },
                    onContinue: {
                        onViewerContinue?()
                        show3DViewer = false
                    }
                )
            }
        }
        .onChange(of: show3DViewer) { _, isShowing in
            if isShowing {
                arManager.pauseSession()
            } else {
                arManager.resumeSession()
            }
        }
        .onAppear {
            onPointCountChanged?(arManager.pointCloudEngine.activePointCount)
            onScanningChanged?(arManager.isScanning)
            if autoStartScan && !didAutoStart && !arManager.isScanning {
                didAutoStart = true
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) {
                    if !arManager.isScanning {
                        arManager.toggleScanning()
                    }
                }
            }
        }
        .onChange(of: arManager.pointCloudEngine.activePointCount) { _, pointCount in
            onPointCountChanged?(pointCount)
        }
        .onChange(of: arManager.isScanning) { _, isScanning in
            onScanningChanged?(isScanning)
        }
        .onChange(of: startScanSignal) { _, _ in
            if !arManager.isScanning {
                arManager.toggleScanning()
            }
        }
        .onChange(of: newScanSignal) { _, _ in
            arManager.resetScan()
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.05) {
                if !arManager.isScanning {
                    arManager.toggleScanning()
                }
            }
        }
        .onChange(of: clearScanSignal) { _, _ in
            arManager.resetScan()
        }
        .onChange(of: saveSnapshotSignal) { _, _ in
            let snapshot = arManager.pointCloudEngine.makeSnapshot()
            let worldOrigin = arManager.pointCloudEngine.getWorldOriginTransform()
            let flattenedTransform: [Float] = [
                worldOrigin.columns.0.x, worldOrigin.columns.0.y, worldOrigin.columns.0.z, worldOrigin.columns.0.w,
                worldOrigin.columns.1.x, worldOrigin.columns.1.y, worldOrigin.columns.1.z, worldOrigin.columns.1.w,
                worldOrigin.columns.2.x, worldOrigin.columns.2.y, worldOrigin.columns.2.z, worldOrigin.columns.2.w,
                worldOrigin.columns.3.x, worldOrigin.columns.3.y, worldOrigin.columns.3.z, worldOrigin.columns.3.w,
            ]
            let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String
            let metadata = ScanCaptureMetadata(
                capturedAt: Date(),
                worldOriginTransform: flattenedTransform,
                location: arManager.latestLocationMetadata,
                deviceModel: UIDevice.current.model,
                systemName: UIDevice.current.systemName,
                systemVersion: UIDevice.current.systemVersion,
                appVersion: appVersion,
                depthResolution: [arManager.depthWidth, arManager.depthHeight],
                colorResolution: [arManager.imageWidth, arManager.imageHeight]
            )
            onSnapshotSaved?(snapshot.data, snapshot.pointCount, metadata)
        }
        .onChange(of: stopScanSignal) { _, _ in
            if arManager.isScanning {
                arManager.toggleScanning()
            }
        }
        .onChange(of: openViewerSignal) { _, _ in
            onPrepareViewer?(arManager.pointCloudEngine)
            show3DViewer = true
        }
        .onChange(of: arManager.hasReceivedFirstFrame) { _, ready in
            guard ready else { return }
            withAnimation(.easeOut(duration: 0.2)) {
                showStartupOverlay = false
            }
            onCameraReady?()
        }
    }

    private func loadCaptureMetadata(for session: ScanSession?) -> ScanCaptureMetadata? {
        guard let session else { return nil }
        guard let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return nil }
        let metadataURL = docs
            .appendingPathComponent("scans", isDirectory: true)
            .appendingPathComponent("\(session.id.uuidString).meta.json")
        guard let data = try? Data(contentsOf: metadataURL) else { return nil }
        return try? JSONDecoder().decode(ScanCaptureMetadata.self, from: data)
    }
}

// Separate HUD to keep ContentView clean
struct HUDView: View {
    @ObservedObject var arManager: ARSessionManager
    @Binding var show3DViewer: Bool
    let showBottomCaptureControls: Bool
    let showMetricsOnlyWhileScanning: Bool
    private let tapHaptic = UIImpactFeedbackGenerator(style: .rigid)
    
    var body: some View {
        VStack {
            // Top bar
            if !showMetricsOnlyWhileScanning || arManager.isScanning {
                HStack {
                    TopInfoPill(
                        text: "\(formatStorage(arManager.storageUsedBytes)) USED",
                        textColor: .secondary
                    )

                    if arManager.isScanning || arManager.pointCloudEngine.activePointCount > 0 {
                        TopInfoPill(
                            text: "\(formatNumber(arManager.pointCloudEngine.activePointCount)) POINTS",
                            actionIcon: "trash.circle",
                            action: {
                                withAnimation(.easeInOut(duration: 0.22)) {
                                    arManager.resetScan()
                                }
                            },
                            isActionDisabled: arManager.pointCloudEngine.activePointCount == 0
                        )
                        .transition(.asymmetric(insertion: .move(edge: .leading).combined(with: .opacity), removal: .opacity))
                    }
                    Spacer()
                    TopInfoPill(
                        text: "\(arManager.depthFPS) FPS",
                        textColor: arManager.depthFPS == 0 ? .secondary : (arManager.depthFPS >= 30 ? .green : .yellow)
                    )
                }
                .padding(.horizontal, 20)
                .padding(.top, 10)
                .animation(.easeInOut(duration: 0.22), value: arManager.pointCloudEngine.activePointCount)
                .animation(.easeInOut(duration: 0.22), value: arManager.isScanning)
            }
            
            Spacer()
            
            // Bottom Controls
            if showBottomCaptureControls {
                VStack(spacing: 12) {
                    // Scan Button
                    Button(action: {
                        emitTapHaptic()
                        arManager.toggleScanning()
                    }) {
                        HStack {
                            Image(systemName: arManager.isScanning ? "stop.fill" : "record.circle")
                            Text(arManager.isScanning ? "STOP SCAN" : "START SCAN")
                        }
                        .font(.system(.subheadline, design: .monospaced).bold())
                        .foregroundStyle(.primary)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                    }
                    .tint(arManager.isScanning ? Color.red.opacity(0.7) : .blue)
                    .buttonStyle(.glassProminent)
                    
                    if !arManager.isScanning {
                        // Only show VIEW 3D button if we have points
                        if arManager.pointCloudEngine.activePointCount > 0 {
                            Button(action: {
                                emitTapHaptic()
                                show3DViewer = true
                            }) {
                                HStack {
                                    Image(systemName: "cube")
                                    Text("VIEW 3D")
                                }
                                .font(.system(.subheadline, design: .monospaced).bold())
                                .foregroundStyle(.primary)
                                .frame(maxWidth: .infinity)
                                .padding(.vertical, 16)
                            }
                            .buttonStyle(.glass)
                            .transition(.asymmetric(insertion: .move(edge: .bottom).combined(with: .opacity), removal: .opacity))
                        }
                    }
                }
                .padding(.horizontal, 16)
                .padding(.bottom, 30)
                .animation(.easeInOut(duration: 0.22), value: arManager.pointCloudEngine.activePointCount)
                .animation(.easeInOut(duration: 0.22), value: arManager.isScanning)
            }
        }
        .overlay {
            GeometryReader { geo in
                if (!showMetricsOnlyWhileScanning || arManager.isScanning) &&
                    (arManager.isPhoneFlatOnTable || arManager.isTooClose || arManager.averageDepth > 0) {
                    let readoutText = depthReadoutText
                    let charWidth: CGFloat = 7.2
                    let horizontalPadding: CGFloat = 14
                    let readoutWidth = max(74, CGFloat(readoutText.count) * charWidth + horizontalPadding)
                    let readoutSize = CGSize(width: readoutWidth, height: 20)
                    let centerX = geo.size.width / 2
                    let centerY = geo.size.height / 2
                    let readoutCenterY = centerY - 40
                    let readoutBottom = readoutCenterY + (readoutSize.height / 2)

                    ZStack {
                        Path { path in
                            path.move(to: CGPoint(x: centerX, y: centerY))
                            path.addLine(to: CGPoint(x: centerX, y: readoutBottom))
                        }
                        .stroke(Color.white.opacity(0.1), lineWidth: 1)

                        Text(readoutText)
                            .font(.system(size: 11, weight: .medium, design: .monospaced))
                            .foregroundColor(.white)
                            .frame(width: readoutSize.width, height: readoutSize.height)
                            .overlay(
                                RoundedRectangle(cornerRadius: 6)
                                    .stroke(Color.white.opacity(0.1), lineWidth: 1)
                            )
                            .position(x: centerX, y: readoutCenterY)

                        Circle()
                            .fill(.white.opacity(0.95))
                            .frame(width: 4, height: 4)
                            .shadow(color: .black.opacity(0.4), radius: 1)
                            .position(x: centerX, y: centerY)
                    }
                    .allowsHitTesting(false)
                }
            }
        }
    }

    private var depthReadoutText: String {
        if arManager.isPhoneFlatOnTable {
            return "PICK UP PHONE"
        }
        if arManager.isTooClose {
            return "TOO CLOSE"
        }
        return String(format: "%.2f m", arManager.averageDepth)
    }

    private func emitTapHaptic() {
        tapHaptic.prepare()
        tapHaptic.impactOccurred(intensity: 1.0)
    }
}

// Minimal Components
struct TopInfoPill: View {
    let text: String
    var textColor: Color = .primary
    var actionIcon: String? = nil
    var action: (() -> Void)? = nil
    var isActionDisabled: Bool = false
    private let tapHaptic = UIImpactFeedbackGenerator(style: .rigid)

    var body: some View {
        Group {
            if let actionIcon, let action {
                Button(action: {
                    emitTapHaptic()
                    action()
                }) {
                    HStack(spacing: 8) {
                        Text(text)
                            .font(.system(size: 10, design: .monospaced).bold())
                        Image(systemName: actionIcon)
                            .font(.system(size: 12, weight: .semibold))
                    }
                    .foregroundStyle(textColor)
                }
                .buttonStyle(.glass)
                .disabled(isActionDisabled)
            } else {
                Button(action: {}) {
                    Text(text)
                        .font(.system(size: 10, design: .monospaced).bold())
                        .foregroundStyle(textColor)
                }
                .buttonStyle(.glass)
                .disabled(true)
            }
        }
    }

    private func emitTapHaptic() {
        tapHaptic.prepare()
        tapHaptic.impactOccurred(intensity: 1.0)
    }
}

// Helper function for formatting numbers with commas
private func formatNumber(_ num: Int) -> String {
    let formatter = NumberFormatter()
    formatter.numberStyle = .decimal
    return formatter.string(from: NSNumber(value: num)) ?? "\(num)"
}

private func formatStorage(_ bytes: Int64) -> String {
    ByteCountFormatter.string(fromByteCount: bytes, countStyle: .file)
}
