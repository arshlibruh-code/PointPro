//
//  PointCloudViewer.swift
//  PointPro
//
//  3D viewport with orbit camera for viewing captured point clouds
//

import SwiftUI
import MetalKit
import simd
import UIKit
import QuartzCore
import Combine

enum CloudColorMode: Int, CaseIterable {
    case auto = 0
    case rgb = 1
    case elevation = 2
    case intensity = 3
    case classification = 4

    var title: String {
        switch self {
        case .auto: return "AUTO"
        case .rgb: return "RGB"
        case .elevation: return "ELEVATION"
        case .intensity: return "INTENSITY"
        case .classification: return "CLASSIFICATION"
        }
    }
}

struct PointCloudRenderColorStatus: Equatable {
    var hasUsableRGB: Bool
    var effectiveMode: CloudColorMode
}

struct PointCloudViewer: View {
    let engine: PointCloudEngine
    let device: MTLDevice
    let session: ScanSession?
    let captureMetadata: ScanCaptureMetadata?
    var onExport: ((ScanSession, PointCloudEngine.ExportFormat, [ScanReportMeasurement], @escaping (Double, String) -> Void, @escaping () -> Bool) -> PointCloudEngine.ExportArtifact?)? = nil
    var onContinue: (() -> Void)? = nil
    @Environment(\.dismiss) var dismiss
    @State private var isExporting = false
    @State private var exportURLs: [URL] = []
    @State private var showShareSheet = false
    @State private var showExportError = false
    @State private var exportStatusText = "Preparing export..."
    @State private var exportProgress: Double = 0.0
    @State private var exportCancellation = ExportCancellationFlag()
    @State private var showExportCancelledNotice = false
    @State private var showExportFormatPicker = false
    @State private var showColorModePicker = false
    @State private var showImportURLSheet = false
    @State private var importURLText = "https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz"
    @State private var isImporting = false
    @State private var importStatusText = "Preparing import..."
    @State private var importProgress: Double = 0.0
    @State private var importErrorText = "Could not import this URL."
    @State private var showImportError = false
    @State private var importCancellation = ExportCancellationFlag()
    @State private var importedSourceLabel: String?
    @State private var importedSourceURLText: String?
    @State private var showImportCancelledNotice = false
    @State private var showGeorefPopover = false
    @State private var recenterSignal = 0
    @State private var isRollUnlocked = false
    @State private var uprightResetSignal = 0
    @State private var selectedColorMode: CloudColorMode = .auto
    @State private var effectiveColorMode: CloudColorMode = .auto
    @State private var hasUsableRGB = true
    @StateObject private var measureController = MeasureController()
    private let buttonHaptic = UIImpactFeedbackGenerator(style: .rigid)
    private let importSampleURLs: [(title: String, url: String)] = [
        ("Autzen Stadium (Sample)", "https://s3.amazonaws.com/hobu-lidar/autzen-classified.copc.laz"),
        ("Madison", "https://data.opengeos.org/madison.copc.laz"),
        ("USGS TX Coastal", "https://data.opengeos.org/USGS_LPC_TX_CoastalRegion_2018_A18_stratmap18-50cm-2995201a1.copc.laz"),
        ("Chicago", "https://data.opengeos.org/chicago.copc.laz")
    ]
    
    var body: some View {
        ZStack {
            // Black background
            Color.black.ignoresSafeArea()
            
            // Metal 3D view
            PointCloudViewerMetal(
                device: device,
                engine: engine,
                isRenderingPaused: showExportFormatPicker || showImportURLSheet,
                recenterSignal: recenterSignal,
                isRollUnlocked: isRollUnlocked,
                uprightResetSignal: uprightResetSignal,
                measureController: measureController,
                selectedColorMode: selectedColorMode,
                onRenderColorStatusChanged: { status in
                    hasUsableRGB = status.hasUsableRGB
                    effectiveColorMode = status.effectiveMode
                }
            )
                .ignoresSafeArea()

            if measureController.isEnabled {
                GeometryReader { geo in
                    Canvas { context, _ in
                        for segment in measureController.projectedCommittedSegments {
                            var path = Path()
                            path.move(to: segment.a)
                            path.addLine(to: segment.b)
                            context.stroke(path, with: .color(segment.tint), lineWidth: segment.width)
                        }

                        for segment in measureController.projectedActiveSegments {
                            var path = Path()
                            path.move(to: segment.a)
                            path.addLine(to: segment.b)
                            context.stroke(path, with: .color(segment.tint), lineWidth: segment.width)
                        }

                        for segment in measureController.projectedCommittedSwathSegments {
                            var path = Path()
                            path.move(to: segment.a)
                            path.addLine(to: segment.b)
                            context.stroke(path, with: .color(segment.tint), lineWidth: segment.width)
                        }

                        for segment in measureController.projectedActiveSwathSegments {
                            var path = Path()
                            path.move(to: segment.a)
                            path.addLine(to: segment.b)
                            context.stroke(path, with: .color(segment.tint), lineWidth: segment.width)
                        }

                        for point in measureController.projectedCommittedVertices {
                            let dotRect = CGRect(x: point.x - 2.8, y: point.y - 2.8, width: 5.6, height: 5.6)
                            context.fill(Path(ellipseIn: dotRect), with: .color(.white))
                        }

                        for point in measureController.projectedActiveVertices {
                            let dotRect = CGRect(x: point.x - 3.0, y: point.y - 3.0, width: 6.0, height: 6.0)
                            context.fill(Path(ellipseIn: dotRect), with: .color(.blue))
                        }

                        let center = CGPoint(x: geo.size.width / 2, y: geo.size.height / 2)
                        var cross = Path()
                        cross.move(to: CGPoint(x: center.x - 10, y: center.y))
                        cross.addLine(to: CGPoint(x: center.x + 10, y: center.y))
                        cross.move(to: CGPoint(x: center.x, y: center.y - 10))
                        cross.addLine(to: CGPoint(x: center.x, y: center.y + 10))
                        context.stroke(cross, with: .color(.white.opacity(0.8)), lineWidth: 0.8)

                        if let projectedSnap = measureController.projectedSnapPoint {
                            var connector = Path()
                            connector.move(to: center)
                            connector.addLine(to: projectedSnap)
                            context.stroke(connector, with: .color(.green.opacity(0.55)), lineWidth: 0.8)

                            let outer = CGRect(x: projectedSnap.x - 7.0, y: projectedSnap.y - 7.0, width: 14.0, height: 14.0)
                            context.stroke(Path(ellipseIn: outer), with: .color(.green), lineWidth: 0.8)
                            let inner = CGRect(x: projectedSnap.x - 2.2, y: projectedSnap.y - 2.2, width: 4.4, height: 4.4)
                            context.fill(Path(ellipseIn: inner), with: .color(.green))
                        }
                    }
                }
                .allowsHitTesting(false)
                .ignoresSafeArea()

                GeometryReader { _ in
                    ForEach(measureController.overlayLabels) { label in
                        Text(label.text)
                            .font(.system(.caption2, design: .monospaced).bold())
                            .foregroundStyle(label.tint)
                            .padding(.horizontal, 6)
                            .padding(.vertical, 3)
                            .background(.thinMaterial, in: Capsule())
                            .position(label.position)
                    }
                }
                .allowsHitTesting(false)
                .ignoresSafeArea()
            }
            
            // UI overlay
            VStack {
                HStack {
                    Button(action: {
                        emitTapHaptic()
                        dismiss()
                    }) {
                        Image(systemName: "xmark")
                            .font(.system(size: 14, weight: .semibold))
                            .frame(width: 30, height: 30)
                    }
                    .buttonStyle(.glass)
                    .padding()

                    Spacer()

                    HStack(spacing: 4) {
                        Button(action: {
                            emitTapHaptic()
                            recenterSignal &+= 1
                        }) {
                            Image(systemName: "scope")
                                .font(.system(size: 14, weight: .semibold))
                                .frame(width: 30, height: 30)
                        }
                        .buttonStyle(.glass)

                        if isRollUnlocked {
                            Button(action: {
                                emitTapHaptic()
                                uprightResetSignal &+= 1
                            }) {
                                Image(systemName: "arrow.up.circle")
                                    .font(.system(size: 13, weight: .semibold))
                                    .frame(width: 30, height: 30)
                            }
                            .buttonStyle(.glass)
                            .accessibilityLabel("Reset Upright")
                        }

                        Button(action: {
                            emitTapHaptic()
                            if isRollUnlocked {
                                uprightResetSignal &+= 1
                                isRollUnlocked = false
                            } else {
                                isRollUnlocked = true
                            }
                        }) {
                            Image(systemName: isRollUnlocked ? "rotate.3d" : "level")
                                .font(.system(size: 13, weight: .semibold))
                                .frame(width: 30, height: 30)
                        }
                        .buttonStyle(.glass)
                        .tint(isRollUnlocked ? .blue : .gray)
                        .accessibilityLabel(isRollUnlocked ? "Free Roll Enabled (Tap to Level Lock)" : "Level Lock Enabled")
                    }
                    .padding()
                }
                
                Spacer()

                if shouldShowInfoPanel {
                    VStack(alignment: .leading, spacing: 8) {
                        Text(panelTitle)
                            .font(.system(.title3, design: .monospaced).weight(.bold))
                            .foregroundStyle(.primary)

                        Text(panelSubtitle)
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)

                        HStack(spacing: 8) {
                            infoGlass("\(formatNumber(panelPointCount)) POINTS", tint: .blue)
                            if let session {
                                infoGlass(formatStorage(session.dataSizeBytes))
                                infoGlass(session.status.rawValue.uppercased())
                                georefStatusPill
                            }
                        }

                        if session == nil {
                            colorModeMenu
                        }

                        if measureController.isEnabled, let analysisCard = measureController.currentAnalysisCard {
                            analysisResultCard(analysisCard)
                                .transition(.move(edge: .bottom).combined(with: .opacity))
                        }

                        HStack(alignment: .bottom, spacing: 10) {
                            if measureController.isEnabled {
                                Button(action: {
                                    emitTapHaptic()
                                    measureController.toggleMode()
                                }) {
                                    Image(systemName: "ruler.fill")
                                        .font(.system(.caption, design: .monospaced).bold())
                                        .frame(width: 36, height: 36)
                                }
                                .buttonStyle(.glassProminent)
                                .tint(.blue)
                                .accessibilityLabel("Disable Measure Mode")
                            } else {
                                Button(action: {
                                    emitTapHaptic()
                                    measureController.toggleMode()
                                }) {
                                    Image(systemName: "ruler")
                                        .font(.system(.caption, design: .monospaced).bold())
                                        .frame(width: 36, height: 36)
                                }
                                .buttonStyle(.glass)
                                .tint(.gray)
                                .accessibilityLabel("Enable Measure Mode")
                            }

                            Spacer(minLength: 0)

                            ZStack(alignment: .trailing) {
                                if measureController.isEnabled {
                                    VStack(alignment: .trailing, spacing: 8) {
                                        HStack(spacing: 6) {
                                            measureToolButton(title: "Path", tool: .path)
                                            measureToolButton(title: "Cross", tool: .crossSection)
                                            measureToolButton(title: "Elev", tool: .elevationProfile)
                                        }

                                        if measureController.selectedTool == .crossSection {
                                            HStack(spacing: 8) {
                                                Text("Swath")
                                                    .font(.system(.caption2, design: .monospaced).weight(.bold))
                                                    .foregroundStyle(.secondary)
                                                Slider(value: $measureController.swathWidthMeters, in: 0.05...2.0, step: 0.01)
                                                    .tint(.blue)
                                                    .frame(width: 120)
                                                Text(formatMeters(measureController.swathWidthMeters))
                                                    .font(.system(.caption2, design: .monospaced))
                                                    .foregroundStyle(.primary)
                                            }
                                            .padding(.horizontal, 10)
                                            .padding(.vertical, 6)
                                            .background(.thinMaterial, in: Capsule())
                                        }

                                        HStack(spacing: 8) {
                                            iconActionButton("plus", label: "Add Vertex") {
                                                emitTapHaptic()
                                                measureController.addVertex()
                                            }
                                            .disabled(!measureController.hasSnapPoint)

                                            iconActionButton("arrow.uturn.backward", label: "Undo Vertex") {
                                                emitTapHaptic()
                                                measureController.undoLastVertex()
                                            }
                                            .disabled(!measureController.hasActiveVertices)

                                            if measureController.selectedTool == .path {
                                                iconActionButton("checkmark", label: "Finish Line") {
                                                    emitTapHaptic()
                                                    measureController.commitOpenMeasurementAndStartNew()
                                                }
                                                .disabled(!measureController.canCommitOpenMeasurement)

                                                iconActionButton("seal", label: "Close Shape") {
                                                    emitTapHaptic()
                                                    measureController.closeCurrentMeasurement()
                                                }
                                                .disabled(!measureController.canCloseCurrentMeasurement)
                                            } else {
                                                iconActionButton(
                                                    measureController.selectedTool == .crossSection ? "square.split.2x1" : "chart.xyaxis.line",
                                                    label: measureController.selectedTool == .crossSection ? "Analyze Cross Section" : "Generate Elevation Profile"
                                                ) {
                                                    emitTapHaptic()
                                                    measureController.finalizeActiveMeasurementForCurrentTool()
                                                }
                                                .disabled(!measureController.canFinalizeCurrentTool)
                                            }
                                        }
                                    }
                                    .padding(.vertical, 2)
                                    .id("measure-actions")
                                    .transition(.asymmetric(
                                        insertion: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing)),
                                        removal: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing))
                                    ))
                                } else {
                                    HStack(spacing: 8) {
                                        if session != nil, let onContinue {
                                            Button(action: {
                                                emitTapHaptic()
                                                onContinue()
                                            }) {
                                                Image(systemName: "dot.radiowaves.left.and.right")
                                                    .font(.system(.caption, design: .monospaced).bold())
                                                    .frame(width: 36, height: 36)
                                            }
                                            .tint(.gray)
                                            .buttonStyle(.glass)
                                            .disabled(isExporting || isImporting || showExportFormatPicker || showImportURLSheet)
                                            .accessibilityLabel("Continue Scan")
                                        }

                                        if session == nil {
                                            Button(action: {
                                                guard !isExporting, !isImporting else { return }
                                                emitTapHaptic()
                                                showImportURLSheet = true
                                            }) {
                                                Image(systemName: "link.badge.plus")
                                                    .font(.system(.caption, design: .monospaced).bold())
                                                    .frame(width: 36, height: 36)
                                            }
                                            .tint(.gray)
                                            .buttonStyle(.glass)
                                            .disabled(isExporting || isImporting)
                                            .accessibilityLabel("Load URL")
                                        }

                                        if session != nil, onExport != nil {
                                            Button(action: {
                                                guard !isExporting, !isImporting else { return }
                                                emitTapHaptic()
                                                showExportFormatPicker = true
                                            }) {
                                                HStack {
                                                    Image(systemName: "square.and.arrow.up")
                                                    Text(isExporting ? "EXPORTING..." : "EXPORT")
                                                }
                                                .font(.system(.caption, design: .monospaced).bold())
                                                .padding(.horizontal, 12)
                                                .frame(height: 36)
                                            }
                                            .tint(.blue)
                                            .buttonStyle(.glassProminent)
                                            .disabled(isExporting || isImporting)
                                        }
                                    }
                                    .padding(.vertical, 2)
                                    .id("default-actions")
                                    .transition(.asymmetric(
                                        insertion: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing)),
                                        removal: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing))
                                    ))
                                }
                            }
                            .animation(.spring(response: 0.28, dampingFraction: 0.85), value: measureController.isEnabled)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding(.horizontal, 16)
                    .padding(.bottom, 22)
                }
            }
        }
        .preferredColorScheme(.dark)
        .overlay(alignment: .top) {
            if isExporting || isImporting {
                HStack(spacing: 10) {
                    VStack(alignment: .leading, spacing: 6) {
                        Text(isImporting ? importStatusText : exportStatusText)
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.white)
                            .lineLimit(1)
                        HStack(spacing: 10) {
                            ProgressView(value: isImporting ? importProgress : exportProgress, total: 1.0)
                                .tint(.white)
                                .frame(maxWidth: .infinity)
                            Text("\(Int((isImporting ? importProgress : exportProgress) * 100))%")
                                .font(.system(.caption, design: .monospaced).bold())
                                .foregroundStyle(.white)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)

                    Button("CANCEL") {
                        emitTapHaptic()
                        if isImporting {
                            importCancellation.requestCancel()
                            importStatusText = "Cancelling import..."
                        } else {
                            exportCancellation.requestCancel()
                            exportStatusText = "Cancelling export..."
                        }
                    }
                    .font(.system(.caption2, design: .monospaced).bold())
                    .buttonStyle(.glass)
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 10)
                .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 14, style: .continuous))
                .padding(.horizontal, 16)
                .padding(.top, 64)
                .transition(.opacity)
            }
        }
        .sheet(isPresented: $showImportURLSheet) {
            NavigationStack {
                VStack(alignment: .leading, spacing: 14) {
                    Text("Enter COPC URL")
                        .font(.system(.headline, design: .monospaced))
                        .foregroundStyle(.primary)

                    TextField("https://example.com/data.copc.laz", text: $importURLText)
                        .textInputAutocapitalization(.never)
                        .disableAutocorrection(true)
                        .keyboardType(.URL)
                        .font(.system(.caption, design: .monospaced))
                        .padding(10)
                        .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 10, style: .continuous))

                    Text("HTTPS direct links are supported in v1.")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.secondary)

                    Text("Quick Samples")
                        .font(.system(.caption, design: .monospaced).weight(.bold))
                        .foregroundStyle(.primary)
                        .padding(.top, 2)

                    VStack(alignment: .leading, spacing: 8) {
                        ForEach(importSampleURLs, id: \.url) { sample in
                            Button(action: {
                                emitTapHaptic()
                                importURLText = sample.url
                            }) {
                                HStack(spacing: 8) {
                                    Image(systemName: "link")
                                        .font(.system(size: 11, weight: .semibold))
                                    Text(sample.title)
                                        .font(.system(.caption2, design: .monospaced).weight(.semibold))
                                        .lineLimit(1)
                                    Spacer(minLength: 0)
                                }
                                .padding(.horizontal, 10)
                                .padding(.vertical, 7)
                                .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 9, style: .continuous))
                            }
                            .buttonStyle(.plain)
                            .foregroundStyle(.primary)
                        }
                    }

                    Spacer(minLength: 0)
                }
                .padding(16)
                .toolbar {
                    ToolbarItem(placement: .cancellationAction) {
                        Button("Cancel") {
                            emitTapHaptic()
                            showImportURLSheet = false
                        }
                    }
                    ToolbarItem(placement: .confirmationAction) {
                        Button("Load") {
                            emitTapHaptic()
                            startImportFromURL()
                        }
                        .buttonStyle(.borderedProminent)
                        .keyboardShortcut(.defaultAction)
                    }
                }
            }
            .tint(.blue)
            .presentationDetents([.fraction(0.46), .large])
            .presentationDragIndicator(.visible)
            .presentationBackground(.thinMaterial)
        }
        .sheet(isPresented: $showShareSheet) {
            if !exportURLs.isEmpty {
                ShareSheet(activityItems: exportURLs)
            }
        }
        .confirmationDialog(
            "Choose Export Format",
            isPresented: $showExportFormatPicker,
            titleVisibility: .visible
        ) {
            if let session {
                Button("LAZ") {
                    emitTapHaptic()
                    startExport(session: session, format: .laz)
                }
                .tint(.blue)
                .keyboardShortcut(.defaultAction)
                Button("PLY Binary Fast") {
                    emitTapHaptic()
                    startExport(session: session, format: .plyBinaryLittleEndian)
                }
                Button("PLY Text ASCII") {
                    emitTapHaptic()
                    startExport(session: session, format: .plyAscii)
                }
                Button("PDF Report") {
                    emitTapHaptic()
                    startExport(session: session, format: .pdfReport)
                }
            }
            Button("Cancel", role: .cancel) {}
        }
        .confirmationDialog(
            "Color By",
            isPresented: $showColorModePicker,
            titleVisibility: .visible
        ) {
            ForEach(CloudColorMode.allCases, id: \.rawValue) { mode in
                Button(selectedColorMode == mode ? "\(mode.title) ✓" : mode.title) {
                    emitTapHaptic()
                    selectedColorMode = mode
                }
            }
            Button("Cancel", role: .cancel) {}
        }
        .alert("Export Failed", isPresented: $showExportError) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("Could not export this scan. Please try again.")
        }
        .alert("Export Cancelled", isPresented: $showExportCancelledNotice) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("The export was cancelled.")
        }
        .alert("Import Failed", isPresented: $showImportError) {
            Button("OK", role: .cancel) { }
        } message: {
            Text(importErrorText)
        }
        .alert("Import Cancelled", isPresented: $showImportCancelledNotice) {
            Button("OK", role: .cancel) { }
        } message: {
            Text("The import was cancelled.")
        }
        .onAppear {
            guard session == nil else { return }
            guard engine.activePointCount == 0 else { return }
            guard !isImporting, !showImportURLSheet else { return }
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.12) {
                if session == nil && engine.activePointCount == 0 && !isImporting {
                    showImportURLSheet = true
                }
            }
        }
    }

    @ViewBuilder
    private func infoGlass(_ text: String, tint: Color = .primary) -> some View {
        Button(action: {}) {
            Text(text)
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(tint)
        }
        .buttonStyle(.glass)
        .disabled(true)
    }

    @ViewBuilder
    private var colorModeMenu: some View {
        Button(action: {
            emitTapHaptic()
            showColorModePicker = true
        }) {
            HStack(spacing: 6) {
                Image(systemName: "paintpalette")
                    .font(.system(size: 11, weight: .semibold))
                Text("COLOR")
                    .font(.system(.caption2, design: .monospaced).weight(.bold))
                Text(colorModeSummaryText)
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
            }
            .padding(.horizontal, 10)
            .frame(height: 30)
        }
        .buttonStyle(.glass)
    }

    private var colorModeSummaryText: String {
        if selectedColorMode == .auto {
            return hasUsableRGB ? "AUTO→RGB" : "AUTO→ELEV"
        }
        return effectiveColorMode.title
    }

    private func emitTapHaptic() {
        buttonHaptic.prepare()
        buttonHaptic.impactOccurred(intensity: 1.0)
    }

    @ViewBuilder
    private func iconActionButton(_ icon: String, label: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: icon)
                .font(.system(.caption, design: .monospaced).bold())
                .frame(width: 36, height: 36)
        }
        .buttonStyle(.glass)
        .accessibilityLabel(label)
    }

    @ViewBuilder
    private func measureToolButton(title: String, tool: MeasureController.MeasureTool) -> some View {
        let isActive = measureController.selectedTool == tool
        if isActive {
            Button(action: {
                emitTapHaptic()
                measureController.setTool(tool)
            }) {
                Text(title.uppercased())
                    .font(.system(.caption2, design: .monospaced).weight(.bold))
                    .padding(.horizontal, 8)
                    .frame(height: 28)
            }
            .buttonStyle(.glassProminent)
            .tint(.blue)
            .accessibilityLabel(title)
        } else {
            Button(action: {
                emitTapHaptic()
                measureController.setTool(tool)
            }) {
                Text(title.uppercased())
                    .font(.system(.caption2, design: .monospaced).weight(.bold))
                    .padding(.horizontal, 8)
                    .frame(height: 28)
            }
            .buttonStyle(.glass)
            .tint(.gray)
            .accessibilityLabel(title)
        }
    }

    @ViewBuilder
    private func analysisResultCard(_ card: MeasureController.AnalysisCardModel) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                Text("M\(card.measurementOrdinal) \(card.tool == .crossSection ? "CROSS SECTION" : "ELEVATION PROFILE")")
                    .font(.system(.caption2, design: .monospaced).weight(.bold))
                    .foregroundStyle(.primary)
                Spacer(minLength: 0)
                Text("\(card.resultIndex)/\(card.resultCount)")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
                analysisNavButton("chevron.left") {
                    emitTapHaptic()
                    measureController.selectPreviousAnalysisCard()
                }
                analysisNavButton("chevron.right") {
                    emitTapHaptic()
                    measureController.selectNextAnalysisCard()
                }
                analysisNavButton("xmark") {
                    emitTapHaptic()
                    measureController.dismissAnalysisCard()
                }
            }

            if card.canAdjustStation {
                HStack(spacing: 8) {
                    Text("Station")
                        .font(.system(.caption2, design: .monospaced).weight(.bold))
                        .foregroundStyle(.secondary)
                    Slider(
                        value: Binding(
                            get: { Double(card.stationFraction) },
                            set: { measureController.updateCurrentAnalysisStation(Float($0)) }
                        ),
                        in: 0...1
                    )
                    .tint(.blue)
                    Text("\(Int((card.stationFraction * 100).rounded()))%")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundStyle(.primary)
                        .frame(width: 36, alignment: .trailing)
                }
            }

            analysisChart(card)
                .frame(height: 120)

            if card.tool == .crossSection {
                Text("L \(formatFloat(card.pathLength))m • W \(formatFloat(card.swathWidthMeters ?? 0))m • Slice \(card.sectionPointCount ?? 0) • Corridor \(card.corridorPointCount)")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
                Text("Bin \(formatOptional(card.profileBinWidthMeters))m • Win \(formatOptional(card.stationWindowMeters))m • Cloud \(card.sourceSampleCount)")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
                Text("Min/Max \(formatOptional(card.minElevation))/\(formatOptional(card.maxElevation))m • Relief \(formatOptional(card.relief))m")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
            } else {
                Text("L \(formatFloat(card.pathLength))m • Min/Max \(formatOptional(card.minElevation))/\(formatOptional(card.maxElevation))m")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
                Text("Bin \(formatOptional(card.profileBinWidthMeters))m • Corridor \(card.corridorPointCount) • Cloud \(card.sourceSampleCount)")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
                Text("Rise/Fall \(formatOptional(card.totalRise))/\(formatOptional(card.totalFall))m • Slope \(formatOptional(card.averageSlopePercent))%/\(formatOptional(card.maxSlopePercent))%")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(.secondary)
            }
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12, style: .continuous))
    }

    @ViewBuilder
    private func analysisChart(_ card: MeasureController.AnalysisCardModel) -> some View {
        GeometryReader { proxy in
            Canvas { context, _ in
                let width = proxy.size.width
                let height = proxy.size.height
                let inset: CGFloat = 8
                let rect = CGRect(x: inset, y: inset, width: max(1, width - inset * 2), height: max(1, height - inset * 2))
                context.fill(Path(roundedRect: rect, cornerRadius: 8), with: .color(.black.opacity(0.12)))

                for idx in 1...3 {
                    let y = rect.minY + (CGFloat(idx) / 4.0) * rect.height
                    var grid = Path()
                    grid.move(to: CGPoint(x: rect.minX, y: y))
                    grid.addLine(to: CGPoint(x: rect.maxX, y: y))
                    context.stroke(grid, with: .color(.white.opacity(0.14)), lineWidth: 0.6)
                }

                if card.tool == .crossSection, card.sectionScatter.count >= 3 {
                    let xs = card.sectionScatter.map(\.x)
                    let ys = card.sectionScatter.map(\.y)
                    guard let minX = xs.min(), let maxX = xs.max(), let minY = ys.min(), let maxY = ys.max() else { return }
                    let spanX = max(0.001, maxX - minX)
                    let spanY = max(0.001, maxY - minY)
                    for p in card.sectionScatter {
                        let nx = (p.x - minX) / spanX
                        let ny = (p.y - minY) / spanY
                        let mapped = CGPoint(
                            x: rect.minX + CGFloat(nx) * rect.width,
                            y: rect.maxY - CGFloat(ny) * rect.height
                        )
                        let pointColor = Color(uiColor: UIColor(
                            red: CGFloat(max(0, min(1, p.color.x))),
                            green: CGFloat(max(0, min(1, p.color.y))),
                            blue: CGFloat(max(0, min(1, p.color.z))),
                            alpha: 1
                        ))
                        context.fill(
                            Path(ellipseIn: CGRect(x: mapped.x - 1.2, y: mapped.y - 1.2, width: 2.4, height: 2.4)),
                            with: .color(pointColor.opacity(0.88))
                        )
                    }
                } else if card.profileSamples.count >= 2 {
                    let minD = card.profileSamples.first?.distance ?? 0
                    let maxD = card.profileSamples.last?.distance ?? 1
                    let minY = card.profileSamples.map(\.elevation).min() ?? 0
                    let maxY = card.profileSamples.map(\.elevation).max() ?? 1
                    let spanD = max(0.001, maxD - minD)
                    let spanY = max(0.001, maxY - minY)
                    var line = Path()
                    for (idx, sample) in card.profileSamples.enumerated() {
                        let nx = (sample.distance - minD) / spanD
                        let ny = (sample.elevation - minY) / spanY
                        let mapped = CGPoint(
                            x: rect.minX + CGFloat(nx) * rect.width,
                            y: rect.maxY - CGFloat(ny) * rect.height
                        )
                        if idx == 0 {
                            line.move(to: mapped)
                        } else {
                            line.addLine(to: mapped)
                        }
                    }
                    context.stroke(line, with: .color(card.tool == .elevationProfile ? .blue : .green), lineWidth: 1.8)
                }
            }
        }
    }

    @ViewBuilder
    private func analysisNavButton(_ icon: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: icon)
                .font(.system(size: 10, weight: .bold))
                .frame(width: 22, height: 22)
        }
        .buttonStyle(.glass)
    }

    private func formatFloat(_ value: Float, digits: Int = 2) -> String {
        String(format: "%.\(digits)f", value)
    }

    private func formatOptional(_ value: Float?, digits: Int = 2) -> String {
        guard let value else { return "-" }
        return String(format: "%.\(digits)f", value)
    }

    private func formatStorage(_ bytes: Int64) -> String {
        let formatter = ByteCountFormatter()
        formatter.allowedUnits = [.useKB, .useMB, .useGB]
        formatter.countStyle = .file
        formatter.includesUnit = true
        formatter.isAdaptive = true
        return formatter.string(fromByteCount: bytes).uppercased()
    }

    private func formatDate(_ date: Date) -> String {
        let formatter = DateFormatter()
        formatter.dateFormat = "MMM d, h:mm a"
        return formatter.string(from: date).uppercased()
    }

    private func formatNumber(_ value: Int) -> String {
        let formatter = NumberFormatter()
        formatter.numberStyle = .decimal
        return formatter.string(from: NSNumber(value: value)) ?? "\(value)"
    }

    private func formatMeters(_ value: Double) -> String {
        let formatter = NumberFormatter()
        formatter.maximumFractionDigits = 1
        formatter.minimumFractionDigits = 1
        let formatted = formatter.string(from: NSNumber(value: max(0, value))) ?? "\(value)"
        return "\(formatted)m"
    }

    private var shouldShowInfoPanel: Bool {
        session != nil || engine.activePointCount > 0 || isImporting
    }

    private var panelTitle: String {
        if let session {
            return session.name
        }
        return importedSourceLabel ?? "Remote Point Cloud"
    }

    private var panelSubtitle: String {
        if let session {
            return "UPDATED \(formatDate(session.updatedAt))"
        }
        if isImporting {
            return "IMPORT IN PROGRESS"
        }
        return importedSourceURLText ?? "Remote URL"
    }

    private var panelPointCount: Int {
        if let session {
            return session.pointCount
        }
        return engine.activePointCount
    }

    private func remoteSourceName(from url: URL) -> String {
        let raw = url.deletingPathExtension().lastPathComponent
        if !raw.isEmpty { return raw }
        if let host = url.host, !host.isEmpty { return host }
        return "Remote Point Cloud"
    }

    private func remoteSourceURLDisplay(from url: URL) -> String {
        let host = url.host ?? "remote"
        let path = url.path
        let combined = path.isEmpty || path == "/" ? host : "\(host)\(path)"
        if combined.count <= 46 { return combined }
        return "\(combined.prefix(43))..."
    }

    @ViewBuilder
    private var georefStatusPill: some View {
        Button(action: {
            emitTapHaptic()
            showGeorefPopover = true
        }) {
            Image(systemName: georefIconName)
                .font(.system(.caption, design: .monospaced).bold())
                .foregroundStyle(georefIconTint)
        }
        .buttonStyle(.glass)
        .accessibilityLabel("Georeference Details")
        .popover(isPresented: $showGeorefPopover, attachmentAnchor: .point(.top), arrowEdge: .bottom) {
            georefDetailsPopover
                .presentationCompactAdaptation(.popover)
        }
    }

    @ViewBuilder
    private var georefDetailsPopover: some View {
        VStack(alignment: .leading, spacing: 10) {
            georefDetailRow(label: "Mode", value: georefMode)
            georefDetailRow(label: "EPSG / CRS", value: georefEPSGAndCRS)
            georefDetailRow(label: "Accuracy", value: georefHorizontalAccuracy)
            georefDetailRow(label: "Lat / Lon", value: georefLatLon)
            georefDetailRow(label: "Altitude", value: georefAltitude)
            georefDetailRow(label: "Heading", value: georefHeading)

            Text("Approximate only. Use GCP/checkpoints for survey-grade georeferencing.")
                .font(.system(.caption2, design: .monospaced))
                .foregroundStyle(.secondary)
                .padding(.top, 4)
        }
        .padding(14)
        .frame(width: 320, alignment: .leading)
        .background(.regularMaterial)
    }

    @ViewBuilder
    private func georefDetailRow(label: String, value: String) -> some View {
        HStack(alignment: .firstTextBaseline, spacing: 8) {
            Text(label.uppercased())
                .font(.system(.caption2, design: .monospaced).weight(.bold))
                .foregroundStyle(.secondary)
                .frame(width: 82, alignment: .leading)
            Text(value)
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.primary)
        }
    }

    private func inferredEPSG(from location: CaptureLocationMetadata) -> Int? {
        let latitude = location.latitude
        let longitude = location.longitude
        guard latitude.isFinite, longitude.isFinite else { return nil }
        guard abs(latitude) <= 84.0, abs(longitude) <= 180.0 else { return nil }
        let zone = max(1, min(60, Int(floor((longitude + 180.0) / 6.0)) + 1))
        return (latitude >= 0 ? 32600 : 32700) + zone
    }

    private func inferredCRSName(from location: CaptureLocationMetadata) -> String? {
        guard inferredEPSG(from: location) != nil else { return nil }
        let zone = max(1, min(60, Int(floor((location.longitude + 180.0) / 6.0)) + 1))
        let hemisphere = location.latitude >= 0 ? "N" : "S"
        return "WGS 84 / UTM zone \(zone)\(hemisphere)"
    }

    private var georefIconName: String {
        captureMetadata?.location == nil ? "location.slash.fill" : "location.fill"
    }

    private var georefIconTint: Color {
        captureMetadata?.location == nil ? .red : .green
    }

    private var georefMode: String {
        captureMetadata?.location == nil ? "Local Only (No GPS)" : "GPS Approximate"
    }

    private var georefEPSGAndCRS: String {
        guard let location = captureMetadata?.location else { return "N/A" }
        guard let epsg = inferredEPSG(from: location) else { return "Unknown" }
        let crs = inferredCRSName(from: location) ?? "Unknown CRS"
        return "EPSG:\(epsg) • \(crs)"
    }

    private var georefHorizontalAccuracy: String {
        guard let location = captureMetadata?.location else { return "N/A" }
        guard location.horizontalAccuracy > 0 else { return "Unknown" }
        return formatMeters(location.horizontalAccuracy)
    }

    private var georefLatLon: String {
        guard let location = captureMetadata?.location else { return "N/A" }
        return "\(formatCoordinate(location.latitude)), \(formatCoordinate(location.longitude))"
    }

    private var georefAltitude: String {
        guard let location = captureMetadata?.location else { return "N/A" }
        return String(format: "%.1fm", location.altitude)
    }

    private var georefHeading: String {
        guard let location = captureMetadata?.location,
              let heading = location.headingDegrees,
              heading.isFinite else { return "N/A" }
        let normalized = heading.truncatingRemainder(dividingBy: 360)
        let wrapped = normalized >= 0 ? normalized : (normalized + 360)
        return String(format: "%.1f°", wrapped)
    }

    private func formatCoordinate(_ value: Double) -> String {
        String(format: "%.6f", value)
    }

    private func startImportFromURL() {
        guard !isExporting, !isImporting else { return }
        let trimmed = importURLText.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let url = URL(string: trimmed),
              let scheme = url.scheme?.lowercased(),
              scheme == "http" || scheme == "https" else {
            importErrorText = "Please enter a valid HTTP(S) URL."
            showImportError = true
            return
        }

        showImportURLSheet = false
        importCancellation = ExportCancellationFlag()
        importProgress = 0
        importStatusText = "Checking remote file..."
        showImportCancelledNotice = false
        isImporting = true
        selectedColorMode = .auto

        engine.importPointCloudFromURL(
            url,
            progress: { progress, message in
                DispatchQueue.main.async {
                    guard isImporting else { return }
                    importProgress = min(max(progress, 0), 1)
                    importStatusText = message
                }
            },
            isCancelled: {
                importCancellation.isCancelled
            },
            completion: { success, message in
                isImporting = false
                if success {
                    importedSourceLabel = remoteSourceName(from: url)
                    importedSourceURLText = remoteSourceURLDisplay(from: url)
                    measureController.resetForNewPointCloud()
                    recenterSignal &+= 1
                } else if importCancellation.isCancelled || (message?.localizedCaseInsensitiveContains("cancel") == true) {
                    showImportCancelledNotice = true
                } else {
                    importErrorText = message ?? "Could not import this URL."
                    showImportError = true
                }
            }
        )
    }

    private func startExport(session: ScanSession, format: PointCloudEngine.ExportFormat) {
        guard let onExport else { return }
        guard !isExporting, !isImporting else { return }
        exportCancellation = ExportCancellationFlag()
        exportProgress = 0
        switch format {
        case .laz:
            exportStatusText = "Generating LAZ..."
        case .plyBinaryLittleEndian:
            exportStatusText = "Generating binary PLY..."
        case .plyAscii:
            exportStatusText = "Generating ASCII PLY..."
        case .pdfReport:
            exportStatusText = "Generating PDF report..."
        }
        isExporting = true
        showExportCancelledNotice = false
        DispatchQueue.global(qos: .utility).async {
            let artifact = onExport(
                session,
                format,
                measureController.exportMeasurements(),
                { progress, message in
                    DispatchQueue.main.async {
                        guard isExporting else { return }
                        exportProgress = min(max(progress, 0), 1)
                        exportStatusText = message
                    }
                },
                {
                    exportCancellation.isCancelled
                }
            )
            DispatchQueue.main.async {
                isExporting = false
                if let artifact {
                    exportURLs = artifact.shareItems
                    showShareSheet = true
                } else if exportCancellation.isCancelled {
                    showExportCancelledNotice = true
                } else {
                    showExportError = true
                }
            }
        }
    }
}

final class MeasureController: ObservableObject {
    enum MeasureTool: String, CaseIterable {
        case path
        case crossSection
        case elevationProfile
    }

    struct MeasurementPath {
        let id: UUID
        let createdAt: Date
        let tool: MeasureTool
        var vertices: [SIMD3<Float>]
        var isClosed: Bool
        var swathWidthMeters: Float?
    }

    struct OverlayLabel: Identifiable {
        let id = UUID()
        let text: String
        let position: CGPoint
        let tint: Color
    }

    struct ScreenSegment {
        let a: CGPoint
        let b: CGPoint
        let tint: Color
        let width: CGFloat
    }

    struct ProfileSample: Hashable {
        let distance: Float
        let elevation: Float
    }

    struct CloudSamplePoint {
        let position: SIMD3<Float>
        let color: SIMD3<Float> // normalized 0...1
    }

    struct SectionScatterPoint {
        let x: Float
        let y: Float
        let color: SIMD3<Float> // normalized 0...1
    }

    struct AnalysisCardModel {
        let measurementID: UUID
        let measurementOrdinal: Int
        let tool: MeasureTool
        let createdAt: Date
        let pathLength: Float
        let swathWidthMeters: Float?
        let profileSamples: [ProfileSample]
        let sectionScatter: [SectionScatterPoint]
        let sectionPointCount: Int?
        let sourceSampleCount: Int
        let corridorPointCount: Int
        let profileBinWidthMeters: Float?
        let stationWindowMeters: Float?
        let minElevation: Float?
        let maxElevation: Float?
        let relief: Float?
        let totalRise: Float?
        let totalFall: Float?
        let averageSlopePercent: Float?
        let maxSlopePercent: Float?
        let stationFraction: Float
        let canAdjustStation: Bool
        let resultIndex: Int
        let resultCount: Int
    }

    @Published var isEnabled = false
    @Published var hasSnapPoint = false
    @Published var projectedActiveVertices: [CGPoint] = []
    @Published var projectedCommittedVertices: [CGPoint] = []
    @Published var projectedActiveSegments: [ScreenSegment] = []
    @Published var projectedCommittedSegments: [ScreenSegment] = []
    @Published var projectedActiveSwathSegments: [ScreenSegment] = []
    @Published var projectedCommittedSwathSegments: [ScreenSegment] = []
    @Published var projectedSnapPoint: CGPoint?
    @Published var overlayLabels: [OverlayLabel] = []
    @Published var selectedTool: MeasureTool = .path
    @Published var swathWidthMeters: Double = 0.25
    @Published var currentAnalysisCard: AnalysisCardModel?

    var hasActiveVertices: Bool { !activeVertices.isEmpty }
    var canCloseCurrentMeasurement: Bool { selectedTool == .path && activeVertices.count >= 3 }
    var canCommitOpenMeasurement: Bool { selectedTool == .path && activeVertices.count >= 2 }
    var canFinalizeCurrentTool: Bool { activeVertices.count >= 2 }

    private struct CorridorPoint {
        let alongDistance: Float
        let lateralDistanceSigned: Float
        let elevation: Float
        let color: SIMD3<Float>
    }

    private struct ComputedAnalysis {
        var profileSamples: [ProfileSample]
        var corridorPoints: [CorridorPoint]
        var pathLength: Float
        var swathWidthMeters: Float?
        var sourceSampleCount: Int
        var corridorPointCount: Int
        var profileBinWidthMeters: Float?
        var stationWindowMeters: Float?
        var minElevation: Float?
        var maxElevation: Float?
        var relief: Float?
        var totalRise: Float?
        var totalFall: Float?
        var averageSlopePercent: Float?
        var maxSlopePercent: Float?
        var stationFraction: Float
        var sectionHalfWindow: Float
    }

    private var snapPoint: SIMD3<Float>?
    private var activeVertices: [SIMD3<Float>] = []
    private var committedPaths: [MeasurementPath] = []
    private var sampledCloudPoints: [CloudSamplePoint] = []
    private var analysisByPathID: [UUID: ComputedAnalysis] = [:]
    private var selectedAnalysisPathID: UUID?

    func currentSnapWorldPoint() -> SIMD3<Float>? {
        snapPoint
    }

    func toggleMode() {
        isEnabled.toggle()
        if !isEnabled {
            snapPoint = nil
            hasSnapPoint = false
            projectedActiveVertices = []
            projectedCommittedVertices = []
            projectedActiveSegments = []
            projectedCommittedSegments = []
            projectedActiveSwathSegments = []
            projectedCommittedSwathSegments = []
            projectedSnapPoint = nil
            overlayLabels = []
            selectedTool = .path
            currentAnalysisCard = nil
            selectedAnalysisPathID = nil
        }
    }

    func setTool(_ tool: MeasureTool) {
        selectedTool = tool
    }

    func resetForNewPointCloud() {
        snapPoint = nil
        hasSnapPoint = false
        activeVertices = []
        committedPaths = []
        sampledCloudPoints = []
        analysisByPathID = [:]
        selectedAnalysisPathID = nil
        currentAnalysisCard = nil
        projectedActiveVertices = []
        projectedCommittedVertices = []
        projectedActiveSegments = []
        projectedCommittedSegments = []
        projectedActiveSwathSegments = []
        projectedCommittedSwathSegments = []
        projectedSnapPoint = nil
        overlayLabels = []
        if selectedTool != .path {
            selectedTool = .path
        }
    }

    func addVertex() {
        guard isEnabled, let snapPoint else { return }
        activeVertices.append(snapPoint)
    }

    func undoLastVertex() {
        guard !activeVertices.isEmpty else { return }
        activeVertices.removeLast()
    }

    func selectPreviousAnalysisCard() {
        let ids = analysisDisplayOrder()
        guard !ids.isEmpty else { return }
        guard let selected = selectedAnalysisPathID, let idx = ids.firstIndex(of: selected) else {
            selectedAnalysisPathID = ids.last
            rebuildCurrentAnalysisCard()
            return
        }
        let prevIndex = idx == 0 ? (ids.count - 1) : (idx - 1)
        selectedAnalysisPathID = ids[prevIndex]
        rebuildCurrentAnalysisCard()
    }

    func selectNextAnalysisCard() {
        let ids = analysisDisplayOrder()
        guard !ids.isEmpty else { return }
        guard let selected = selectedAnalysisPathID, let idx = ids.firstIndex(of: selected) else {
            selectedAnalysisPathID = ids.first
            rebuildCurrentAnalysisCard()
            return
        }
        let nextIndex = (idx + 1) % ids.count
        selectedAnalysisPathID = ids[nextIndex]
        rebuildCurrentAnalysisCard()
    }

    func dismissAnalysisCard() {
        selectedAnalysisPathID = nil
        currentAnalysisCard = nil
    }

    func updateCurrentAnalysisStation(_ fraction: Float) {
        guard let selectedAnalysisPathID, var analysis = analysisByPathID[selectedAnalysisPathID] else { return }
        analysis.stationFraction = min(max(fraction, 0), 1)
        analysisByPathID[selectedAnalysisPathID] = analysis
        rebuildCurrentAnalysisCard()
    }

    func closeCurrentMeasurement() {
        guard canCloseCurrentMeasurement else { return }
        committedPaths.append(MeasurementPath(
            id: UUID(),
            createdAt: Date(),
            tool: .path,
            vertices: activeVertices,
            isClosed: true,
            swathWidthMeters: nil
        ))
        activeVertices = []
        rebuildCurrentAnalysisCard()
    }

    func commitOpenMeasurementAndStartNew() {
        guard canCommitOpenMeasurement else { return }
        committedPaths.append(MeasurementPath(
            id: UUID(),
            createdAt: Date(),
            tool: .path,
            vertices: activeVertices,
            isClosed: false,
            swathWidthMeters: nil
        ))
        activeVertices = []
        rebuildCurrentAnalysisCard()
    }

    func finalizeActiveMeasurementForCurrentTool() {
        guard canFinalizeCurrentTool else { return }
        switch selectedTool {
        case .path:
            commitOpenMeasurementAndStartNew()
        case .crossSection:
            let newPath = MeasurementPath(
                id: UUID(),
                createdAt: Date(),
                tool: .crossSection,
                vertices: activeVertices,
                isClosed: false,
                swathWidthMeters: Float(swathWidthMeters)
            )
            committedPaths.append(newPath)
            refreshAnalysis(for: newPath)
            selectedAnalysisPathID = newPath.id
            rebuildCurrentAnalysisCard()
            activeVertices = []
        case .elevationProfile:
            let newPath = MeasurementPath(
                id: UUID(),
                createdAt: Date(),
                tool: .elevationProfile,
                vertices: activeVertices,
                isClosed: false,
                swathWidthMeters: nil
            )
            committedPaths.append(newPath)
            refreshAnalysis(for: newPath)
            selectedAnalysisPathID = newPath.id
            rebuildCurrentAnalysisCard()
            activeVertices = []
        }
    }

    private struct ElevationStats {
        let minElevation: Float
        let maxElevation: Float
        let relief: Float
        let totalRise: Float
        let totalFall: Float
        let averageSlopePercent: Float
        let maxSlopePercent: Float
    }

    private func computeElevationStats(vertices: [SIMD3<Float>]) -> ElevationStats? {
        guard vertices.count >= 2 else { return nil }
        var minY = Float.greatestFiniteMagnitude
        var maxY = -Float.greatestFiniteMagnitude
        var totalRise: Float = 0
        var totalFall: Float = 0
        var maxSlopePercent: Float = 0
        var totalHorizontalDistance: Float = 0

        for idx in 1..<vertices.count {
            let a = vertices[idx - 1]
            let b = vertices[idx]
            minY = min(minY, a.y, b.y)
            maxY = max(maxY, a.y, b.y)

            let deltaY = b.y - a.y
            if deltaY >= 0 {
                totalRise += deltaY
            } else {
                totalFall += abs(deltaY)
            }

            let horizontalDistance = simd_length(SIMD2<Float>(b.x - a.x, b.z - a.z))
            totalHorizontalDistance += horizontalDistance
            if horizontalDistance > 1e-6 {
                let slopePercent = abs(deltaY / horizontalDistance) * 100
                maxSlopePercent = max(maxSlopePercent, slopePercent)
            }
        }

        guard minY.isFinite, maxY.isFinite else { return nil }
        let relief = max(0, maxY - minY)
        let averageSlopePercent = totalHorizontalDistance > 1e-6
            ? ((totalRise + totalFall) / totalHorizontalDistance) * 100
            : 0
        return ElevationStats(
            minElevation: minY,
            maxElevation: maxY,
            relief: relief,
            totalRise: totalRise,
            totalFall: totalFall,
            averageSlopePercent: averageSlopePercent,
            maxSlopePercent: maxSlopePercent
        )
    }

    private func reportType(for path: MeasurementPath) -> ScanReportMeasurementType {
        switch path.tool {
        case .path:
            if path.isClosed { return .closedArea }
            return path.vertices.count == 2 ? .distance : .polyline
        case .crossSection:
            return .crossSection
        case .elevationProfile:
            return .elevationProfile
        }
    }

    private func analysisDisplayOrder() -> [UUID] {
        committedPaths
            .filter { $0.tool == .crossSection || $0.tool == .elevationProfile }
            .map(\.id)
            .filter { analysisByPathID[$0] != nil }
    }

    private func signedLateral(
        segment: SIMD2<Float>,
        vectorFromStart: SIMD2<Float>,
        distance: Float
    ) -> Float {
        guard distance > 0 else { return 0 }
        let cross = (segment.x * vectorFromStart.y) - (segment.y * vectorFromStart.x)
        let sign: Float = cross >= 0 ? 1 : -1
        return distance * sign
    }

    private func fallbackProfile(for vertices: [SIMD3<Float>]) -> [ProfileSample] {
        guard vertices.count >= 2 else { return [] }
        var samples: [ProfileSample] = [ProfileSample(distance: 0, elevation: vertices[0].y)]
        var total: Float = 0
        for idx in 1..<vertices.count {
            let a = SIMD2<Float>(vertices[idx - 1].x, vertices[idx - 1].z)
            let b = SIMD2<Float>(vertices[idx].x, vertices[idx].z)
            total += simd_length(b - a)
            samples.append(ProfileSample(distance: total, elevation: vertices[idx].y))
        }
        return samples
    }

    private func median(_ values: [Float]) -> Float {
        let sorted = values.sorted()
        guard !sorted.isEmpty else { return 0 }
        if sorted.count % 2 == 1 { return sorted[sorted.count / 2] }
        return (sorted[(sorted.count / 2) - 1] + sorted[sorted.count / 2]) * 0.5
    }

    private func percentile(_ values: [Float], fraction: Float) -> Float {
        let sorted = values.sorted()
        guard !sorted.isEmpty else { return 0 }
        let p = max(0, min(1, fraction))
        let idx = Int(round(p * Float(sorted.count - 1)))
        return sorted[max(0, min(sorted.count - 1, idx))]
    }

    private func fillMissing(_ values: inout [Float?]) {
        guard let firstValid = values.firstIndex(where: { $0 != nil }) else { return }
        for idx in 0..<firstValid {
            values[idx] = values[firstValid]
        }
        var lastKnown = firstValid
        for idx in (firstValid + 1)..<values.count {
            if values[idx] != nil {
                let start = lastKnown
                let end = idx
                let gap = end - start
                if gap > 1, let startValue = values[start], let endValue = values[end] {
                    for g in 1..<gap {
                        let t = Float(g) / Float(gap)
                        values[start + g] = startValue + ((endValue - startValue) * t)
                    }
                }
                lastKnown = idx
            }
        }
        for idx in (lastKnown + 1)..<values.count {
            values[idx] = values[lastKnown]
        }
    }

    private func smoothSeries(_ values: [Float], radius: Int) -> [Float] {
        guard !values.isEmpty, radius > 0 else { return values }
        var result = values
        for idx in values.indices {
            var weightedSum: Float = 0
            var weightTotal: Float = 0
            let start = max(values.startIndex, idx - radius)
            let end = min(values.endIndex - 1, idx + radius)
            for sampleIdx in start...end {
                let offset = abs(sampleIdx - idx)
                let weight = Float(radius - offset + 1)
                weightedSum += values[sampleIdx] * weight
                weightTotal += weight
            }
            result[idx] = weightTotal > 0 ? (weightedSum / weightTotal) : values[idx]
        }
        return result
    }

    private func computeProfileStats(_ profileSamples: [ProfileSample]) -> (
        pathLength: Float,
        minElevation: Float?,
        maxElevation: Float?,
        relief: Float?,
        totalRise: Float?,
        totalFall: Float?,
        averageSlopePercent: Float?,
        maxSlopePercent: Float?
    ) {
        guard profileSamples.count >= 2 else {
            return (0, nil, nil, nil, nil, nil, nil, nil)
        }
        let minElevation = profileSamples.map(\.elevation).min()
        let maxElevation = profileSamples.map(\.elevation).max()
        var totalRise: Float = 0
        var totalFall: Float = 0
        var maxSlope: Float = 0
        var totalHorizontal: Float = 0
        var totalAbsDelta: Float = 0
        for idx in 1..<profileSamples.count {
            let d = profileSamples[idx].distance - profileSamples[idx - 1].distance
            guard d > 1e-6 else { continue }
            let dz = profileSamples[idx].elevation - profileSamples[idx - 1].elevation
            if dz >= 0 {
                totalRise += dz
            } else {
                totalFall += abs(dz)
            }
            totalHorizontal += d
            totalAbsDelta += abs(dz)
            maxSlope = max(maxSlope, abs(dz / d) * 100)
        }
        let avgSlope = totalHorizontal > 1e-6 ? (totalAbsDelta / totalHorizontal * 100) : 0
        let length = max(0, profileSamples.last!.distance - profileSamples.first!.distance)
        return (
            length,
            minElevation,
            maxElevation,
            (minElevation != nil && maxElevation != nil) ? max(0, maxElevation! - minElevation!) : nil,
            totalRise,
            totalFall,
            avgSlope,
            maxSlope
        )
    }

    private func refreshAnalysis(for path: MeasurementPath) {
        guard path.tool == .crossSection || path.tool == .elevationProfile else { return }
        let analysis = buildAnalysis(for: path)
        analysisByPathID[path.id] = analysis
    }

    private func buildAnalysis(for path: MeasurementPath) -> ComputedAnalysis {
        guard path.vertices.count >= 2 else {
            return ComputedAnalysis(
                profileSamples: [],
                corridorPoints: [],
                pathLength: 0,
                swathWidthMeters: path.swathWidthMeters,
                sourceSampleCount: sampledCloudPoints.count,
                corridorPointCount: 0,
                profileBinWidthMeters: nil,
                stationWindowMeters: nil,
                minElevation: nil,
                maxElevation: nil,
                relief: nil,
                totalRise: nil,
                totalFall: nil,
                averageSlopePercent: nil,
                maxSlopePercent: nil,
                stationFraction: 0.5,
                sectionHalfWindow: max(0.06, min(1.2, (path.swathWidthMeters ?? 0.12) * 0.9))
            )
        }

        struct SegmentInfo {
            let a: SIMD2<Float>
            let v: SIMD2<Float>
            let len: Float
            let invLen2: Float
            let startDistance: Float
        }

        let profileSwath = path.tool == .crossSection ? max(0.04, path.swathWidthMeters ?? 0.12) : max(0.12, path.swathWidthMeters ?? 0.12)
        let profileHalf = profileSwath * 0.5
        let sectionHalfWindow = max(0.06, min(1.2, (path.swathWidthMeters ?? 0.12) * 0.9))

        var segments: [SegmentInfo] = []
        segments.reserveCapacity(path.vertices.count - 1)
        var cumulativeDistance: Float = 0
        for idx in 1..<path.vertices.count {
            let a = SIMD2<Float>(path.vertices[idx - 1].x, path.vertices[idx - 1].z)
            let b = SIMD2<Float>(path.vertices[idx].x, path.vertices[idx].z)
            let v = b - a
            let len = simd_length(v)
            guard len > 1e-6 else { continue }
            segments.append(
                SegmentInfo(
                    a: a,
                    v: v,
                    len: len,
                    invLen2: 1.0 / simd_length_squared(v),
                    startDistance: cumulativeDistance
                )
            )
            cumulativeDistance += len
        }

        guard !segments.isEmpty, cumulativeDistance > 1e-6 else {
            let profile = fallbackProfile(for: path.vertices)
            let stats = computeProfileStats(profile)
            let fallbackBinWidth: Float? = profile.count >= 2
                ? max(1e-6, stats.pathLength / Float(profile.count - 1))
                : nil
            return ComputedAnalysis(
                profileSamples: profile,
                corridorPoints: [],
                pathLength: stats.pathLength,
                swathWidthMeters: path.swathWidthMeters,
                sourceSampleCount: sampledCloudPoints.count,
                corridorPointCount: 0,
                profileBinWidthMeters: fallbackBinWidth,
                stationWindowMeters: sectionHalfWindow * 2,
                minElevation: stats.minElevation,
                maxElevation: stats.maxElevation,
                relief: stats.relief,
                totalRise: stats.totalRise,
                totalFall: stats.totalFall,
                averageSlopePercent: stats.averageSlopePercent,
                maxSlopePercent: stats.maxSlopePercent,
                stationFraction: 0.5,
                sectionHalfWindow: sectionHalfWindow
            )
        }

        var corridor: [CorridorPoint] = []
        corridor.reserveCapacity(8_000)
        for samplePoint in sampledCloudPoints {
            let point = samplePoint.position
            let p2 = SIMD2<Float>(point.x, point.z)
            var bestDistance = Float.greatestFiniteMagnitude
            var bestAlong: Float = 0
            var bestSigned: Float = 0
            for segment in segments {
                let w = p2 - segment.a
                let rawT = simd_dot(w, segment.v) * segment.invLen2
                let t = max(0, min(1, rawT))
                let projected = segment.a + (segment.v * t)
                let delta = p2 - projected
                let lateralDistance = simd_length(delta)
                if lateralDistance < bestDistance {
                    bestDistance = lateralDistance
                    bestAlong = segment.startDistance + (t * segment.len)
                    bestSigned = signedLateral(segment: segment.v, vectorFromStart: w, distance: lateralDistance)
                }
            }
            if bestDistance <= profileHalf {
                corridor.append(CorridorPoint(
                    alongDistance: bestAlong,
                    lateralDistanceSigned: bestSigned,
                    elevation: point.y,
                    color: samplePoint.color
                ))
            }
        }

        var profileSamples: [ProfileSample] = []
        var profileBinWidth: Float?
        if !corridor.isEmpty {
            let targetBinWidth = max(0.01, min(0.05, profileSwath * 0.35))
            let sampleCount = max(32, min(420, Int(ceil(Double(cumulativeDistance / targetBinWidth))) + 1))
            let binCount = max(2, sampleCount)
            let binWidth = cumulativeDistance / Float(binCount - 1)
            profileBinWidth = binWidth
            var bins = Array(repeating: [Float](), count: binCount)
            for sample in corridor {
                let idx = max(0, min(binCount - 1, Int(floor(sample.alongDistance / max(binWidth, 1e-6)))))
                bins[idx].append(sample.elevation)
            }
            var binElevations = Array<Float?>(repeating: nil, count: binCount)
            for idx in 0..<binCount where !bins[idx].isEmpty {
                if path.tool == .elevationProfile {
                    // Lower percentile reduces spikes from vertical clutter and produces a more natural terrain-like profile.
                    binElevations[idx] = percentile(bins[idx], fraction: 0.35)
                } else {
                    binElevations[idx] = median(bins[idx])
                }
            }
            fillMissing(&binElevations)
            if binElevations.contains(where: { $0 != nil }) {
                var values = binElevations.map { $0 ?? 0 }
                if path.tool == .elevationProfile {
                    values = smoothSeries(values, radius: 2)
                    values = smoothSeries(values, radius: 2)
                }
                for idx in 0..<binCount {
                    profileSamples.append(ProfileSample(distance: Float(idx) * binWidth, elevation: values[idx]))
                }
            }
        }
        if profileSamples.count < 2 {
            profileSamples = fallbackProfile(for: path.vertices)
            if profileSamples.count >= 2 {
                profileBinWidth = max(1e-6, cumulativeDistance / Float(profileSamples.count - 1))
            }
        }

        let stats = computeProfileStats(profileSamples)
        return ComputedAnalysis(
            profileSamples: profileSamples,
            corridorPoints: corridor,
            pathLength: stats.pathLength,
            swathWidthMeters: path.swathWidthMeters,
            sourceSampleCount: sampledCloudPoints.count,
            corridorPointCount: corridor.count,
            profileBinWidthMeters: profileBinWidth,
            stationWindowMeters: sectionHalfWindow * 2,
            minElevation: stats.minElevation,
            maxElevation: stats.maxElevation,
            relief: stats.relief,
            totalRise: stats.totalRise,
            totalFall: stats.totalFall,
            averageSlopePercent: stats.averageSlopePercent,
            maxSlopePercent: stats.maxSlopePercent,
            stationFraction: 0.5,
            sectionHalfWindow: sectionHalfWindow
        )
    }

    private func sectionScatter(for analysis: ComputedAnalysis) -> [SectionScatterPoint] {
        guard !analysis.corridorPoints.isEmpty else { return [] }
        let swath = max(0.04, analysis.swathWidthMeters ?? 0.12)
        let halfSwath = swath * 0.5
        let center = analysis.pathLength * analysis.stationFraction
        return analysis.corridorPoints
            .filter {
                abs($0.alongDistance - center) <= analysis.sectionHalfWindow &&
                abs($0.lateralDistanceSigned) <= halfSwath
            }
            .prefix(1200)
            .map {
                SectionScatterPoint(
                    x: $0.lateralDistanceSigned,
                    y: $0.elevation,
                    color: $0.color
                )
            }
    }

    private func sectionPointCount(for analysis: ComputedAnalysis) -> Int {
        guard !analysis.corridorPoints.isEmpty else { return 0 }
        let swath = max(0.04, analysis.swathWidthMeters ?? 0.12)
        let halfSwath = swath * 0.5
        let center = analysis.pathLength * analysis.stationFraction
        return analysis.corridorPoints.reduce(0) { count, point in
            let inSection = abs(point.alongDistance - center) <= analysis.sectionHalfWindow &&
                abs(point.lateralDistanceSigned) <= halfSwath
            return count + (inSection ? 1 : 0)
        }
    }

    private func rebuildCurrentAnalysisCard() {
        let ids = analysisDisplayOrder()
        guard !ids.isEmpty else {
            currentAnalysisCard = nil
            selectedAnalysisPathID = nil
            return
        }
        if selectedAnalysisPathID == nil || ids.contains(selectedAnalysisPathID!) == false {
            selectedAnalysisPathID = ids.last
        }
        guard let selectedAnalysisPathID,
              let path = committedPaths.first(where: { $0.id == selectedAnalysisPathID }),
              let analysis = analysisByPathID[selectedAnalysisPathID],
              let resultIndex = ids.firstIndex(of: selectedAnalysisPathID),
              let measurementOrdinal = committedPaths.firstIndex(where: { $0.id == selectedAnalysisPathID }).map({ $0 + 1 }) else {
            currentAnalysisCard = nil
            return
        }

        currentAnalysisCard = AnalysisCardModel(
            measurementID: selectedAnalysisPathID,
            measurementOrdinal: measurementOrdinal,
            tool: path.tool,
            createdAt: path.createdAt,
            pathLength: analysis.pathLength,
            swathWidthMeters: analysis.swathWidthMeters,
            profileSamples: analysis.profileSamples,
            sectionScatter: path.tool == .crossSection ? sectionScatter(for: analysis) : [],
            sectionPointCount: path.tool == .crossSection ? sectionPointCount(for: analysis) : nil,
            sourceSampleCount: analysis.sourceSampleCount,
            corridorPointCount: analysis.corridorPointCount,
            profileBinWidthMeters: analysis.profileBinWidthMeters,
            stationWindowMeters: analysis.stationWindowMeters,
            minElevation: analysis.minElevation,
            maxElevation: analysis.maxElevation,
            relief: analysis.relief,
            totalRise: analysis.totalRise,
            totalFall: analysis.totalFall,
            averageSlopePercent: analysis.averageSlopePercent,
            maxSlopePercent: analysis.maxSlopePercent,
            stationFraction: analysis.stationFraction,
            canAdjustStation: path.tool == .crossSection,
            resultIndex: resultIndex + 1,
            resultCount: ids.count
        )
    }

    func exportMeasurements() -> [ScanReportMeasurement] {
        committedPaths.map { path in
            let length = totalLength(vertices: path.vertices, closed: false)
            if path.tool == .path, path.isClosed {
                let perimeter = totalLength(vertices: path.vertices, closed: true)
                let area = polygonArea(vertices: path.vertices)
                return ScanReportMeasurement(
                    id: path.id.uuidString,
                    type: .closedArea,
                    createdAt: path.createdAt,
                    vertexCount: path.vertices.count,
                    vertices: path.vertices.map {
                        ScanReportVertex(x: Double($0.x), y: Double($0.y), z: Double($0.z))
                    },
                    lengthMeters: nil,
                    perimeterMeters: Double(perimeter),
                    areaSquareMeters: area.map(Double.init),
                    swathWidthMeters: nil,
                    minElevationMeters: nil,
                    maxElevationMeters: nil,
                    reliefMeters: nil,
                    totalRiseMeters: nil,
                    totalFallMeters: nil,
                    averageSlopePercent: nil,
                    maxSlopePercent: nil,
                    notes: nil
                )
            }

            let sampledAnalysis = analysisByPathID[path.id]
            let vertexStats = computeElevationStats(vertices: path.vertices)
            return ScanReportMeasurement(
                id: path.id.uuidString,
                type: reportType(for: path),
                createdAt: path.createdAt,
                vertexCount: path.vertices.count,
                vertices: path.vertices.map {
                    ScanReportVertex(x: Double($0.x), y: Double($0.y), z: Double($0.z))
                },
                lengthMeters: Double(sampledAnalysis?.pathLength ?? length),
                perimeterMeters: nil,
                areaSquareMeters: nil,
                swathWidthMeters: path.swathWidthMeters.map(Double.init),
                minElevationMeters: (sampledAnalysis?.minElevation).map(Double.init) ?? vertexStats.map { Double($0.minElevation) },
                maxElevationMeters: (sampledAnalysis?.maxElevation).map(Double.init) ?? vertexStats.map { Double($0.maxElevation) },
                reliefMeters: (sampledAnalysis?.relief).map(Double.init) ?? vertexStats.map { Double($0.relief) },
                totalRiseMeters: (sampledAnalysis?.totalRise).map(Double.init) ?? vertexStats.map { Double($0.totalRise) },
                totalFallMeters: (sampledAnalysis?.totalFall).map(Double.init) ?? vertexStats.map { Double($0.totalFall) },
                averageSlopePercent: (sampledAnalysis?.averageSlopePercent).map(Double.init) ?? vertexStats.map { Double($0.averageSlopePercent) },
                maxSlopePercent: (sampledAnalysis?.maxSlopePercent).map(Double.init) ?? vertexStats.map { Double($0.maxSlopePercent) },
                notes: {
                    switch path.tool {
                    case .path: return nil
                    case .crossSection: return "Sampled cross section"
                    case .elevationProfile: return "Sampled elevation profile"
                    }
                }()
            )
        }
    }

    func updateFrame(
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize,
        snapPoint: SIMD3<Float>?,
        cloudSamplePoints: [CloudSamplePoint]
    ) {
        guard isEnabled else { return }
        self.snapPoint = snapPoint
        sampledCloudPoints = cloudSamplePoints
        hasSnapPoint = (snapPoint != nil)
        projectedSnapPoint = snapPoint.flatMap {
            project($0, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize)
        }

        projectedActiveVertices = activeVertices.compactMap {
            project($0, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize)
        }

        projectedActiveSegments = makeSegments(
            vertices: activeVertices,
            closed: false,
            tint: .blue,
            width: 2.0,
            viewMatrix: viewMatrix,
            projectionMatrix: projectionMatrix,
            viewportSize: viewportSize
        )

        projectedActiveSwathSegments = selectedTool == .crossSection
            ? makeSwathSegments(
                vertices: activeVertices,
                swathWidthMeters: Float(swathWidthMeters),
                tint: .blue.opacity(0.88),
                width: 1.0,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            )
            : []

        var committedVertices: [CGPoint] = []
        var committedSegments: [ScreenSegment] = []
        var committedSwathSegments: [ScreenSegment] = []
        var labels: [OverlayLabel] = []

        for path in committedPaths {
            committedVertices.append(contentsOf: path.vertices.compactMap {
                project($0, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize)
            })
            committedSegments.append(contentsOf: makeSegments(
                vertices: path.vertices,
                closed: path.isClosed,
                tint: path.tool == .path ? .white.opacity(0.9) : (path.tool == .crossSection ? .blue.opacity(0.9) : .orange.opacity(0.9)),
                width: 1.6,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            ))

            if path.tool == .crossSection {
                committedSwathSegments.append(contentsOf: makeSwathSegments(
                    vertices: path.vertices,
                    swathWidthMeters: path.swathWidthMeters ?? Float(swathWidthMeters),
                    tint: .blue.opacity(0.84),
                    width: 1.0,
                    viewMatrix: viewMatrix,
                    projectionMatrix: projectionMatrix,
                    viewportSize: viewportSize
                ))
            }

            if path.tool == .path {
                labels.append(contentsOf: segmentLabels(
                    for: path.vertices,
                    closed: path.isClosed,
                    tint: .white,
                    viewMatrix: viewMatrix,
                    projectionMatrix: projectionMatrix,
                    viewportSize: viewportSize
                ))
            }

            if let summary = summaryLabel(
                for: path.vertices,
                closed: path.isClosed,
                tool: path.tool,
                swathWidthMeters: path.swathWidthMeters,
                tint: .white,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            ) {
                labels.append(summary)
            }
        }

        projectedCommittedVertices = committedVertices
        projectedCommittedSegments = committedSegments
        projectedCommittedSwathSegments = committedSwathSegments

        if selectedTool == .path {
            labels.append(contentsOf: segmentLabels(
                for: activeVertices,
                closed: false,
                tint: .blue,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            ))
        }

        if let activeSummary = summaryLabel(
            for: activeVertices,
            closed: false,
            tool: selectedTool,
            swathWidthMeters: selectedTool == .crossSection ? Float(swathWidthMeters) : nil,
            tint: .blue,
            viewMatrix: viewMatrix,
            projectionMatrix: projectionMatrix,
            viewportSize: viewportSize
        ) {
            labels.append(activeSummary)
        }
        overlayLabels = labels
    }

    private func project(
        _ point: SIMD3<Float>,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> CGPoint? {
        let viewProjection = projectionMatrix * viewMatrix
        let clip = viewProjection * SIMD4<Float>(point.x, point.y, point.z, 1)
        guard clip.w > 0 else { return nil }
        let ndc = clip / clip.w
        guard abs(ndc.x) <= 1.2, abs(ndc.y) <= 1.2, ndc.z >= -1.2, ndc.z <= 1.2 else { return nil }
        let x = (CGFloat(ndc.x) * 0.5 + 0.5) * viewportSize.width
        let y = (1 - (CGFloat(ndc.y) * 0.5 + 0.5)) * viewportSize.height
        return CGPoint(x: x, y: y)
    }

    private func segmentLabels(
        for vertices: [SIMD3<Float>],
        closed: Bool,
        tint: Color,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> [OverlayLabel] {
        guard vertices.count >= 2 else { return [] }
        var labels: [OverlayLabel] = []

        for idx in 1..<vertices.count {
            guard let a = project(vertices[idx - 1], viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize),
                  let b = project(vertices[idx], viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize) else {
                continue
            }
            let distance = simd_distance(vertices[idx - 1], vertices[idx])
            let mid = CGPoint(x: (a.x + b.x) / 2, y: (a.y + b.y) / 2)
            labels.append(OverlayLabel(text: "\(String(format: "%.2f", distance))m", position: mid, tint: tint))
        }

        if closed, let firstV = vertices.first, let lastV = vertices.last,
           let firstP = project(firstV, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize),
           let lastP = project(lastV, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize) {
            let distance = simd_distance(lastV, firstV)
            let mid = CGPoint(x: (lastP.x + firstP.x) / 2, y: (lastP.y + firstP.y) / 2)
            labels.append(OverlayLabel(text: "\(String(format: "%.2f", distance))m", position: mid, tint: tint))
        }
        return labels
    }

    private func makeSegments(
        vertices: [SIMD3<Float>],
        closed: Bool,
        tint: Color,
        width: CGFloat,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> [ScreenSegment] {
        guard vertices.count >= 2 else { return [] }
        var segments: [ScreenSegment] = []

        for idx in 1..<vertices.count {
            guard let a = project(vertices[idx - 1], viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize),
                  let b = project(vertices[idx], viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize) else {
                continue
            }
            segments.append(ScreenSegment(a: a, b: b, tint: tint, width: width))
        }

        if closed, let first = vertices.first, let last = vertices.last,
           let a = project(last, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize),
           let b = project(first, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize) {
            segments.append(ScreenSegment(a: a, b: b, tint: tint, width: width))
        }

        return segments
    }

    private func makeSwathSegments(
        vertices: [SIMD3<Float>],
        swathWidthMeters: Float,
        tint: Color,
        width: CGFloat,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> [ScreenSegment] {
        guard vertices.count >= 2 else { return [] }
        let halfSwathMeters = max(0.005, swathWidthMeters * 0.5)

        struct SegmentBasis {
            let normal: CGPoint
            let pixelsPerMeter: CGFloat
        }

        var projected: [CGPoint] = []
        projected.reserveCapacity(vertices.count)
        for vertex in vertices {
            guard let point = project(vertex, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize) else {
                return []
            }
            projected.append(point)
        }

        var basisBySegment: [SegmentBasis] = []
        basisBySegment.reserveCapacity(max(0, vertices.count - 1))
        for idx in 1..<vertices.count {
            let worldA = vertices[idx - 1]
            let worldB = vertices[idx]
            let a = projected[idx - 1]
            let b = projected[idx]

            let dx = b.x - a.x
            let dy = b.y - a.y
            let screenLength = hypot(dx, dy)
            let worldLength = CGFloat(simd_distance(worldA, worldB))
            guard screenLength > 0.0001, worldLength > 0.0001 else { continue }

            let tx = dx / screenLength
            let ty = dy / screenLength
            let nx = -ty
            let ny = tx
            basisBySegment.append(
                SegmentBasis(
                    normal: CGPoint(x: nx, y: ny),
                    pixelsPerMeter: screenLength / worldLength
                )
            )
        }

        guard basisBySegment.count == vertices.count - 1 else { return [] }

        var left: [CGPoint] = []
        var right: [CGPoint] = []
        left.reserveCapacity(vertices.count)
        right.reserveCapacity(vertices.count)

        for idx in 0..<vertices.count {
            let base = projected[idx]
            let normal: CGPoint
            let ppm: CGFloat

            if idx == 0 {
                normal = basisBySegment[0].normal
                ppm = basisBySegment[0].pixelsPerMeter
            } else if idx == vertices.count - 1 {
                normal = basisBySegment[basisBySegment.count - 1].normal
                ppm = basisBySegment[basisBySegment.count - 1].pixelsPerMeter
            } else {
                let prev = basisBySegment[idx - 1]
                let next = basisBySegment[idx]
                let avgNX = prev.normal.x + next.normal.x
                let avgNY = prev.normal.y + next.normal.y
                let nLen = hypot(avgNX, avgNY)
                if nLen > 0.0001 {
                    normal = CGPoint(x: avgNX / nLen, y: avgNY / nLen)
                } else {
                    normal = next.normal
                }
                ppm = (prev.pixelsPerMeter + next.pixelsPerMeter) * 0.5
            }

            let offset = CGFloat(halfSwathMeters) * ppm
            let ox = normal.x * offset
            let oy = normal.y * offset
            left.append(CGPoint(x: base.x + ox, y: base.y + oy))
            right.append(CGPoint(x: base.x - ox, y: base.y - oy))
        }

        var segments: [ScreenSegment] = []
        segments.reserveCapacity((vertices.count - 1) * 2 + 2)
        for idx in 1..<vertices.count {
            segments.append(ScreenSegment(a: left[idx - 1], b: left[idx], tint: tint, width: width))
            segments.append(ScreenSegment(a: right[idx - 1], b: right[idx], tint: tint, width: width))
        }

        if let firstLeft = left.first, let firstRight = right.first,
           let lastLeft = left.last, let lastRight = right.last {
            segments.append(ScreenSegment(a: firstLeft, b: firstRight, tint: tint.opacity(0.55), width: width))
            segments.append(ScreenSegment(a: lastLeft, b: lastRight, tint: tint.opacity(0.55), width: width))
        }

        return segments
    }

    private func summaryLabel(
        for vertices: [SIMD3<Float>],
        closed: Bool,
        tool: MeasureTool,
        swathWidthMeters: Float?,
        tint: Color,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> OverlayLabel? {
        guard vertices.count >= 2 else { return nil }
        let total = totalLength(vertices: vertices, closed: closed)

        let text: String
        switch tool {
        case .path:
            if closed, let area = polygonArea(vertices: vertices) {
                text = "P \(String(format: "%.2f", total))m • A \(String(format: "%.2f", area))m²"
            } else {
                text = "L \(String(format: "%.2f", total))m"
            }
        case .crossSection:
            if let stats = computeElevationStats(vertices: vertices) {
                text = "CS L \(String(format: "%.2f", total))m • W \(String(format: "%.2f", swathWidthMeters ?? 0))m • R \(String(format: "%.2f", stats.relief))m"
            } else {
                text = "CS L \(String(format: "%.2f", total))m • W \(String(format: "%.2f", swathWidthMeters ?? 0))m"
            }
        case .elevationProfile:
            if let stats = computeElevationStats(vertices: vertices) {
                text = "EP L \(String(format: "%.2f", total))m • dZ \(String(format: "%.2f", stats.relief))m"
            } else {
                text = "EP L \(String(format: "%.2f", total))m"
            }
        }

        let centroid = vertices.reduce(SIMD3<Float>(repeating: 0), +) / Float(vertices.count)
        let centroidProjection = project(
            centroid,
            viewMatrix: viewMatrix,
            projectionMatrix: projectionMatrix,
            viewportSize: viewportSize
        )
        let fallbackProjection = vertices.lazy.compactMap { [self] in
            self.project($0, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize)
        }.first
        guard let position = centroidProjection ?? fallbackProjection else { return nil }
        return OverlayLabel(text: text, position: position, tint: tint)
    }

    private func totalLength(vertices: [SIMD3<Float>], closed: Bool) -> Float {
        guard vertices.count >= 2 else { return 0 }
        var total: Float = 0
        for idx in 1..<vertices.count {
            total += simd_distance(vertices[idx - 1], vertices[idx])
        }
        if closed, let first = vertices.first, let last = vertices.last {
            total += simd_distance(last, first)
        }
        return total
    }

    private func polygonArea(vertices: [SIMD3<Float>]) -> Float? {
        guard vertices.count >= 3 else { return nil }

        // Newell normal for robust polygon plane normal estimation.
        var normal = SIMD3<Float>(repeating: 0)
        for i in 0..<vertices.count {
            let current = vertices[i]
            let next = vertices[(i + 1) % vertices.count]
            normal.x += (current.y - next.y) * (current.z + next.z)
            normal.y += (current.z - next.z) * (current.x + next.x)
            normal.z += (current.x - next.x) * (current.y + next.y)
        }
        let nLen = simd_length(normal)
        guard nLen > 1e-6 else { return nil }
        let n = normal / nLen

        // Build in-plane basis.
        let reference = abs(n.z) < 0.9 ? SIMD3<Float>(0, 0, 1) : SIMD3<Float>(0, 1, 0)
        let u = simd_normalize(simd_cross(reference, n))
        let v = simd_cross(n, u)

        let origin = vertices[0]
        var points2D: [SIMD2<Float>] = []
        points2D.reserveCapacity(vertices.count)
        for p in vertices {
            let rel = p - origin
            points2D.append(SIMD2<Float>(simd_dot(rel, u), simd_dot(rel, v)))
        }

        var area2: Float = 0
        for i in 0..<points2D.count {
            let a = points2D[i]
            let b = points2D[(i + 1) % points2D.count]
            area2 += a.x * b.y - b.x * a.y
        }
        return abs(area2) * 0.5
    }
}

private final class ExportCancellationFlag {
    private let lock = NSLock()
    private var cancelled = false

    var isCancelled: Bool {
        lock.lock()
        defer { lock.unlock() }
        return cancelled
    }

    func requestCancel() {
        lock.lock()
        cancelled = true
        lock.unlock()
    }
}

struct ShareSheet: UIViewControllerRepresentable {
    let activityItems: [Any]

    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: activityItems, applicationActivities: nil)
    }

    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}

struct PointCloudViewerMetal: UIViewRepresentable {
    let device: MTLDevice
    let engine: PointCloudEngine
    let isRenderingPaused: Bool
    let recenterSignal: Int
    let isRollUnlocked: Bool
    let uprightResetSignal: Int
    let measureController: MeasureController
    let selectedColorMode: CloudColorMode
    let onRenderColorStatusChanged: (PointCloudRenderColorStatus) -> Void
    
    func makeUIView(context: Context) -> MTKView {
        let mtkView = MTKView()
        mtkView.device = device
        mtkView.delegate = context.coordinator
        mtkView.backgroundColor = .black
        mtkView.clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.preferredFramesPerSecond = 30
        
        // Enable multitouch for gestures
        mtkView.isMultipleTouchEnabled = true
        
        return mtkView
    }
    
    func updateUIView(_ uiView: MTKView, context: Context) {
        uiView.isPaused = isRenderingPaused
        context.coordinator.updateRecenterSignal(recenterSignal)
        context.coordinator.updateRollUnlocked(isRollUnlocked)
        context.coordinator.updateUprightResetSignal(uprightResetSignal)
        context.coordinator.updateSelectedColorMode(selectedColorMode)
        // Setup gestures if not already done
        if uiView.gestureRecognizers?.isEmpty ?? true {
            context.coordinator.setupGestures(for: uiView)
        }
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator(
            device: device,
            engine: engine,
            measureController: measureController,
            onRenderColorStatusChanged: onRenderColorStatusChanged
        )
    }
    
    class Coordinator: NSObject, MTKViewDelegate, UIGestureRecognizerDelegate {
        private struct VoxelMirror {
            var positionAndConfidence: SIMD4<Float>
            var colorAndSampleCount: SIMD4<Float>
        }

        private struct RenderColorUniforms {
            var mode: UInt32
            var hasUsableRGB: UInt32
            var minElevation: Float
            var maxElevation: Float
            var minIntensity: Float
            var maxIntensity: Float
        }

        private struct RenderColorStats {
            var minElevation: Float = -1
            var maxElevation: Float = 1
            var minIntensity: Float = 0
            var maxIntensity: Float = 1
            var hasUsableRGB: Bool = true
        }

        private struct CameraPose {
            let target: SIMD3<Float>
            let distance: Float
            let pitch: Float
            let yaw: Float
            let roll: Float
        }

        private struct CameraAnimation {
            let from: CameraPose
            let to: CameraPose
            let startTime: CFTimeInterval
            let duration: CFTimeInterval
        }

        private let device: MTLDevice
        private let engine: PointCloudEngine
        private let measureController: MeasureController
        private let onRenderColorStatusChanged: (PointCloudRenderColorStatus) -> Void
        private let commandQueue: MTLCommandQueue
        private var pipelineState: MTLRenderPipelineState!
        private var depthStencilState: MTLDepthStencilState!
        private let cameraFOVDegrees: Float = 60.0
        private var selectedColorMode: CloudColorMode = .auto
        private var renderColorStats = RenderColorStats()
        private var lastColorStatsPointCount: Int = -1
        private var lastColorStatsRefreshTime: TimeInterval = 0
        private var lastRenderColorStatus: PointCloudRenderColorStatus?
        
        // Orbit camera state
        private var cameraDistance: Float = 2.0
        private var cameraPitch: Float = 0.3  // Initial tilt
        private var cameraYaw: Float = 0.0
        private var cameraRoll: Float = 0.0
        private var orbitTarget: SIMD3<Float> = .zero
        private var shouldAutoFit = true
        private var pendingManualRecenter = false
        private var lastRecenterSignal = 0
        private var rollUnlocked = false
        private var pendingUprightReset = false
        private var lastUprightResetSignal = 0
        private var cameraAnimation: CameraAnimation?

        private var sampledPoints: [MeasureController.CloudSamplePoint] = []
        private var lastSamplePointCount: Int = -1
        private var lastSampleRefreshTime: TimeInterval = 0
        
        // Gesture tracking
        // (handled by UIGestureRecognizers)
        
        init(
            device: MTLDevice,
            engine: PointCloudEngine,
            measureController: MeasureController,
            onRenderColorStatusChanged: @escaping (PointCloudRenderColorStatus) -> Void
        ) {
            self.device = device
            self.engine = engine
            self.measureController = measureController
            self.onRenderColorStatusChanged = onRenderColorStatusChanged
            self.commandQueue = device.makeCommandQueue()!
            
            super.init()
            setupPipeline()
        }
        
        private func setupPipeline() {
            let library = device.makeDefaultLibrary()!
            let vertexFn = library.makeFunction(name: "pointVertex")!
            let fragmentFn = library.makeFunction(name: "pointFragmentRound")!
            
            let desc = MTLRenderPipelineDescriptor()
            desc.vertexFunction = vertexFn
            desc.fragmentFunction = fragmentFn
            desc.colorAttachments[0].pixelFormat = .bgra8Unorm
            desc.colorAttachments[0].isBlendingEnabled = true
            desc.colorAttachments[0].rgbBlendOperation = .add
            desc.colorAttachments[0].alphaBlendOperation = .add
            desc.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
            desc.colorAttachments[0].sourceAlphaBlendFactor = .sourceAlpha
            desc.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
            desc.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
            desc.depthAttachmentPixelFormat = .depth32Float
            
            pipelineState = try! device.makeRenderPipelineState(descriptor: desc)
            
            let depthDesc = MTLDepthStencilDescriptor()
            depthDesc.depthCompareFunction = .lessEqual
            depthDesc.isDepthWriteEnabled = true
            depthStencilState = device.makeDepthStencilState(descriptor: depthDesc)!
        }

        func updateRecenterSignal(_ signal: Int) {
            guard signal != lastRecenterSignal else { return }
            lastRecenterSignal = signal
            pendingManualRecenter = true
        }

        func updateRollUnlocked(_ unlocked: Bool) {
            guard rollUnlocked != unlocked else { return }
            rollUnlocked = unlocked
            if !unlocked {
                pendingUprightReset = true
            }
        }

        func updateUprightResetSignal(_ signal: Int) {
            guard signal != lastUprightResetSignal else { return }
            lastUprightResetSignal = signal
            pendingUprightReset = true
        }

        func updateSelectedColorMode(_ mode: CloudColorMode) {
            selectedColorMode = mode
        }
        
        func draw(in view: MTKView) {
            guard let renderPassDescriptor = view.currentRenderPassDescriptor,
                  let commandBuffer = commandQueue.makeCommandBuffer(),
                  let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }

            if engine.activePointCount == 0 {
                shouldAutoFit = true
            }

            if pendingManualRecenter {
                pendingManualRecenter = false
                shouldAutoFit = !recenterCameraToPointCloud(viewportSize: view.bounds.size, animated: true)
            } else if shouldAutoFit && engine.activePointCount > 0 {
                shouldAutoFit = !recenterCameraToPointCloud(viewportSize: view.bounds.size, animated: true)
            }

            if pendingUprightReset {
                pendingUprightReset = false
                if abs(cameraRoll) > 0.0005 {
                    startCameraAnimation(
                        to: CameraPose(
                            target: orbitTarget,
                            distance: cameraDistance,
                            pitch: cameraPitch,
                            yaw: cameraYaw,
                            roll: 0
                        ),
                        duration: 0.18
                    )
                } else {
                    cameraRoll = 0
                }
            }

            updateCameraAnimationIfNeeded()
            refreshRenderColorStatsIfNeeded()
            let effectiveColorMode = resolvedColorMode()
            let status = PointCloudRenderColorStatus(
                hasUsableRGB: renderColorStats.hasUsableRGB,
                effectiveMode: effectiveColorMode
            )
            if status != lastRenderColorStatus {
                lastRenderColorStatus = status
                DispatchQueue.main.async { [status, onRenderColorStatusChanged] in
                    onRenderColorStatusChanged(status)
                }
            }
            
            // Build orbit camera matrices
            let viewMatrix = makeOrbitViewMatrix()
            let aspect = max(Float(view.bounds.width / max(1, view.bounds.height)), 0.2)
            let nearPlane = max(0.01, min(10.0, cameraDistance * 0.0008))
            let farPlane = max(300.0, min(2_000_000.0, cameraDistance * 200.0))
            let projectionMatrix = makeProjectionMatrix(aspect: aspect, fov: cameraFOVDegrees, near: nearPlane, far: farPlane)

            if measureController.isEnabled {
                refreshPointSamplesIfNeeded()
                let snapPoint = nearestPointNearCenter(
                    viewMatrix: viewMatrix,
                    projectionMatrix: projectionMatrix,
                    viewportSize: view.bounds.size
                )
                let applyUpdate: () -> Void = { [weak self] in
                    guard let self = self else { return }
                    self.measureController.updateFrame(
                        viewMatrix: viewMatrix,
                        projectionMatrix: projectionMatrix,
                        viewportSize: view.bounds.size,
                        snapPoint: snapPoint,
                        cloudSamplePoints: self.sampledPoints
                    )
                }
                if Thread.isMainThread {
                    applyUpdate()
                } else {
                    DispatchQueue.main.async(execute: applyUpdate)
                }
            }
            
            var vpMatrix = projectionMatrix * viewMatrix
            var colorUniforms = RenderColorUniforms(
                mode: UInt32(effectiveColorMode.rawValue),
                hasUsableRGB: renderColorStats.hasUsableRGB ? 1 : 0,
                minElevation: renderColorStats.minElevation,
                maxElevation: renderColorStats.maxElevation,
                minIntensity: renderColorStats.minIntensity,
                maxIntensity: renderColorStats.maxIntensity
            )
            
            // Render point cloud
            encoder.setRenderPipelineState(pipelineState)
            encoder.setDepthStencilState(depthStencilState)
            encoder.setVertexBuffer(engine.voxelBuffer, offset: 0, index: 0)
            encoder.setVertexBytes(&vpMatrix, length: MemoryLayout<simd_float4x4>.stride, index: 1)
            encoder.setVertexBytes(&colorUniforms, length: MemoryLayout<RenderColorUniforms>.stride, index: 2)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: engine.maxVoxels)
            
            encoder.endEncoding()
            commandBuffer.present(view.currentDrawable!)
            commandBuffer.commit()
        }
        
        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}
        
        // MARK: - Camera Math
        
        private func makeOrbitViewMatrix() -> simd_float4x4 {
            let eye = cameraPosition()
            let basis = cameraBasis()
            return lookAt(eye: eye, center: orbitTarget, up: basis.up)
        }

        private func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
            let f = simd_normalize(center - eye)
            let s = simd_normalize(simd_cross(f, up))
            let u = simd_cross(s, f)

            return simd_float4x4(
                SIMD4<Float>(s.x, u.x, -f.x, 0),
                SIMD4<Float>(s.y, u.y, -f.y, 0),
                SIMD4<Float>(s.z, u.z, -f.z, 0),
                SIMD4<Float>(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)
            )
        }
        
        private func makeProjectionMatrix(aspect: Float, fov: Float, near: Float, far: Float) -> simd_float4x4 {
            let fovRadians = fov * .pi / 180.0
            let yScale = 1.0 / tan(fovRadians * 0.5)
            let xScale = yScale / aspect
            let zRange = far - near
            let zScale = -(far + near) / zRange
            let wzScale = -2.0 * far * near / zRange
            
            return simd_float4x4(
                SIMD4<Float>(xScale, 0, 0, 0),
                SIMD4<Float>(0, yScale, 0, 0),
                SIMD4<Float>(0, 0, zScale, -1),
                SIMD4<Float>(0, 0, wzScale, 0)
            )
        }

        @discardableResult
        private func recenterCameraToPointCloud(viewportSize: CGSize, animated: Bool) -> Bool {
            guard let bounds = computePointCloudBounds() else { return false }

            let minPoint = bounds.min
            let maxPoint = bounds.max
            let center = (minPoint + maxPoint) * 0.5
            let halfExtents = (maxPoint - minPoint) * 0.5
            var radius = simd_length(halfExtents)
            if !radius.isFinite || radius < 0.02 {
                radius = 0.02
            }

            let safeAspect = max(Float(viewportSize.width / max(1, viewportSize.height)), 0.2)
            let verticalHalfFOV = (cameraFOVDegrees * .pi / 180.0) * 0.5
            let horizontalHalfFOV = atan(tan(verticalHalfFOV) * safeAspect)
            let limitingHalfFOV = max(min(verticalHalfFOV, horizontalHalfFOV), 0.08)

            let framingPadding: Float = 1.18
            let targetDistance = max(
                0.12,
                min(50_000.0, (radius / tan(limitingHalfFOV)) * framingPadding)
            )

            let targetPose = CameraPose(
                target: center,
                distance: targetDistance,
                pitch: 0.3,
                yaw: 0.0,
                roll: 0.0
            )
            if animated {
                startCameraAnimation(to: targetPose, duration: 0.28)
            } else {
                applyCameraPose(targetPose)
            }
            return true
        }

        private func currentCameraPose() -> CameraPose {
            CameraPose(
                target: orbitTarget,
                distance: cameraDistance,
                pitch: cameraPitch,
                yaw: cameraYaw,
                roll: cameraRoll
            )
        }

        private func applyCameraPose(_ pose: CameraPose) {
            orbitTarget = pose.target
            cameraDistance = pose.distance
            cameraPitch = pose.pitch
            cameraYaw = pose.yaw
            cameraRoll = pose.roll
        }

        private func startCameraAnimation(to target: CameraPose, duration: CFTimeInterval) {
            updateCameraAnimationIfNeeded()
            let startPose = currentCameraPose()
            cameraAnimation = CameraAnimation(
                from: startPose,
                to: target,
                startTime: CACurrentMediaTime(),
                duration: max(0.05, duration)
            )
        }

        private func cancelCameraAnimation() {
            cameraAnimation = nil
        }

        private func updateCameraAnimationIfNeeded() {
            guard let animation = cameraAnimation else { return }
            let elapsed = CACurrentMediaTime() - animation.startTime
            let t = min(max(elapsed / animation.duration, 0), 1)
            let eased = easeOutCubic(Float(t))

            let interpolated = CameraPose(
                target: simd_mix(animation.from.target, animation.to.target, SIMD3<Float>(repeating: eased)),
                distance: animation.from.distance + (animation.to.distance - animation.from.distance) * eased,
                pitch: animation.from.pitch + (animation.to.pitch - animation.from.pitch) * eased,
                yaw: animation.from.yaw + (animation.to.yaw - animation.from.yaw) * eased,
                roll: animation.from.roll + (animation.to.roll - animation.from.roll) * eased
            )
            applyCameraPose(interpolated)

            if t >= 1 {
                applyCameraPose(animation.to)
                cameraAnimation = nil
            }
        }

        private func easeOutCubic(_ t: Float) -> Float {
            let inv = 1 - t
            return 1 - (inv * inv * inv)
        }

        private func computePointCloudBounds() -> (min: SIMD3<Float>, max: SIMD3<Float>)? {
            let voxels = engine.voxelBuffer.contents().assumingMemoryBound(to: VoxelMirror.self)
            var hasAnyPoint = false
            var minPoint = SIMD3<Float>(repeating: 0)
            var maxPoint = SIMD3<Float>(repeating: 0)

            for idx in 0..<engine.maxVoxels {
                let voxel = voxels[idx]
                if voxel.colorAndSampleCount.w <= 0 { continue }

                let point = SIMD3<Float>(
                    voxel.positionAndConfidence.x,
                    voxel.positionAndConfidence.y,
                    voxel.positionAndConfidence.z
                )
                if !hasAnyPoint {
                    minPoint = point
                    maxPoint = point
                    hasAnyPoint = true
                } else {
                    minPoint = simd_min(minPoint, point)
                    maxPoint = simd_max(maxPoint, point)
                }
            }

            return hasAnyPoint ? (minPoint, maxPoint) : nil
        }

        private func refreshPointSamplesIfNeeded() {
            let now = CACurrentMediaTime()
            let activeCount = max(1, engine.activePointCount)
            let refreshInterval: TimeInterval = activeCount < 120_000 ? 0.22 : 0.12
            let shouldRefreshByCount = abs(engine.activePointCount - lastSamplePointCount) > 400
            let shouldRefreshByTime = (now - lastSampleRefreshTime) > refreshInterval
            guard shouldRefreshByCount || shouldRefreshByTime || sampledPoints.isEmpty else { return }

            func normalizeColor(_ value: Float) -> Float {
                let scaled = value <= 1.0 ? (value * 255.0) : value
                return max(0, min(255, scaled)) / 255.0
            }

            let targetSampleCount = 48_000
            let step = max(1, activeCount / targetSampleCount)
            let voxels = engine.voxelBuffer.contents().assumingMemoryBound(to: VoxelMirror.self)

            var newSampled: [MeasureController.CloudSamplePoint] = []
            newSampled.reserveCapacity(min(targetSampleCount, activeCount))

            for idx in stride(from: 0, to: engine.maxVoxels, by: step) {
                let voxel = voxels[idx]
                if voxel.colorAndSampleCount.w <= 0 { continue }
                newSampled.append(
                    MeasureController.CloudSamplePoint(
                        position: SIMD3<Float>(
                            voxel.positionAndConfidence.x,
                            voxel.positionAndConfidence.y,
                            voxel.positionAndConfidence.z
                        ),
                        color: SIMD3<Float>(
                            normalizeColor(voxel.colorAndSampleCount.x),
                            normalizeColor(voxel.colorAndSampleCount.y),
                            normalizeColor(voxel.colorAndSampleCount.z)
                        )
                    )
                )
                if newSampled.count >= targetSampleCount { break }
            }

            sampledPoints = newSampled
            lastSamplePointCount = engine.activePointCount
            lastSampleRefreshTime = now
        }

        private func resolvedColorMode() -> CloudColorMode {
            if selectedColorMode == .auto {
                return renderColorStats.hasUsableRGB ? .rgb : .elevation
            }
            return selectedColorMode
        }

        private func refreshRenderColorStatsIfNeeded() {
            let now = CACurrentMediaTime()
            let activeCount = engine.activePointCount
            let shouldRefreshByCount = abs(activeCount - lastColorStatsPointCount) > 3_000
            let shouldRefreshByTime = (now - lastColorStatsRefreshTime) > 0.30
            guard shouldRefreshByCount || shouldRefreshByTime || lastColorStatsPointCount < 0 else { return }

            guard activeCount > 0 else {
                renderColorStats = RenderColorStats()
                lastColorStatsPointCount = 0
                lastColorStatsRefreshTime = now
                return
            }

            let voxels = engine.voxelBuffer.contents().assumingMemoryBound(to: VoxelMirror.self)
            let sampleStride = max(1, engine.maxVoxels / 120_000)

            var hasAny = false
            var minElevation = Float.greatestFiniteMagnitude
            var maxElevation = -Float.greatestFiniteMagnitude
            var minIntensity = Float.greatestFiniteMagnitude
            var maxIntensity = -Float.greatestFiniteMagnitude

            var sampleCount = 0
            var brightWhiteCount = 0
            var luminanceMean: Float = 0
            var luminanceM2: Float = 0

            for idx in stride(from: 0, to: engine.maxVoxels, by: sampleStride) {
                let voxel = voxels[idx]
                if voxel.colorAndSampleCount.w <= 0 { continue }

                hasAny = true
                sampleCount += 1

                let y = voxel.positionAndConfidence.y
                minElevation = min(minElevation, y)
                maxElevation = max(maxElevation, y)

                let intensity = max(0, min(1, voxel.positionAndConfidence.w))
                minIntensity = min(minIntensity, intensity)
                maxIntensity = max(maxIntensity, intensity)

                let r = max(0, min(1, voxel.colorAndSampleCount.x))
                let g = max(0, min(1, voxel.colorAndSampleCount.y))
                let b = max(0, min(1, voxel.colorAndSampleCount.z))
                if abs(r - 1) + abs(g - 1) + abs(b - 1) < 0.03 {
                    brightWhiteCount += 1
                }

                let luminance = (0.2126 * r) + (0.7152 * g) + (0.0722 * b)
                let delta = luminance - luminanceMean
                luminanceMean += delta / Float(sampleCount)
                let delta2 = luminance - luminanceMean
                luminanceM2 += delta * delta2
            }

            guard hasAny else {
                renderColorStats = RenderColorStats()
                lastColorStatsPointCount = activeCount
                lastColorStatsRefreshTime = now
                return
            }

            let elevationSpan = max(0.001, maxElevation - minElevation)
            let intensitySpan = max(0.001, maxIntensity - minIntensity)
            let whiteFraction = sampleCount > 0 ? Float(brightWhiteCount) / Float(sampleCount) : 1.0
            let luminanceVariance = sampleCount > 1 ? luminanceM2 / Float(sampleCount - 1) : 0
            let hasUsableRGB = (whiteFraction < 0.985) && (luminanceVariance > 0.00005)

            renderColorStats = RenderColorStats(
                minElevation: minElevation,
                maxElevation: minElevation + elevationSpan,
                minIntensity: minIntensity,
                maxIntensity: minIntensity + intensitySpan,
                hasUsableRGB: hasUsableRGB
            )
            lastColorStatsPointCount = activeCount
            lastColorStatsRefreshTime = now
        }

        private func nearestPointNearCenter(
            viewMatrix: simd_float4x4,
            projectionMatrix: simd_float4x4,
            viewportSize: CGSize
        ) -> SIMD3<Float>? {
            guard !sampledPoints.isEmpty else { return nil }

            let viewProjection = projectionMatrix * viewMatrix
            let center = CGPoint(x: viewportSize.width / 2, y: viewportSize.height / 2)
            let snapThreshold: CGFloat = 84
            let snapThresholdSquared = snapThreshold * snapThreshold

            var bestPoint: SIMD3<Float>?
            var bestDistSquared = CGFloat.greatestFiniteMagnitude
            var bestDepth = Float.greatestFiniteMagnitude

            for sample in sampledPoints {
                let point = sample.position
                let clip = viewProjection * SIMD4<Float>(point.x, point.y, point.z, 1)
                if clip.w <= 0 { continue }
                let ndc = clip / clip.w
                if ndc.z < -1 || ndc.z > 1 { continue }

                let px = (CGFloat(ndc.x) * 0.5 + 0.5) * viewportSize.width
                let py = (1 - (CGFloat(ndc.y) * 0.5 + 0.5)) * viewportSize.height
                let dx = px - center.x
                let dy = py - center.y
                let distSquared = dx * dx + dy * dy
                if distSquared > snapThresholdSquared { continue }
                if ndc.z > 0.999 { continue }

                if distSquared + 0.01 < bestDistSquared ||
                    (abs(distSquared - bestDistSquared) <= 0.01 && ndc.z < bestDepth) {
                    bestDistSquared = distSquared
                    bestDepth = ndc.z
                    bestPoint = point
                }
            }

            guard bestDistSquared <= snapThresholdSquared else { return nil }
            return bestPoint
        }
        
        // MARK: - Gesture Handling Setup
        
        func setupGestures(for view: MTKView) {
            let panGesture = UIPanGestureRecognizer(target: self, action: #selector(handlePan(_:)))
            panGesture.minimumNumberOfTouches = 1
            panGesture.maximumNumberOfTouches = 2
            panGesture.delegate = self
            view.addGestureRecognizer(panGesture)
            
            let pinchGesture = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
            pinchGesture.delegate = self
            view.addGestureRecognizer(pinchGesture)

            let rotationGesture = UIRotationGestureRecognizer(target: self, action: #selector(handleRotation(_:)))
            rotationGesture.delegate = self
            view.addGestureRecognizer(rotationGesture)
        }
        
        @objc private func handlePan(_ gesture: UIPanGestureRecognizer) {
            cancelCameraAnimation()
            let translation = gesture.translation(in: gesture.view)
            
            if gesture.numberOfTouches == 1 {
                // Single finger - pan in current screen-space basis (respects roll/orientation).
                let panSpeed = 0.0014 * cameraDistance
                let basis = cameraBasis()
                orbitTarget -= basis.right * Float(translation.x) * panSpeed
                orbitTarget += basis.up * Float(translation.y) * panSpeed
            } else if gesture.numberOfTouches == 2 {
                // Two fingers - orbit around current crosshair snap (if available) at gesture start
                if gesture.state == .began, let snap = measureController.currentSnapWorldPoint() {
                    orbitTarget = snap
                }
                cameraYaw -= Float(translation.x) * 0.01
                cameraPitch -= Float(translation.y) * 0.01
                cameraPitch = max(-Float.pi / 2 + 0.01, min(Float.pi / 2 - 0.01, cameraPitch))
            }
            
            gesture.setTranslation(.zero, in: gesture.view)
        }
        
        @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
            cancelCameraAnimation()
            let normalizedDelta = Float(log2(max(0.01, Double(gesture.scale))))
            cameraDistance *= exp2(-normalizedDelta * 1.25)
            cameraDistance = max(0.12, min(100_000.0, cameraDistance))
            gesture.scale = 1.0
        }

        @objc private func handleRotation(_ gesture: UIRotationGestureRecognizer) {
            guard rollUnlocked else { return }
            cancelCameraAnimation()
            // Incremental twist delta; reset each frame for stable, low-latency roll control.
            let delta = Float(gesture.rotation)
            gesture.rotation = 0
            if abs(delta) < 0.01 { return } // Small deadzone to avoid accidental roll jitter.

            cameraRoll += delta * 0.8
            let maxRoll = Float.pi * 0.95
            cameraRoll = max(-maxRoll, min(maxRoll, cameraRoll))
        }

        func gestureRecognizer(
            _ gestureRecognizer: UIGestureRecognizer,
            shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer
        ) -> Bool {
            true
        }

        private func cameraPosition() -> SIMD3<Float> {
            let dir = SIMD3<Float>(
                cos(cameraPitch) * sin(cameraYaw),
                sin(cameraPitch),
                cos(cameraPitch) * cos(cameraYaw)
            )
            return orbitTarget - dir * cameraDistance
        }

        private func cameraBasis() -> (forward: SIMD3<Float>, right: SIMD3<Float>, up: SIMD3<Float>) {
            let eye = cameraPosition()
            let forward = simd_normalize(orbitTarget - eye)
            var upReference = SIMD3<Float>(0, 1, 0)
            if abs(simd_dot(forward, upReference)) > 0.98 {
                upReference = SIMD3<Float>(0, 0, 1)
            }
            let baseRight = simd_normalize(simd_cross(forward, upReference))
            let baseUp = simd_normalize(simd_cross(baseRight, forward))

            let cosRoll = cos(cameraRoll)
            let sinRoll = sin(cameraRoll)
            let rolledRight = simd_normalize(baseRight * cosRoll + baseUp * sinRoll)
            let rolledUp = simd_normalize(baseUp * cosRoll - baseRight * sinRoll)
            return (forward, rolledRight, rolledUp)
        }
    }
}

// MARK: - Matrix Helpers

extension simd_float4x4 {
    init(rotationAbout axis: SIMD3<Float>, by angle: Float) {
        let c = cos(angle)
        let s = sin(angle)
        let t = 1 - c
        let x = axis.x, y = axis.y, z = axis.z
        
        self.init(
            SIMD4<Float>(t * x * x + c,     t * x * y + z * s, t * x * z - y * s, 0),
            SIMD4<Float>(t * x * y - z * s, t * y * y + c,     t * y * z + x * s, 0),
            SIMD4<Float>(t * x * z + y * s, t * y * z - x * s, t * z * z + c,     0),
            SIMD4<Float>(0, 0, 0, 1)
        )
    }
    
    init(translation: SIMD3<Float>) {
        self.init(
            SIMD4<Float>(1, 0, 0, 0),
            SIMD4<Float>(0, 1, 0, 0),
            SIMD4<Float>(0, 0, 1, 0),
            SIMD4<Float>(translation.x, translation.y, translation.z, 1)
        )
    }
}
