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

struct PointCloudViewer: View {
    let engine: PointCloudEngine
    let device: MTLDevice
    let session: ScanSession?
    var onExport: ((ScanSession, PointCloudEngine.PLYExportFormat, @escaping (Double, String) -> Void, @escaping () -> Bool) -> URL?)? = nil
    var onContinue: (() -> Void)? = nil
    @Environment(\.dismiss) var dismiss
    @State private var isExporting = false
    @State private var exportURL: URL?
    @State private var showShareSheet = false
    @State private var showExportError = false
    @State private var exportStatusText = "Preparing export..."
    @State private var exportProgress: Double = 0.0
    @State private var exportCancellation = ExportCancellationFlag()
    @State private var showExportCancelledNotice = false
    @State private var showExportFormatPicker = false
    @StateObject private var measureController = MeasureController()
    private let buttonHaptic = UIImpactFeedbackGenerator(style: .rigid)
    
    var body: some View {
        ZStack {
            // Black background
            Color.black.ignoresSafeArea()
            
            // Metal 3D view
            PointCloudViewerMetal(
                device: device,
                engine: engine,
                isRenderingPaused: showExportFormatPicker,
                measureController: measureController
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
                }
                
                Spacer()

                if let session {
                    VStack(alignment: .leading, spacing: 8) {
                        Text(session.name)
                            .font(.system(.title3, design: .monospaced).weight(.bold))
                            .foregroundStyle(.primary)

                        Text("UPDATED \(formatDate(session.updatedAt))")
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)

                        HStack(spacing: 8) {
                            infoGlass("\(formatNumber(session.pointCount)) POINTS", tint: .blue)
                            infoGlass(formatStorage(session.dataSizeBytes))
                            infoGlass(session.status.rawValue.uppercased())
                        }

                        HStack(spacing: 10) {
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

                                        iconActionButton("plus.circle", label: "New Measurement") {
                                            emitTapHaptic()
                                            measureController.commitOpenMeasurementAndStartNew()
                                        }
                                        .disabled(!measureController.canCommitOpenMeasurement)

                                        iconActionButton("seal", label: "Close Shape") {
                                            emitTapHaptic()
                                            measureController.closeCurrentMeasurement()
                                        }
                                        .disabled(!measureController.canCloseCurrentMeasurement)
                                    }
                                    .padding(.vertical, 2)
                                    .id("measure-actions")
                                    .transition(.asymmetric(
                                        insertion: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing)),
                                        removal: .move(edge: .trailing).combined(with: .opacity).combined(with: .scale(scale: 0.94, anchor: .trailing))
                                    ))
                                } else {
                                    HStack(spacing: 8) {
                                        if let onContinue {
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
                                            .disabled(isExporting || showExportFormatPicker)
                                            .accessibilityLabel("Continue Scan")
                                        }

                                        if onExport != nil {
                                            Button(action: {
                                                guard !isExporting else { return }
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
                                            .disabled(isExporting)
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
            if isExporting {
                HStack(spacing: 10) {
                    VStack(alignment: .leading, spacing: 6) {
                        Text(exportStatusText)
                            .font(.system(.caption, design: .monospaced))
                            .foregroundStyle(.white)
                            .lineLimit(1)
                        HStack(spacing: 10) {
                            ProgressView(value: exportProgress, total: 1.0)
                                .tint(.white)
                                .frame(maxWidth: .infinity)
                            Text("\(Int(exportProgress * 100))%")
                                .font(.system(.caption, design: .monospaced).bold())
                                .foregroundStyle(.white)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)

                    Button("CANCEL") {
                        emitTapHaptic()
                        exportCancellation.requestCancel()
                        exportStatusText = "Cancelling export..."
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
        .sheet(isPresented: $showShareSheet) {
            if let exportURL {
                ShareSheet(activityItems: [exportURL])
            }
        }
        .confirmationDialog(
            "Choose Export Format",
            isPresented: $showExportFormatPicker,
            titleVisibility: .visible
        ) {
            if let session {
                Button("PLY (Fast Binary)") {
                    emitTapHaptic()
                    startExport(session: session, format: .binaryLittleEndian)
                }
                Button("PLY (Readable ASCII)") {
                    emitTapHaptic()
                    startExport(session: session, format: .ascii)
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

    private func startExport(session: ScanSession, format: PointCloudEngine.PLYExportFormat) {
        guard let onExport else { return }
        guard !isExporting else { return }
        exportCancellation = ExportCancellationFlag()
        exportProgress = 0
        exportStatusText = format == .binaryLittleEndian ? "Generating binary PLY..." : "Generating ASCII PLY..."
        isExporting = true
        showExportCancelledNotice = false
        DispatchQueue.global(qos: .utility).async {
            let url = onExport(
                session,
                format,
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
                if let url {
                    exportURL = url
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
    struct MeasurementPath {
        var vertices: [SIMD3<Float>]
        var isClosed: Bool
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

    @Published var isEnabled = false
    @Published var hasSnapPoint = false
    @Published var projectedActiveVertices: [CGPoint] = []
    @Published var projectedCommittedVertices: [CGPoint] = []
    @Published var projectedActiveSegments: [ScreenSegment] = []
    @Published var projectedCommittedSegments: [ScreenSegment] = []
    @Published var projectedSnapPoint: CGPoint?
    @Published var overlayLabels: [OverlayLabel] = []

    var hasActiveVertices: Bool { !activeVertices.isEmpty }
    var canCloseCurrentMeasurement: Bool { activeVertices.count >= 3 }
    var canCommitOpenMeasurement: Bool { activeVertices.count >= 2 }

    private var snapPoint: SIMD3<Float>?
    private var activeVertices: [SIMD3<Float>] = []
    private var committedPaths: [MeasurementPath] = []

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
            projectedSnapPoint = nil
            overlayLabels = []
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

    func closeCurrentMeasurement() {
        guard activeVertices.count >= 3 else { return }
        committedPaths.append(MeasurementPath(vertices: activeVertices, isClosed: true))
        activeVertices = []
    }

    func commitOpenMeasurementAndStartNew() {
        guard activeVertices.count >= 2 else { return }
        committedPaths.append(MeasurementPath(vertices: activeVertices, isClosed: false))
        activeVertices = []
    }

    func updateFrame(
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize,
        snapPoint: SIMD3<Float>?
    ) {
        guard isEnabled else { return }
        self.snapPoint = snapPoint
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

        var committedVertices: [CGPoint] = []
        var committedSegments: [ScreenSegment] = []

        var labels: [OverlayLabel] = []
        for path in committedPaths {
            committedVertices.append(contentsOf: path.vertices.compactMap {
                project($0, viewMatrix: viewMatrix, projectionMatrix: projectionMatrix, viewportSize: viewportSize)
            })
            committedSegments.append(contentsOf: makeSegments(
                vertices: path.vertices,
                closed: path.isClosed,
                tint: .white.opacity(0.9),
                width: 1.6,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            ))
            labels.append(contentsOf: segmentLabels(
                for: path.vertices,
                closed: path.isClosed,
                tint: .white,
                viewMatrix: viewMatrix,
                projectionMatrix: projectionMatrix,
                viewportSize: viewportSize
            ))
            if let summary = summaryLabel(
                for: path.vertices,
                closed: path.isClosed,
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

        labels.append(contentsOf: segmentLabels(
            for: activeVertices,
            closed: false,
            tint: .blue,
            viewMatrix: viewMatrix,
            projectionMatrix: projectionMatrix,
            viewportSize: viewportSize
        ))
        if let activeSummary = summaryLabel(
            for: activeVertices,
            closed: false,
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

    private func summaryLabel(
        for vertices: [SIMD3<Float>],
        closed: Bool,
        tint: Color,
        viewMatrix: simd_float4x4,
        projectionMatrix: simd_float4x4,
        viewportSize: CGSize
    ) -> OverlayLabel? {
        guard vertices.count >= 2 else { return nil }
        let total = totalLength(vertices: vertices, closed: closed)

        var text = "L \(String(format: "%.2f", total))m"
        if closed, let area = polygonArea(vertices: vertices) {
            text = "P \(String(format: "%.2f", total))m • A \(String(format: "%.2f", area))m²"
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
    let measureController: MeasureController
    
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
        // Setup gestures if not already done
        if uiView.gestureRecognizers?.isEmpty ?? true {
            context.coordinator.setupGestures(for: uiView)
        }
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator(device: device, engine: engine, measureController: measureController)
    }
    
    class Coordinator: NSObject, MTKViewDelegate, UIGestureRecognizerDelegate {
        private let device: MTLDevice
        private let engine: PointCloudEngine
        private let measureController: MeasureController
        private let commandQueue: MTLCommandQueue
        private var pipelineState: MTLRenderPipelineState!
        private var depthStencilState: MTLDepthStencilState!
        
        // Orbit camera state
        private var cameraDistance: Float = 2.0
        private var cameraPitch: Float = 0.3  // Initial tilt
        private var cameraYaw: Float = 0.0
        private var cameraRoll: Float = 0.0
        private var orbitTarget: SIMD3<Float> = .zero

        private var sampledPoints: [SIMD3<Float>] = []
        private var lastSamplePointCount: Int = -1
        private var lastSampleRefreshTime: TimeInterval = 0
        
        // Gesture tracking
        // (handled by UIGestureRecognizers)
        
        init(device: MTLDevice, engine: PointCloudEngine, measureController: MeasureController) {
            self.device = device
            self.engine = engine
            self.measureController = measureController
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
        
        func draw(in view: MTKView) {
            guard let renderPassDescriptor = view.currentRenderPassDescriptor,
                  let commandBuffer = commandQueue.makeCommandBuffer(),
                  let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
            
            // Build orbit camera matrices
            let viewMatrix = makeOrbitViewMatrix()
            let aspect = Float(view.bounds.width / view.bounds.height)
            let projectionMatrix = makeProjectionMatrix(aspect: aspect, fov: 60.0, near: 0.01, far: 100.0)

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
                        snapPoint: snapPoint
                    )
                }
                if Thread.isMainThread {
                    applyUpdate()
                } else {
                    DispatchQueue.main.async(execute: applyUpdate)
                }
            }
            
            var vpMatrix = projectionMatrix * viewMatrix
            
            // Render point cloud
            encoder.setRenderPipelineState(pipelineState)
            encoder.setDepthStencilState(depthStencilState)
            encoder.setVertexBuffer(engine.voxelBuffer, offset: 0, index: 0)
            encoder.setVertexBytes(&vpMatrix, length: MemoryLayout<simd_float4x4>.stride, index: 1)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: engine.maxVoxels)
            
            encoder.endEncoding()
            commandBuffer.present(view.currentDrawable!)
            commandBuffer.commit()
        }
        
        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}
        
        // MARK: - Camera Math
        
        private func makeOrbitViewMatrix() -> simd_float4x4 {
            let dir = SIMD3<Float>(
                cos(cameraPitch) * sin(cameraYaw),
                sin(cameraPitch),
                cos(cameraPitch) * cos(cameraYaw)
            )
            let eye = orbitTarget - dir * cameraDistance
            let forward = simd_normalize(orbitTarget - eye)
            var upReference = SIMD3<Float>(0, 1, 0)
            if abs(simd_dot(forward, upReference)) > 0.98 {
                upReference = SIMD3<Float>(0, 0, 1)
            }
            let right = simd_normalize(simd_cross(forward, upReference))
            let up = simd_normalize(simd_cross(right, forward))

            let cosRoll = cos(cameraRoll)
            let sinRoll = sin(cameraRoll)
            let rolledUp = simd_normalize(up * cosRoll - right * sinRoll)

            return lookAt(eye: eye, center: orbitTarget, up: rolledUp)
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

        private func refreshPointSamplesIfNeeded() {
            let now = CACurrentMediaTime()
            let shouldRefreshByCount = abs(engine.activePointCount - lastSamplePointCount) > 400
            let shouldRefreshByTime = (now - lastSampleRefreshTime) > 0.12
            guard shouldRefreshByCount || shouldRefreshByTime || sampledPoints.isEmpty else { return }

            struct VoxelMirror {
                var positionAndConfidence: SIMD4<Float>
                var colorAndSampleCount: SIMD4<Float>
            }

            let targetSampleCount = 24_000
            let divisor = max(1, min(targetSampleCount, max(1, engine.activePointCount)))
            let step = max(1, engine.maxVoxels / divisor)
            let voxels = engine.voxelBuffer.contents().assumingMemoryBound(to: VoxelMirror.self)

            var newSampled: [SIMD3<Float>] = []
            newSampled.reserveCapacity(min(targetSampleCount, max(1, engine.activePointCount)))

            for idx in stride(from: 0, to: engine.maxVoxels, by: step) {
                let voxel = voxels[idx]
                if voxel.colorAndSampleCount.w <= 0 { continue }
                newSampled.append(SIMD3<Float>(
                    voxel.positionAndConfidence.x,
                    voxel.positionAndConfidence.y,
                    voxel.positionAndConfidence.z
                ))
            }

            sampledPoints = newSampled
            lastSamplePointCount = engine.activePointCount
            lastSampleRefreshTime = now
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

            for point in sampledPoints {
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
            let translation = gesture.translation(in: gesture.view)
            
            if gesture.numberOfTouches == 1 {
                // Single finger - pan target in camera plane
                let panSpeed = 0.0014 * cameraDistance
                let forward = simd_normalize(orbitTarget - cameraPosition())
                let right = simd_normalize(simd_cross(forward, SIMD3<Float>(0, 1, 0)))
                let up = simd_normalize(simd_cross(right, forward))
                orbitTarget -= right * Float(translation.x) * panSpeed
                orbitTarget += up * Float(translation.y) * panSpeed
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
            cameraDistance /= Float(gesture.scale)
            cameraDistance = max(0.12, min(30.0, cameraDistance))  // Wider zoom range for detailed picking
            gesture.scale = 1.0
        }

        @objc private func handleRotation(_ gesture: UIRotationGestureRecognizer) {
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
