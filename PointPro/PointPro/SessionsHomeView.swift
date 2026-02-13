//
//  SessionsHomeView.swift
//  PointPro
//

import SwiftUI
import UIKit

struct SessionsHomeView: View {
    private enum CaptureFlowState {
        case idle
        case recording
        case paused
    }

    @EnvironmentObject private var store: ScanSessionStore
    @State private var activeSessionID: UUID?
    @State private var newScanSignal = 0
    @State private var clearScanSignal = 0
    @State private var saveSnapshotSignal = 0
    @State private var startScanSignal = 0
    @State private var stopScanSignal = 0
    @State private var openViewerSignal = 0
    @State private var isScanning = false
    @State private var livePointCount = 0
    @State private var flowState: CaptureFlowState = .idle

    @State private var showScansSheet = false
    @State private var isCameraReady = false
    @State private var showNoPointsAlert = false
    @State private var viewerOpenedFromStop = false
    private let tapHaptic = UIImpactFeedbackGenerator(style: .rigid)

    var body: some View {
        ZStack(alignment: .bottom) {
            ContentView(
                showBottomCaptureControls: false,
                showMetricsOnlyWhileScanning: false,
                newScanSignal: newScanSignal,
                clearScanSignal: clearScanSignal,
                saveSnapshotSignal: saveSnapshotSignal,
                startScanSignal: startScanSignal,
                stopScanSignal: stopScanSignal,
                openViewerSignal: openViewerSignal,
                viewerSession: activeSessionID.flatMap { store.session(with: $0) },
                onSnapshotSaved: { data, pointCount in
                    if let id = activeSessionID {
                        store.savePointCloud(data, pointCount: pointCount, for: id)
                    }
                },
                onPrepareViewer: { engine in
                    // For the current capture flow, use live in-memory voxels directly.
                    // Only load from disk when browsing completed scans from idle/list.
                    guard flowState == .idle else { return }
                    if let id = activeSessionID,
                       let loaded = store.loadPointCloud(for: id) {
                        engine.loadSnapshot(loaded.data, pointCount: loaded.pointCount)
                    }
                },
                onViewerDismissed: {
                    // If viewer came from idle/list browsing, clear loaded points so
                    // we return to a clean camera viewport.
                    if flowState == .idle {
                        activeSessionID = nil
                        clearScanSignal += 1
                    } else if viewerOpenedFromStop {
                        // Stop flow completed without "continue": return to idle camera.
                        viewerOpenedFromStop = false
                        flowState = .idle
                        activeSessionID = nil
                        clearScanSignal += 1
                    }
                },
                onViewerContinue: {
                    guard activeSessionID != nil else { return }
                    viewerOpenedFromStop = false
                    if !isScanning {
                        startScanSignal += 1
                    }
                    flowState = .recording
                },
                onPointCountChanged: { count in
                    livePointCount = count
                    if let id = activeSessionID {
                        store.updatePointCount(count, for: id)
                    }
                },
                onScanningChanged: { scanning in
                    isScanning = scanning
                    if let id = activeSessionID {
                        store.setScanning(scanning, for: id)
                    }
                },
                onCameraReady: {
                    isCameraReady = true
                }
            )

            if isCameraReady {
                VStack(spacing: 10) {
                    controlsView
                }
                .padding(.horizontal, 16)
                .padding(.bottom, 30)
            }
        }
        .sheet(isPresented: $showScansSheet) {
            ScanListSheet(
                sessions: store.sessions,
                onSelect: { session in
                    activeSessionID = session.id
                    openViewerSignal += 1
                    showScansSheet = false
                },
                onRename: { id, newName in
                    store.renameSession(id, to: newName)
                },
                onDelete: { id in
                    if activeSessionID == id {
                        activeSessionID = nil
                    }
                    store.deleteSession(id)
                }
            )
            .presentationDetents([.fraction(0.5), .large])
            .presentationDragIndicator(.visible)
            .presentationBackground(.thinMaterial)
            .presentationBackgroundInteraction(.enabled)
            .presentationContentInteraction(.scrolls)
        }
        .onChange(of: flowState) { _, newState in
            if newState != .idle && showScansSheet {
                showScansSheet = false
            }
        }
        .onChange(of: isScanning) { _, scanning in
            if scanning {
                flowState = .recording
            }
        }
        .alert("No Points Captured", isPresented: $showNoPointsAlert) {
            Button("OK", role: .cancel) {}
        } message: {
            Text("No scan data was captured, so nothing was saved.")
        }
    }

    @ViewBuilder
    private var controlsView: some View {
        switch flowState {
        case .idle:
            if !showScansSheet {
                Button(action: {
                    emitTapHaptic()
                    startNewScan()
                }) {
                    HStack {
                        Image(systemName: "record.circle")
                        Text("NEW SCAN")
                    }
                    .font(.system(.subheadline, design: .monospaced).bold())
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                }
                .tint(.blue)
                .buttonStyle(.glassProminent)
            }

            Button(action: {
                emitTapHaptic()
                showScansSheet = true
            }) {
                HStack {
                    Image(systemName: "list.bullet")
                    Text("VIEW SCANS")
                }
                .font(.system(.subheadline, design: .monospaced).bold())
                .frame(maxWidth: .infinity)
                .padding(.vertical, 14)
            }
            .tint(.gray)
            .buttonStyle(.glass)

        case .recording:
            HStack(spacing: 10) {
                Button(action: {
                    emitTapHaptic()
                    pauseScan()
                }) {
                    HStack {
                        Image(systemName: "pause.fill")
                        Text("PAUSE")
                    }
                    .font(.system(.subheadline, design: .monospaced).bold())
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                }
                .tint(.gray)
                .buttonStyle(.glass)

                Button(action: {
                    emitTapHaptic()
                    stopScanAndOpenViewer()
                }) {
                    HStack {
                        Image(systemName: "stop.fill")
                        Text("STOP")
                    }
                    .font(.system(.subheadline, design: .monospaced).bold())
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                }
                .tint(.red)
                .buttonStyle(.glassProminent)
            }

            Button(action: {
                emitTapHaptic()
                cancelScan()
            }) {
                HStack {
                    Image(systemName: "xmark.circle")
                    Text("CANCEL SCAN")
                }
                .font(.system(.subheadline, design: .monospaced).bold())
                .frame(maxWidth: .infinity)
                .padding(.vertical, 12)
            }
            .tint(.gray)
            .buttonStyle(.glass)

        case .paused:
            HStack(spacing: 10) {
                Button(action: {
                    emitTapHaptic()
                    resumeScan()
                }) {
                    HStack {
                        Image(systemName: "play.fill")
                        Text("RESUME")
                    }
                    .font(.system(.subheadline, design: .monospaced).bold())
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                }
                .tint(.blue)
                .buttonStyle(.glassProminent)

                Button(action: {
                    emitTapHaptic()
                    stopScanAndOpenViewer()
                }) {
                    HStack {
                        Image(systemName: "stop.fill")
                        Text("STOP")
                    }
                    .font(.system(.subheadline, design: .monospaced).bold())
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                }
                .tint(.red)
                .buttonStyle(.glass)
            }
            
            Button(action: {
                emitTapHaptic()
                cancelScan()
            }) {
                HStack {
                    Image(systemName: "xmark.circle")
                    Text("CANCEL SCAN")
                }
                .font(.system(.subheadline, design: .monospaced).bold())
                .frame(maxWidth: .infinity)
                .padding(.vertical, 12)
            }
            .tint(.gray)
            .buttonStyle(.glass)
        }
    }

    private func startNewScan() {
        let newSession = store.createSession()
        activeSessionID = newSession.id
        newScanSignal += 1
        flowState = .recording
    }

    private func pauseScan() {
        if isScanning {
            stopScanSignal += 1
        }
        flowState = .paused
    }

    private func resumeScan() {
        if !isScanning {
            startScanSignal += 1
        }
        flowState = .recording
    }

    private func stopScanAndOpenViewer() {
        if isScanning {
            stopScanSignal += 1
        }

        guard livePointCount > 0 else {
            if let id = activeSessionID {
                store.deleteSession(id)
            }
            activeSessionID = nil
            clearScanSignal += 1
            flowState = .idle
            showNoPointsAlert = true
            return
        }

        saveSnapshotSignal += 1
        if let id = activeSessionID {
            store.finalizeSession(id)
        }
        viewerOpenedFromStop = true
        openViewerSignal += 1
        flowState = .paused
    }

    private func cancelScan() {
        if isScanning {
            stopScanSignal += 1
        }
        if let id = activeSessionID {
            store.deleteSession(id)
        }
        viewerOpenedFromStop = false
        flowState = .idle
        activeSessionID = nil
        clearScanSignal += 1
    }

    private func emitTapHaptic() {
        tapHaptic.prepare()
        tapHaptic.impactOccurred(intensity: 1.0)
    }
}

private struct ScanListSheet: View {
    let sessions: [ScanSession]
    let onSelect: (ScanSession) -> Void
    let onRename: (UUID, String) -> Void
    let onDelete: (UUID) -> Void

    @State private var renameTarget: ScanSession?
    @State private var renameText = ""
    private let tapHaptic = UIImpactFeedbackGenerator(style: .rigid)

    var body: some View {
        NavigationStack {
            Group {
                if sessions.isEmpty {
                    ContentUnavailableView(
                        "No Scans Yet",
                        systemImage: "point.3.connected.trianglepath.dotted",
                        description: Text("Start a new scan to see it here.")
                    )
                } else {
                    List {
                        ForEach(sessions) { session in
                            Button(action: {
                                emitTapHaptic()
                                onSelect(session)
                            }) {
                                HStack {
                                    VStack(alignment: .leading, spacing: 4) {
                                        Text(session.name)
                                            .font(.headline)
                                        Text(session.updatedAt.formatted(date: .abbreviated, time: .shortened))
                                            .font(.caption)
                                            .foregroundStyle(.secondary)
                                    }
                                    Spacer()
                            VStack(alignment: .trailing, spacing: 4) {
                                Text("\(formatNumber(session.pointCount)) PTS")
                                    .font(.system(.caption, design: .monospaced).bold())
                                Text(formatStorage(session.dataSizeBytes))
                                    .font(.caption2)
                                    .foregroundStyle(.secondary)
                                Text(session.status.rawValue.uppercased())
                                    .font(.caption2)
                                    .foregroundStyle(.secondary)
                            }
                                }
                                .padding(.vertical, 2)
                                .padding(.horizontal, 16)
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .contentShape(Rectangle())
                            }
                            .buttonStyle(.plain)
                            .listRowInsets(EdgeInsets(top: 8, leading: 0, bottom: 8, trailing: 0))
                            .listRowBackground(Color.clear)
                            .swipeActions(edge: .trailing, allowsFullSwipe: false) {
                                Button("Rename") {
                                    emitTapHaptic()
                                    renameTarget = session
                                    renameText = session.name
                                }
                                .tint(.gray)

                                Button("Delete", role: .destructive) {
                                    emitTapHaptic()
                                    onDelete(session.id)
                                }
                            }
                        }
                    }
                    .listStyle(.plain)
                    .scrollContentBackground(.hidden)
                    .contentMargins(.zero, for: .scrollContent)
                }
            }
            .navigationTitle("Scans")
            .navigationBarTitleDisplayMode(.inline)
        }
        .alert("Rename Scan", isPresented: Binding(
            get: { renameTarget != nil },
            set: { isPresented in
                if !isPresented { renameTarget = nil }
            }
        )) {
            TextField("Scan name", text: $renameText)
            Button("Cancel", role: .cancel) {
                emitTapHaptic()
                renameTarget = nil
            }
            Button("Save") {
                emitTapHaptic()
                if let session = renameTarget {
                    onRename(session.id, renameText)
                }
                renameTarget = nil
            }
        }
    }

    private func emitTapHaptic() {
        tapHaptic.prepare()
        tapHaptic.impactOccurred(intensity: 1.0)
    }
}

private func formatNumber(_ num: Int) -> String {
    let formatter = NumberFormatter()
    formatter.numberStyle = .decimal
    return formatter.string(from: NSNumber(value: num)) ?? "\(num)"
}

private func formatStorage(_ bytes: Int64) -> String {
    ByteCountFormatter.string(fromByteCount: bytes, countStyle: .file)
}
