//
//  ScanSessionStore.swift
//  PointPro
//

import Foundation
import Combine
import QuartzCore

@MainActor
final class ScanSessionStore: ObservableObject {
    @Published private(set) var sessions: [ScanSession] = []

    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()
    private var pendingSaveWorkItem: DispatchWorkItem?
    private var lastPointCountUpdateBySession: [UUID: (count: Int, time: CFTimeInterval)] = [:]
    private let pointCountUIUpdateInterval: CFTimeInterval = 0.25
    private let pointCountUIUpdateDelta: Int = 200

    init() {
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        sessions = []
        loadAsync()
    }

    func createSession() -> ScanSession {
        let now = Date()
        let index = sessions.count + 1
        let session = ScanSession(
            id: UUID(),
            name: "Scan \(index)",
            createdAt: now,
            updatedAt: now,
            status: .ready,
            pointCount: 0
        )
        sessions.insert(session, at: 0)
        save()
        return session
    }

    func session(with id: UUID) -> ScanSession? {
        sessions.first(where: { $0.id == id })
    }

    func updatePointCount(_ pointCount: Int, for id: UUID) {
        guard let idx = sessions.firstIndex(where: { $0.id == id }) else { return }
        let clampedCount = max(0, pointCount)
        guard sessions[idx].pointCount != clampedCount else { return }

        let now = CACurrentMediaTime()
        if let previous = lastPointCountUpdateBySession[id] {
            let delta = abs(clampedCount - previous.count)
            let elapsed = now - previous.time
            if delta < pointCountUIUpdateDelta && elapsed < pointCountUIUpdateInterval {
                return
            }
        }

        sessions[idx].pointCount = clampedCount
        lastPointCountUpdateBySession[id] = (clampedCount, now)
        scheduleDebouncedSave()
    }

    func setScanning(_ isScanning: Bool, for id: UUID) {
        guard let idx = sessions.firstIndex(where: { $0.id == id }) else { return }
        if isScanning {
            sessions[idx].status = .scanning
        } else if sessions[idx].status == .scanning {
            sessions[idx].status = .ready
        }
        sessions[idx].updatedAt = Date()
        sortSessions()
        save()
    }

    func finalizeSession(_ id: UUID) {
        guard let idx = sessions.firstIndex(where: { $0.id == id }) else { return }
        sessions[idx].status = .completed
        sessions[idx].updatedAt = Date()
        sortSessions()
        save()
    }

    func renameSession(_ id: UUID, to newName: String) {
        let trimmed = newName.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty,
              let idx = sessions.firstIndex(where: { $0.id == id }) else { return }
        sessions[idx].name = trimmed
        sessions[idx].updatedAt = Date()
        sortSessions()
        save()
    }

    func deleteSession(_ id: UUID) {
        sessions.removeAll(where: { $0.id == id })
        lastPointCountUpdateBySession.removeValue(forKey: id)
        try? FileManager.default.removeItem(at: artifactURL(for: id))
        try? FileManager.default.removeItem(at: metadataURL(for: id))
        save()
    }

    func savePointCloud(_ data: Data, pointCount: Int, for id: UUID, metadata: ScanCaptureMetadata?) {
        guard let idx = sessions.firstIndex(where: { $0.id == id }) else { return }
        ensureScansDirectory()
        let url = artifactURL(for: id)
        do {
            try data.write(to: url, options: .atomic)
            if let metadata {
                let encoded = try encoder.encode(metadata)
                try encoded.write(to: metadataURL(for: id), options: .atomic)
            } else {
                try? FileManager.default.removeItem(at: metadataURL(for: id))
            }
            sessions[idx].pointCount = pointCount
            sessions[idx].dataSizeBytes = Int64(data.count)
            sessions[idx].updatedAt = Date()
            lastPointCountUpdateBySession[id] = (max(0, pointCount), CACurrentMediaTime())
            sortSessions()
            save()
        } catch {
            return
        }
    }

    func loadPointCloud(for id: UUID) -> (data: Data, pointCount: Int)? {
        guard let session = sessions.first(where: { $0.id == id }) else { return nil }
        let url = artifactURL(for: id)
        guard let data = try? Data(contentsOf: url) else { return nil }
        return (data, session.pointCount)
    }

    func loadCaptureMetadata(for id: UUID) -> ScanCaptureMetadata? {
        let url = metadataURL(for: id)
        guard let data = try? Data(contentsOf: url) else { return nil }
        return try? decoder.decode(ScanCaptureMetadata.self, from: data)
    }

    private func loadAsync() {
        let url = fileURL
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self else { return }
            guard let data = try? Data(contentsOf: url),
                  let decoded = try? JSONDecoder().decode([ScanSession].self, from: data) else {
                return
            }
            let sorted = decoded.sorted(by: { $0.updatedAt > $1.updatedAt })
            Task { @MainActor in
                self.sessions = sorted
            }
        }
    }

    private func save() {
        pendingSaveWorkItem?.cancel()
        pendingSaveWorkItem = nil
        guard let data = try? encoder.encode(sessions) else { return }
        try? data.write(to: fileURL, options: .atomic)
    }

    private func scheduleDebouncedSave() {
        pendingSaveWorkItem?.cancel()
        let workItem = DispatchWorkItem { [weak self] in
            self?.save()
        }
        pendingSaveWorkItem = workItem
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35, execute: workItem)
    }

    private func sortSessions() {
        sessions.sort(by: { $0.updatedAt > $1.updatedAt })
    }

    private var fileURL: URL {
        let base = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        return base.appendingPathComponent("scan_sessions.json")
    }

    private var scansDirectoryURL: URL {
        let base = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        return base.appendingPathComponent("scans", isDirectory: true)
    }

    private func artifactURL(for id: UUID) -> URL {
        scansDirectoryURL.appendingPathComponent("\(id.uuidString).pcraw")
    }

    private func metadataURL(for id: UUID) -> URL {
        scansDirectoryURL.appendingPathComponent("\(id.uuidString).meta.json")
    }

    private func ensureScansDirectory() {
        try? FileManager.default.createDirectory(at: scansDirectoryURL, withIntermediateDirectories: true)
    }
}
