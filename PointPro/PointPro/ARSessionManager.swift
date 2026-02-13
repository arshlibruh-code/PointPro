//
//  ARSessionManager.swift
//  PointPro
//

import ARKit
import Combine
import CoreVideo
import CoreMotion
import UIKit

class ARSessionManager: NSObject, ObservableObject {
    
    // MARK: - Published State
    @Published var isLiDARAvailable: Bool = false
    @Published var sessionState: String = "Initializing..."
    @Published var depthWidth: Int = 0
    @Published var depthHeight: Int = 0
    @Published var pointsPerFrame: Int = 0
    @Published var averageDepth: Float = 0.0
    @Published var frameCount: Int = 0
    @Published var depthFPS: Int = 0
    @Published var storageUsedBytes: Int64 = 0
    @Published var isScanning: Bool = false
    @Published var isTooClose: Bool = false
    @Published var isPhoneFlatOnTable: Bool = false
    @Published var highConfidencePercent: Float = 0.0
    @Published var mediumConfidencePercent: Float = 0.0
    @Published var lowConfidencePercent: Float = 0.0
    @Published var hasReceivedFirstFrame: Bool = false
    
    // MARK: - Engine
    let pointCloudEngine = PointCloudEngine()
    let session = ARSession()
    private let sessionDelegateQueue = DispatchQueue(label: "com.pointpro.arsession.delegate", qos: .userInitiated)
    private var sessionConfiguration: ARWorldTrackingConfiguration?
    
    private var lastDepthFrameTimestamp: TimeInterval?
    private let tooCloseEnterDepth: Float = 0.14
    private let tooCloseExitDepth: Float = 0.18
    private let stateConfirmFrames: Int = 2
    private var tooCloseFrames: Int = 0
    private var safeFrames: Int = 0
    private var tooCloseState: Bool = false
    private var tooCloseHapticTimer: Timer?
    private let haptic = UIImpactFeedbackGenerator(style: .rigid)
    private var storageRefreshTimer: Timer?
    private var hasStartedStorageMonitoring = false
    private let motionManager = CMMotionManager()
    private var flatCandidateStart: Date?
    private let flatConfirmDuration: TimeInterval = 0.3
    private let flatEnterThreshold: Double = 0.92
    private let flatExitThreshold: Double = 0.85
    
    override init() {
        super.init()
        startMotionUpdates()
    }
    
    func startSession() {
        guard ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) else {
            DispatchQueue.main.async { self.isLiDARAvailable = false }
            return
        }
        
        let config = ARWorldTrackingConfiguration()
        config.frameSemantics = .sceneDepth
        sessionConfiguration = config
        hasReceivedFirstFrame = false
        
        session.delegateQueue = sessionDelegateQueue
        session.delegate = self
        session.run(config)
        
        // Pass session to engine for anchor management
        pointCloudEngine.setARSession(session)
        
        DispatchQueue.main.async { 
            self.isLiDARAvailable = true
            self.sessionState = "Ready"
        }
    }

    func pauseSession() {
        session.pause()
        stopTooCloseHaptics()
    }

    func resumeSession() {
        guard let config = sessionConfiguration else {
            startSession()
            return
        }
        session.run(config)
    }
    
    func toggleScanning() {
        isScanning.toggle()
        if isScanning {
            pointCloudEngine.startScanning()
            sessionState = "Scanning..."
        } else {
            pointCloudEngine.stopScanning()
            sessionState = "Paused"
            stopTooCloseHaptics()
        }
    }
    
    func resetScan() {
        pointCloudEngine.clearBuffer()
        isScanning = false
        sessionState = "Cleared"
        isTooClose = false
        tooCloseState = false
        tooCloseFrames = 0
        safeFrames = 0
        stopTooCloseHaptics()
    }

    deinit {
        motionManager.stopDeviceMotionUpdates()
        storageRefreshTimer?.invalidate()
    }

}

extension ARSessionManager: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        if !hasReceivedFirstFrame {
            DispatchQueue.main.async {
                self.hasReceivedFirstFrame = true
            }
        }
        if !hasStartedStorageMonitoring {
            hasStartedStorageMonitoring = true
            DispatchQueue.main.async {
                self.startStorageMonitoring()
            }
        }
        if let depthData = frame.sceneDepth {
            // HUD Stats calculation
            let width = CVPixelBufferGetWidth(depthData.depthMap)
            let height = CVPixelBufferGetHeight(depthData.depthMap)
            
            // FPS calculation based on ARFrame timestamp delta (stable + clamped)
            if let lastTimestamp = lastDepthFrameTimestamp {
                let delta = frame.timestamp - lastTimestamp
                if delta > 0 {
                    let rawFPS = 1.0 / delta
                    let clampedFPS = max(0, min(120, Int(rawFPS.rounded())))
                    DispatchQueue.main.async {
                        self.depthFPS = clampedFPS
                    }
                }
            }
            lastDepthFrameTimestamp = frame.timestamp
            DispatchQueue.main.async {
                self.depthWidth = width
                self.depthHeight = height
                self.pointsPerFrame = width * height
            }
            
            // Depth point meter (center)
            CVPixelBufferLockBaseAddress(depthData.depthMap, .readOnly)
            let depthBase = CVPixelBufferGetBaseAddress(depthData.depthMap)?.assumingMemoryBound(to: Float32.self)
            if let depthBase = depthBase {
                let depthStats = centerDepthStats3x3(depth: depthBase, width: width, height: height)

                if depthStats.median > 0 {
                    // Use both minimum and median depths for robust near-surface detection.
                    // This prevents random far spikes from escaping TOO CLOSE when almost touching a surface.
                    if depthStats.minimum < tooCloseEnterDepth {
                        tooCloseFrames += 1
                        safeFrames = 0
                    } else if depthStats.minimum > tooCloseExitDepth && depthStats.median > tooCloseExitDepth {
                        safeFrames += 1
                        tooCloseFrames = 0
                    }

                    if !tooCloseState && tooCloseFrames >= stateConfirmFrames {
                        tooCloseState = true
                        DispatchQueue.main.async {
                            self.isTooClose = true
                            self.startTooCloseHaptics()
                        }
                    } else if tooCloseState && safeFrames >= stateConfirmFrames {
                        tooCloseState = false
                        DispatchQueue.main.async {
                            self.isTooClose = false
                            self.stopTooCloseHaptics()
                        }
                    }

                    if !tooCloseState {
                        DispatchQueue.main.async {
                            self.averageDepth = depthStats.median
                        }
                    }
                }
            }
            CVPixelBufferUnlockBaseAddress(depthData.depthMap, .readOnly)
        }
        
        // Pass to Metal Engine
        autoreleasepool {
            pointCloudEngine.processFrame(frame)
        }
        
        DispatchQueue.main.async { self.frameCount += 1 }
    }

    private func centerDepthStats3x3(depth: UnsafePointer<Float32>, width: Int, height: Int) -> (median: Float, minimum: Float) {
        let midX = width / 2
        let midY = height / 2
        var samples: [Float] = []
        samples.reserveCapacity(9)

        for dy in -1...1 {
            for dx in -1...1 {
                let x = min(max(midX + dx, 0), width - 1)
                let y = min(max(midY + dy, 0), height - 1)
                let value = depth[y * width + x]
                if value.isFinite && value > 0 {
                    samples.append(value)
                }
            }
        }

        guard !samples.isEmpty else {
            return (0, 0)
        }

        samples.sort()
        return (samples[samples.count / 2], samples[0])
    }

    private func startTooCloseHaptics() {
        guard tooCloseHapticTimer == nil else { return }
        haptic.prepare()
        haptic.impactOccurred(intensity: 1.0)
        tooCloseHapticTimer = Timer.scheduledTimer(withTimeInterval: 0.35, repeats: true) { [weak self] _ in
            guard let self = self else { return }
            self.haptic.prepare()
            self.haptic.impactOccurred(intensity: 1.0)
        }
    }

    private func stopTooCloseHaptics() {
        tooCloseHapticTimer?.invalidate()
        tooCloseHapticTimer = nil
    }

    private func startStorageMonitoring() {
        refreshStorageUsage()
        storageRefreshTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
            self?.refreshStorageUsage()
        }
    }

    private func refreshStorageUsage() {
        DispatchQueue.global(qos: .utility).async { [weak self] in
            guard let self = self else { return }
            let bytes = self.calculateDocumentsDirectorySize()
            DispatchQueue.main.async {
                self.storageUsedBytes = bytes
            }
        }
    }

    private func calculateDocumentsDirectorySize() -> Int64 {
        guard let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first,
              let enumerator = FileManager.default.enumerator(
                at: documentsURL,
                includingPropertiesForKeys: [.isRegularFileKey, .totalFileAllocatedSizeKey, .fileAllocatedSizeKey],
                options: [.skipsHiddenFiles],
                errorHandler: nil
              ) else {
            return 0
        }

        var total: Int64 = 0
        for case let fileURL as URL in enumerator {
            guard let values = try? fileURL.resourceValues(forKeys: [.isRegularFileKey, .totalFileAllocatedSizeKey, .fileAllocatedSizeKey]),
                  values.isRegularFile == true else {
                continue
            }
            total += Int64(values.totalFileAllocatedSize ?? values.fileAllocatedSize ?? 0)
        }
        return total
    }

    private func startMotionUpdates() {
        guard motionManager.isDeviceMotionAvailable else { return }
        motionManager.deviceMotionUpdateInterval = 1.0 / 30.0
        motionManager.startDeviceMotionUpdates(to: .main) { [weak self] motion, _ in
            guard let self, let motion else { return }

            let absGravityZ = abs(motion.gravity.z)
            let isFlatNow = absGravityZ >= self.flatEnterThreshold

            if isFlatNow {
                if self.flatCandidateStart == nil {
                    self.flatCandidateStart = Date()
                }
                if let start = self.flatCandidateStart,
                   Date().timeIntervalSince(start) >= self.flatConfirmDuration {
                    self.isPhoneFlatOnTable = true
                }
            } else {
                self.flatCandidateStart = nil
                if self.isPhoneFlatOnTable && absGravityZ <= self.flatExitThreshold {
                    self.isPhoneFlatOnTable = false
                }
            }
        }
    }
}
