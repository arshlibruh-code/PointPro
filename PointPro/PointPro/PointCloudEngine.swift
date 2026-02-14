//
//  PointCloudEngine.swift
//  PointPro
//
//  Metal Compute Edition
//  Processes LiDAR frames entirely on GPU.
//

import ARKit
import Metal
import MetalKit
import Combine
import UIKit

private final class URLImportDownloadDelegate: NSObject, URLSessionDownloadDelegate, URLSessionTaskDelegate {
    private let progressHandler: (Double, Int64, Int64) -> Void
    private let completionHandler: (URL?, URLResponse?, Error?) -> Void
    private var downloadedURL: URL?

    init(
        progressHandler: @escaping (Double, Int64, Int64) -> Void,
        completionHandler: @escaping (URL?, URLResponse?, Error?) -> Void
    ) {
        self.progressHandler = progressHandler
        self.completionHandler = completionHandler
    }

    func urlSession(
        _ session: URLSession,
        downloadTask: URLSessionDownloadTask,
        didWriteData bytesWritten: Int64,
        totalBytesWritten: Int64,
        totalBytesExpectedToWrite: Int64
    ) {
        if totalBytesExpectedToWrite > 0 {
            let fraction = min(max(Double(totalBytesWritten) / Double(totalBytesExpectedToWrite), 0), 1)
            progressHandler(fraction, totalBytesWritten, totalBytesExpectedToWrite)
        } else {
            progressHandler(-1, totalBytesWritten, totalBytesExpectedToWrite)
        }
    }

    func urlSession(_ session: URLSession, downloadTask: URLSessionDownloadTask, didFinishDownloadingTo location: URL) {
        downloadedURL = location
    }

    func urlSession(_ session: URLSession, task: URLSessionTask, didCompleteWithError error: Error?) {
        completionHandler(downloadedURL, task.response, error)
    }
}

private final class LAZDecodeProgressBox {
    let onProgress: (Double) -> Void
    let isCancelled: () -> Bool

    init(onProgress: @escaping (Double) -> Void, isCancelled: @escaping () -> Bool) {
        self.onProgress = onProgress
        self.isCancelled = isCancelled
    }
}

private let lazDecodeProgressThunk: @convention(c) (Float, UnsafeMutableRawPointer?) -> Void = { fraction, context in
    guard let context else { return }
    let box = Unmanaged<LAZDecodeProgressBox>.fromOpaque(context).takeUnretainedValue()
    let clamped = min(max(Double(fraction), 0), 1)
    box.onProgress(clamped)
}

private let lazDecodeCancelThunk: @convention(c) (UnsafeMutableRawPointer?) -> Bool = { context in
    guard let context else { return false }
    let box = Unmanaged<LAZDecodeProgressBox>.fromOpaque(context).takeUnretainedValue()
    return box.isCancelled()
}

class PointCloudEngine: ObservableObject {
    enum PLYExportFormat {
        case binaryLittleEndian
        case ascii
    }

    enum ExportFormat {
        case laz
        case plyBinaryLittleEndian
        case plyAscii
        case pdfReport
    }

    struct ExportArtifact {
        let primaryURL: URL
        let sidecarURL: URL?
        var additionalURLs: [URL] = []

        var shareItems: [URL] {
            var items: [URL] = [primaryURL]
            if let sidecarURL {
                items.append(sidecarURL)
            }
            items.append(contentsOf: additionalURLs)
            return items
        }
    }

    private enum COPCStreamAttempt {
        case fallbackToFull
        case success
        case failure(String)
    }

    private struct COPCHeaderInfo {
        var pointFormat: UInt8
        var pointRecordLength: UInt16
        var scaleX: Double
        var scaleY: Double
        var scaleZ: Double
        var offsetX: Double
        var offsetY: Double
        var offsetZ: Double
        var centerX: Double
        var centerY: Double
        var centerZ: Double
        var rootHierarchyOffset: UInt64
        var rootHierarchySize: UInt64
        var pointCount: UInt64
    }

    private struct COPCHierarchyEntry {
        var level: Int32
        var x: Int32
        var y: Int32
        var z: Int32
        var offset: UInt64
        var byteSize: Int32
        var pointCount: Int32
    }
    
    // MARK: - Constants
    let maxVoxels: Int = 2_000_000 // 2 million points for higher detail
    let voxelSize: Float = 0.005   // 5mm voxels (4x detail improvement)
    
    // MARK: - Metal Resources
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private var computePipeline: MTLComputePipelineState!
    
    // Buffers
    private(set) var voxelBuffer: MTLBuffer!
    private var uniformBuffer: MTLBuffer!
    private var counterBuffer: MTLBuffer!
    
    // MARK: - State
    var isScanning: Bool = false
    private var worldOriginAnchor: ARAnchor?
    private var lastCameraPosition: SIMD3<Float>?
    private var isMoving: Bool = false
    
    // Stats for HUD
    @Published var activePointCount: Int = 0
    
    // Texture Caches
    private var cvTextureCache: CVMetalTextureCache?
    private let copcNodeSnapshotCache = NSCache<NSString, NSData>()
    private let inFlightSemaphore = DispatchSemaphore(value: 1)
    private let generationLock = NSLock()
    private var generationToken: UInt64 = 0
    private var lastProcessedTimestamp: TimeInterval = 0
    private let minProcessInterval: TimeInterval = 1.0 / 15.0
    
    // ARSession reference (for anchor management)
    private weak var arSession: ARSession?
    
    init() {
        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!
        
        setupMetal()
        setupBuffers()

        copcNodeSnapshotCache.totalCostLimit = 96 * 1_024 * 1_024
        copcNodeSnapshotCache.countLimit = 512
        
        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, device, nil, &cvTextureCache)
    }
    
    private func setupMetal() {
        let library = device.makeDefaultLibrary()!
        let kernel = library.makeFunction(name: "accumulatePoints")!
        computePipeline = try! device.makeComputePipelineState(function: kernel)
    }
    
    private func setupBuffers() {
        // Voxel Grid: 1M voxels * ~32 bytes = 32MB
        let voxelSizeInBytes = MemoryLayout<Voxel>.stride
        voxelBuffer = device.makeBuffer(length: maxVoxels * voxelSizeInBytes, options: .storageModeShared)
        
        // Atomic Counter
        counterBuffer = device.makeBuffer(length: MemoryLayout<UInt32>.stride, options: .storageModeShared)
        
        // Clear both buffers
        clearBuffer()
        
        // Uniforms
        uniformBuffer = device.makeBuffer(length: MemoryLayout<Uniforms>.stride, options: .storageModeShared)
    }
    
    
    func setARSession(_ session: ARSession) {
        self.arSession = session
    }
    
    func startScanning() {
        guard let session = arSession else { return }
        isScanning = true
        lastProcessedTimestamp = 0
        
        // Create world origin anchor at current camera position
        if worldOriginAnchor == nil {
            worldOriginAnchor = ARAnchor(name: "WorldOrigin", transform: session.currentFrame?.camera.transform ?? matrix_identity_float4x4)
            session.add(anchor: worldOriginAnchor!)
        }
    }
    
    func stopScanning() {
        isScanning = false
    }
    
    func clearBuffer() {
        // Invalidate any in-flight frame work so stale callbacks cannot republish points.
        invalidateGeneration()
        drainInFlightWork()

        let ptr = voxelBuffer.contents()
        memset(ptr, 0, voxelBuffer.length)
        resetCounter()
        currentPointCount = 0
        lastProcessedTimestamp = 0
        lastCameraPosition = nil
        DispatchQueue.main.async {
            self.activePointCount = 0
        }
        
        // Remove and recreate anchor for next scan
        if let anchor = worldOriginAnchor, let session = arSession {
            session.remove(anchor: anchor)
            worldOriginAnchor = nil
        }
    }
    
    func getWorldOriginTransform() -> simd_float4x4 {
        return worldOriginAnchor?.transform ?? matrix_identity_float4x4
    }
    
    private func resetCounter() {
        let ptr = counterBuffer.contents().assumingMemoryBound(to: UInt32.self)
        ptr.pointee = 0
    }

    private func resetCounter(to count: Int) {
        let ptr = counterBuffer.contents().assumingMemoryBound(to: UInt32.self)
        ptr.pointee = UInt32(max(0, count))
    }

    private func invalidateGeneration() {
        generationLock.lock()
        generationToken &+= 1
        generationLock.unlock()
    }

    private func currentGeneration() -> UInt64 {
        generationLock.lock()
        let token = generationToken
        generationLock.unlock()
        return token
    }

    private func isGenerationCurrent(_ token: UInt64) -> Bool {
        generationLock.lock()
        let isCurrent = generationToken == token
        generationLock.unlock()
        return isCurrent
    }

    private func drainInFlightWork() {
        inFlightSemaphore.wait()
        inFlightSemaphore.signal()
    }
    
    func processFrame(_ frame: ARFrame) {
        guard isScanning else { return }
        let generation = currentGeneration()
        guard let depthData = frame.sceneDepth,
              let confidenceMap = depthData.confidenceMap else { return }
        guard frame.timestamp - lastProcessedTimestamp >= minProcessInterval else { return }
        lastProcessedTimestamp = frame.timestamp

        // Backpressure: if GPU is still processing prior frames, drop this frame.
        // Prevents ARSession warning: "delegate is retaining N ARFrames".
        guard inFlightSemaphore.wait(timeout: .now()) == .success else { return }
        var shouldSignalOnExit = true
        defer {
            if shouldSignalOnExit {
                inFlightSemaphore.signal()
            }
        }
        guard isGenerationCurrent(generation) else { return }
        
        // 1. Detect Camera Motion (Apple approach - accumulate when MOVING)
        let currentPosition = SIMD3<Float>(frame.camera.transform.columns.3.x, 
                                           frame.camera.transform.columns.3.y,
                                           frame.camera.transform.columns.3.z)
        
        if let lastPos = lastCameraPosition {
            let movement = distance(currentPosition, lastPos)
            // Accumulate only when camera has moved enough (new angle = new data)
            // This is the OPPOSITE of what we were doing before
            let hasMovedEnough = movement > 0.02  // 2cm threshold
            
            // Skip frame if camera is TOO still (redundant data from same viewpoint)
            if !hasMovedEnough && currentPointCount > 0 {
                return
            }
        }
        lastCameraPosition = currentPosition
        
        let depthMap = depthData.depthMap
        let capturedImage = frame.capturedImage
        
        // 2. Convert CVPixelBuffers to Metal Textures (Zero-copy)
        guard let depthTexture = createTexture(from: depthMap, pixelFormat: .r32Float, planeIndex: 0),
              let yTexture = createTexture(from: capturedImage, pixelFormat: .r8Unorm, planeIndex: 0),
              let cbcrTexture = createTexture(from: capturedImage, pixelFormat: .rg8Unorm, planeIndex: 1),
              let confidenceTexture = createTexture(from: confidenceMap, pixelFormat: .r8Uint, planeIndex: 0) else { return }
        
        // 3. Update Uniforms
        updateUniforms(frame: frame, depthMap: depthMap)
        
        // 4. Dispatch Compute
        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeComputeCommandEncoder() else { return }
        
        encoder.setComputePipelineState(computePipeline)
        encoder.setTexture(depthTexture, index: 0)
        encoder.setTexture(yTexture, index: 1)
        encoder.setTexture(cbcrTexture, index: 2)
        encoder.setTexture(confidenceTexture, index: 3)
        encoder.setBuffer(voxelBuffer, offset: 0, index: 0)
        encoder.setBuffer(uniformBuffer, offset: 0, index: 1)
        encoder.setBuffer(counterBuffer, offset: 0, index: 2)
        
        let w = computePipeline.threadExecutionWidth
        let h = computePipeline.maxTotalThreadsPerThreadgroup / w
        let threadsPerGroup = MTLSize(width: w, height: h, depth: 1)
        let gridInThreads = MTLSize(width: CVPixelBufferGetWidth(depthMap), height: CVPixelBufferGetHeight(depthMap), depth: 1)
        
        encoder.dispatchThreads(gridInThreads, threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()
        
        let semaphore = inFlightSemaphore
        commandBuffer.addCompletedHandler { [weak self] _ in
            semaphore.signal()
            guard let self = self else { return }
            guard self.isGenerationCurrent(generation) else { return }
            let count = self.counterBuffer.contents().assumingMemoryBound(to: UInt32.self).pointee
            DispatchQueue.main.async {
                guard self.isGenerationCurrent(generation) else { return }
                self.activePointCount = Int(count)
                self.currentPointCount = Int(count)
            }
        }
        
        commandBuffer.commit()
        shouldSignalOnExit = false
    }

    func makeSnapshot() -> (data: Data, pointCount: Int) {
        // Compact snapshot format:
        // [magic: UInt32][version: UInt32][count: UInt32][repeated: index(UInt32)+Voxel(32 bytes)]
        let magic: UInt32 = 0x50435331 // 'PCS1'
        let version: UInt32 = 1
        let voxelStride = MemoryLayout<Voxel>.stride
        let voxelCount = maxVoxels

        let voxels = voxelBuffer.contents().assumingMemoryBound(to: Voxel.self)
        var entries: [(UInt32, Voxel)] = []
        entries.reserveCapacity(max(0, activePointCount))

        for i in 0..<voxelCount {
            let v = voxels[i]
            if v.colorAndSampleCount.w > 0 {
                entries.append((UInt32(i), v))
            }
        }

        var out = Data()
        var m = magic, ver = version, c = UInt32(entries.count)
        out.append(Data(bytes: &m, count: MemoryLayout<UInt32>.size))
        out.append(Data(bytes: &ver, count: MemoryLayout<UInt32>.size))
        out.append(Data(bytes: &c, count: MemoryLayout<UInt32>.size))

        for (idx, voxel) in entries {
            var i = idx
            var v = voxel
            out.append(Data(bytes: &i, count: MemoryLayout<UInt32>.size))
            out.append(Data(bytes: &v, count: voxelStride))
        }

        return (out, entries.count)
    }

    func loadSnapshot(_ data: Data, pointCount: Int) {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride

        // Backward compatibility: raw full-buffer snapshots from older builds.
        if data.count == voxelBuffer.length {
            let byteCount = min(data.count, voxelBuffer.length)
            data.withUnsafeBytes { bytes in
                guard let src = bytes.baseAddress else { return }
                memcpy(voxelBuffer.contents(), src, byteCount)
            }
            resetCounter(to: pointCount)
            currentPointCount = pointCount
            DispatchQueue.main.async {
                self.activePointCount = pointCount
            }
            return
        }

        guard data.count >= headerSize else { return }

        // Clear first, then apply compact entries.
        memset(voxelBuffer.contents(), 0, voxelBuffer.length)

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        guard let magic = readUInt32(at: 0) else { return }
        guard magic == 0x50435331 else { return }
        guard let version = readUInt32(at: MemoryLayout<UInt32>.size) else { return }
        guard version == 1 else { return }

        guard let storedCount = readUInt32(at: MemoryLayout<UInt32>.size * 2) else { return }
        let maxRecordsBySize = max(0, (data.count - headerSize) / recordSize)
        let count = min(Int(storedCount), maxRecordsBySize)

        var offset = headerSize
        let voxels = voxelBuffer.contents().assumingMemoryBound(to: Voxel.self)
        var loaded = 0

        for _ in 0..<count {
            let needed = recordSize
            guard offset + needed <= data.count else { break }

            guard let rawIndex = readUInt32(at: offset) else { break }
            let index = Int(rawIndex)
            offset += MemoryLayout<UInt32>.size

            guard index >= 0 && index < maxVoxels else {
                offset += voxelStride
                continue
            }

            guard let voxel = readVoxel(at: offset) else { break }
            offset += voxelStride
            voxels[index] = voxel
            loaded += 1
        }

        resetCounter(to: loaded)
        currentPointCount = loaded
        DispatchQueue.main.async {
            self.activePointCount = loaded
        }
    }

    @discardableResult
    private func mergeCompactSnapshot(_ data: Data) -> Int {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride

        guard data.count >= headerSize else { return 0 }

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        guard let magic = readUInt32(at: 0), magic == 0x50435331 else { return 0 }
        guard let version = readUInt32(at: MemoryLayout<UInt32>.size), version == 1 else { return 0 }
        guard let storedCount = readUInt32(at: MemoryLayout<UInt32>.size * 2) else { return 0 }

        let maxRecordsBySize = max(0, (data.count - headerSize) / recordSize)
        let count = min(Int(storedCount), maxRecordsBySize)
        let voxels = voxelBuffer.contents().assumingMemoryBound(to: Voxel.self)

        var offset = headerSize
        var newlyInserted = 0
        for _ in 0..<count {
            guard offset + recordSize <= data.count else { break }
            guard let rawIndex = readUInt32(at: offset) else { break }
            let index = Int(rawIndex)
            offset += MemoryLayout<UInt32>.size

            guard index >= 0 && index < maxVoxels else {
                offset += voxelStride
                continue
            }
            guard let incoming = readVoxel(at: offset) else { break }
            offset += voxelStride
            if incoming.colorAndSampleCount.w <= 0 { continue }

            if voxels[index].colorAndSampleCount.w <= 0 {
                newlyInserted += 1
            }
            voxels[index] = incoming
        }

        if newlyInserted > 0 {
            currentPointCount = min(maxVoxels, currentPointCount + newlyInserted)
            resetCounter(to: currentPointCount)
            activePointCount = currentPointCount
        }

        return newlyInserted
    }

    private func parseCOPCHeaderInfo(from data: Data) -> COPCHeaderInfo? {
        func readUInt8(_ offset: Int) -> UInt8? {
            guard offset + 1 <= data.count else { return nil }
            return data[offset]
        }

        func readUInt16LE(_ offset: Int) -> UInt16? {
            guard offset + 2 <= data.count else { return nil }
            var value: UInt16 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + 2))
            }
            return UInt16(littleEndian: value)
        }

        func readUInt32LE(_ offset: Int) -> UInt32? {
            guard offset + 4 <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + 4))
            }
            return UInt32(littleEndian: value)
        }

        func readUInt64LE(_ offset: Int) -> UInt64? {
            guard offset + 8 <= data.count else { return nil }
            var value: UInt64 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + 8))
            }
            return UInt64(littleEndian: value)
        }

        func readDoubleLE(_ offset: Int) -> Double? {
            guard let bits = readUInt64LE(offset) else { return nil }
            return Double(bitPattern: bits)
        }

        guard let headerSize = readUInt16LE(94),
              let pointOffset = readUInt32LE(96),
              let vlrCount = readUInt32LE(100),
              let pointFormatRaw = readUInt8(104),
              let pointRecordLength = readUInt16LE(105),
              let scaleX = readDoubleLE(131),
              let scaleY = readDoubleLE(139),
              let scaleZ = readDoubleLE(147),
              let offsetX = readDoubleLE(155),
              let offsetY = readDoubleLE(163),
              let offsetZ = readDoubleLE(171) else {
            return nil
        }

        let needed = Int(pointOffset)
        guard needed > 0, needed <= data.count else { return nil }

        var copcCenterX: Double?
        var copcCenterY: Double?
        var copcCenterZ: Double?
        var rootHierOffset: UInt64?
        var rootHierSize: UInt64?

        var cursor = Int(headerSize)
        for _ in 0..<vlrCount {
            guard cursor + 54 <= needed else { break }

            guard let recordID = readUInt16LE(cursor + 18),
                  let recordLength = readUInt16LE(cursor + 20) else {
                break
            }

            let userIdStart = cursor + 2
            let userIdEnd = userIdStart + 16
            guard userIdEnd <= needed else { break }
            let userIdData = data.subdata(in: userIdStart..<userIdEnd)
            let userId = String(data: userIdData, encoding: .ascii)?
                .trimmingCharacters(in: .controlCharacters.union(.whitespaces))
                .replacingOccurrences(of: "\0", with: "") ?? ""

            let payloadStart = cursor + 54
            let payloadEnd = payloadStart + Int(recordLength)
            guard payloadEnd <= needed else { break }

            if userId == "copc", recordID == 1, recordLength >= 56 {
                let payload = data.subdata(in: payloadStart..<payloadEnd)
                func payloadU64(_ off: Int) -> UInt64 {
                    var value: UInt64 = 0
                    _ = withUnsafeMutableBytes(of: &value) { dst in
                        payload.copyBytes(to: dst, from: off..<(off + 8))
                    }
                    return UInt64(littleEndian: value)
                }
                func payloadDouble(_ off: Int) -> Double {
                    Double(bitPattern: payloadU64(off))
                }
                copcCenterX = payloadDouble(0)
                copcCenterY = payloadDouble(8)
                copcCenterZ = payloadDouble(16)
                rootHierOffset = payloadU64(40)
                rootHierSize = payloadU64(48)
                break
            }

            cursor = payloadEnd
        }

        guard let centerX = copcCenterX,
              let centerY = copcCenterY,
              let centerZ = copcCenterZ,
              let rootOffset = rootHierOffset,
              let rootSize = rootHierSize,
              rootSize > 0 else {
            return nil
        }

        let pointCount = readUInt64LE(247) ?? UInt64(readUInt32LE(107) ?? 0)
        return COPCHeaderInfo(
            pointFormat: pointFormatRaw & 0x3f,
            pointRecordLength: pointRecordLength,
            scaleX: scaleX,
            scaleY: scaleY,
            scaleZ: scaleZ,
            offsetX: offsetX,
            offsetY: offsetY,
            offsetZ: offsetZ,
            centerX: centerX,
            centerY: centerY,
            centerZ: centerZ,
            rootHierarchyOffset: rootOffset,
            rootHierarchySize: rootSize,
            pointCount: pointCount
        )
    }

    private func parseCOPCHierarchyEntries(from data: Data) -> [COPCHierarchyEntry] {
        guard data.count >= 32 else { return [] }

        func readInt32LE(_ offset: Int) -> Int32 {
            var value: Int32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + 4))
            }
            return Int32(littleEndian: value)
        }

        func readUInt64LE(_ offset: Int) -> UInt64 {
            var value: UInt64 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + 8))
            }
            return UInt64(littleEndian: value)
        }

        let entrySize = 32
        let count = data.count / entrySize
        var entries: [COPCHierarchyEntry] = []
        entries.reserveCapacity(count)
        for i in 0..<count {
            let off = i * entrySize
            entries.append(
                COPCHierarchyEntry(
                    level: readInt32LE(off),
                    x: readInt32LE(off + 4),
                    y: readInt32LE(off + 8),
                    z: readInt32LE(off + 12),
                    offset: readUInt64LE(off + 16),
                    byteSize: readInt32LE(off + 24),
                    pointCount: readInt32LE(off + 28)
                )
            )
        }
        return entries
    }

    private func copcNodeCacheKey(for remoteURL: URL, entry: COPCHierarchyEntry) -> NSString {
        "\(remoteURL.absoluteString)|\(entry.level)|\(entry.x)|\(entry.y)|\(entry.z)|\(entry.offset)|\(entry.byteSize)|\(entry.pointCount)" as NSString
    }

    private func recommendedCOPCStreamPointBudget() -> Int {
        let ramBytes = ProcessInfo.processInfo.physicalMemory
        let budget: Int
        if ramBytes >= 7_500_000_000 {
            budget = 1_200_000
        } else if ramBytes >= 5_500_000_000 {
            budget = 900_000
        } else {
            budget = 650_000
        }
        return min(maxVoxels, budget)
    }

    private func selectCOPCEntries(
        from entries: [COPCHierarchyEntry],
        targetPointBudget: Int,
        maxNodes: Int,
        excludingOffsets: Set<UInt64> = []
    ) -> [COPCHierarchyEntry] {
        guard targetPointBudget > 0, maxNodes > 0 else { return [] }

        var selected: [COPCHierarchyEntry] = []
        selected.reserveCapacity(min(maxNodes, entries.count))
        var accumulated = 0
        var seenOffsets = excludingOffsets

        for entry in entries {
            if seenOffsets.contains(entry.offset) {
                continue
            }
            selected.append(entry)
            seenOffsets.insert(entry.offset)
            accumulated += max(0, Int(entry.pointCount))
            if selected.count >= maxNodes || accumulated >= targetPointBudget {
                break
            }
        }
        return selected
    }

    private func tryImportCOPCStreaming(
        _ remoteURL: URL,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> COPCStreamAttempt {
        func finishCancelled() -> COPCStreamAttempt {
            .failure("Import cancelled.")
        }

        if isCancelled?() == true {
            return finishCancelled()
        }

        let config = URLSessionConfiguration.ephemeral
        config.timeoutIntervalForRequest = 25
        config.timeoutIntervalForResource = 120
        let session = URLSession(configuration: config)
        defer { session.invalidateAndCancel() }

        func rangeError(_ message: String, code: Int = 1) -> NSError {
            NSError(
                domain: "PointCloudEngine.COPC",
                code: code,
                userInfo: [NSLocalizedDescriptionKey: message]
            )
        }

        func fetchRange(start: UInt64, length: UInt64) -> Result<Data, Error> {
            guard length > 0 else { return .failure(rangeError("Invalid range length.")) }
            let end = start + length - 1
            var req = URLRequest(url: remoteURL)
            req.timeoutInterval = 30
            req.setValue("bytes=\(start)-\(end)", forHTTPHeaderField: "Range")

            let sem = DispatchSemaphore(value: 0)
            var responseData: Data?
            var response: URLResponse?
            var responseError: Error?
            let task = session.dataTask(with: req) { data, resp, err in
                responseData = data
                response = resp
                responseError = err
                sem.signal()
            }
            task.resume()

            while true {
                if sem.wait(timeout: .now() + 0.1) == .success {
                    break
                }
                if isCancelled?() == true {
                    task.cancel()
                    return .failure(rangeError("Import cancelled."))
                }
            }

            if let responseError {
                let ns = responseError as NSError
                if ns.domain == NSURLErrorDomain && ns.code == NSURLErrorCancelled {
                    return .failure(rangeError("Import cancelled."))
                }
                return .failure(responseError)
            }

            guard let http = response as? HTTPURLResponse else {
                return .failure(rangeError("Invalid server response."))
            }

            guard let payload = responseData else {
                return .failure(rangeError("No response data."))
            }

            if http.statusCode == 206 {
                return .success(payload)
            }
            if http.statusCode == 200, start == 0 {
                return .success(payload)
            }
            return .failure(rangeError("Server does not support range requests for COPC streaming."))
        }

        func fetchRangeWithRetry(
            start: UInt64,
            length: UInt64,
            attempts: Int = 3
        ) -> Result<Data, Error> {
            let maxAttempts = max(1, attempts)
            var attempt = 0
            var lastError: Error = rangeError("Unknown range request error.")

            while attempt < maxAttempts {
                if isCancelled?() == true {
                    return .failure(rangeError("Import cancelled.", code: NSURLErrorCancelled))
                }

                switch fetchRange(start: start, length: length) {
                case .success(let data):
                    return .success(data)
                case .failure(let error):
                    lastError = error
                    let nsError = error as NSError
                    if nsError.domain == NSURLErrorDomain && nsError.code == NSURLErrorCancelled {
                        return .failure(error)
                    }
                }

                attempt += 1
                if attempt < maxAttempts {
                    Thread.sleep(forTimeInterval: 0.20 * Double(attempt))
                }
            }

            return .failure(lastError)
        }

        func decodeCOPCNodeChunk(_ chunkData: Data, entry: COPCHierarchyEntry, headerInfo: COPCHeaderInfo) -> Result<Data, Error> {
            var snapshotPointer: UnsafeMutablePointer<UInt8>?
            var snapshotSize: UInt32 = 0
            var snapshotPointCount: UInt32 = 0
            var errorBuffer = [CChar](repeating: 0, count: 512)

            let decodeOK = chunkData.withUnsafeBytes { raw -> Bool in
                guard let base = raw.baseAddress?.assumingMemoryBound(to: UInt8.self) else { return false }
                return pp_laz_decompress_chunk_to_snapshot(
                    base,
                    UInt32(chunkData.count),
                    headerInfo.pointFormat,
                    headerInfo.pointRecordLength,
                    UInt32(entry.pointCount),
                    headerInfo.scaleX,
                    headerInfo.scaleY,
                    headerInfo.scaleZ,
                    headerInfo.offsetX,
                    headerInfo.offsetY,
                    headerInfo.offsetZ,
                    Float(headerInfo.centerX),
                    Float(headerInfo.centerY),
                    Float(headerInfo.centerZ),
                    self.voxelSize,
                    UInt32(self.maxVoxels),
                    &snapshotPointer,
                    &snapshotSize,
                    &snapshotPointCount,
                    &errorBuffer,
                    UInt32(errorBuffer.count)
                )
            }

            guard decodeOK, let snapshotPointer, snapshotSize > 0, snapshotPointCount > 0 else {
                let message = String(cString: errorBuffer)
                return .failure(rangeError(message.isEmpty ? "Failed to decode streamed COPC node." : message))
            }

            let snapshotData = Data(bytes: snapshotPointer, count: Int(snapshotSize))
            pp_free_buffer(snapshotPointer)
            return .success(snapshotData)
        }

        progress?(0.05, "Checking COPC header...")
        let firstHeaderData: Data
        switch fetchRangeWithRetry(start: 0, length: 4096, attempts: 2) {
        case .success(let data):
            firstHeaderData = data
        case .failure:
            return .fallbackToFull
        }

        guard firstHeaderData.count >= 100 else {
            return .fallbackToFull
        }
        let pointOffset: UInt32 = {
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                firstHeaderData.copyBytes(to: dst, from: 96..<100)
            }
            return UInt32(littleEndian: value)
        }()
        if pointOffset == 0 {
            return .fallbackToFull
        }

        if pointOffset > 8_000_000 {
            return .fallbackToFull
        }

        let fullHeaderData: Data
        switch fetchRangeWithRetry(start: 0, length: UInt64(pointOffset), attempts: 2) {
        case .success(let data):
            fullHeaderData = data
        case .failure:
            return .fallbackToFull
        }

        guard let headerInfo = parseCOPCHeaderInfo(from: fullHeaderData) else {
            return .fallbackToFull
        }

        if isCancelled?() == true {
            return finishCancelled()
        }

        progress?(0.10, "Loading COPC hierarchy...")
        var dataEntries: [COPCHierarchyEntry] = []
        var hierarchyQueue: [(offset: UInt64, size: UInt64)] = [(headerInfo.rootHierarchyOffset, headerInfo.rootHierarchySize)]
        var hierarchySeenOffsets = Set<UInt64>()
        var queueIndex = 0
        var hierarchyPagesRead = 0
        let maxHierarchyPages = 384

        while queueIndex < hierarchyQueue.count && hierarchyPagesRead < maxHierarchyPages {
            if isCancelled?() == true {
                return finishCancelled()
            }

            let page = hierarchyQueue[queueIndex]
            queueIndex += 1
            guard page.size > 0, !hierarchySeenOffsets.contains(page.offset) else { continue }
            hierarchySeenOffsets.insert(page.offset)

            let pageData: Data
            switch fetchRangeWithRetry(start: page.offset, length: page.size, attempts: 3) {
            case .success(let data):
                pageData = data
            case .failure(let error):
                if hierarchyPagesRead == 0 {
                    return .failure(error.localizedDescription)
                }
                continue
            }

            hierarchyPagesRead += 1
            let entries = parseCOPCHierarchyEntries(from: pageData)
            for entry in entries {
                if entry.pointCount > 0, entry.byteSize > 0, entry.offset > 0 {
                    dataEntries.append(entry)
                } else if entry.pointCount == -1, entry.byteSize > 0, entry.offset > 0 {
                    hierarchyQueue.append((entry.offset, UInt64(entry.byteSize)))
                }
            }

            if hierarchyPagesRead == 1 || hierarchyPagesRead % 8 == 0 {
                let phase = min(1.0, Double(hierarchyPagesRead) / 32.0)
                progress?(0.10 + (0.08 * phase), "Loading COPC hierarchy... \(hierarchyPagesRead) page(s)")
            }
        }

        if hierarchyPagesRead >= maxHierarchyPages {
            progress?(0.18, "Hierarchy limit reached, streaming best available detail...")
        }

        if dataEntries.isEmpty {
            return .failure("No streamable nodes found in COPC hierarchy.")
        }

        var uniqueEntriesByOffset: [UInt64: COPCHierarchyEntry] = [:]
        uniqueEntriesByOffset.reserveCapacity(dataEntries.count)
        for entry in dataEntries {
            if let existing = uniqueEntriesByOffset[entry.offset] {
                if entry.level < existing.level || (entry.level == existing.level && entry.pointCount > existing.pointCount) {
                    uniqueEntriesByOffset[entry.offset] = entry
                }
            } else {
                uniqueEntriesByOffset[entry.offset] = entry
            }
        }
        dataEntries = Array(uniqueEntriesByOffset.values)

        dataEntries.sort { a, b in
            if a.level != b.level {
                return a.level < b.level
            }
            return a.pointCount > b.pointCount
        }

        let targetPointBudget = recommendedCOPCStreamPointBudget()
        let previewBudget = min(max(90_000, targetPointBudget / 5), 260_000)
        let previewEntries = selectCOPCEntries(
            from: dataEntries,
            targetPointBudget: previewBudget,
            maxNodes: 12
        )
        let previewOffsets = Set(previewEntries.map(\.offset))
        let previewPointEstimate = previewEntries.reduce(0) { partial, entry in
            partial + max(0, Int(entry.pointCount))
        }
        let refinementBudget = max(0, targetPointBudget - previewPointEstimate)
        let refinementEntries = selectCOPCEntries(
            from: dataEntries,
            targetPointBudget: refinementBudget,
            maxNodes: 160,
            excludingOffsets: previewOffsets
        )
        let selectedEntries = previewEntries + refinementEntries

        if selectedEntries.isEmpty {
            return .failure("No preview nodes selected for COPC stream.")
        }

        DispatchQueue.main.sync {
            self.clearBuffer()
        }

        progress?(0.20, "Streaming preview...")

        var loadedPreviewNodes = 0
        var loadedRefinementNodes = 0
        var loadedAny = false
        var mergedPointCount = 0
        let totalPreviewNodes = max(1, previewEntries.count)
        let totalRefinementNodes = max(1, refinementEntries.count)
        let totalSelectedNodes = max(1, selectedEntries.count)

        for (idx, entry) in selectedEntries.enumerated() {
            if isCancelled?() == true {
                return finishCancelled()
            }

            let isPreviewNode = idx < previewEntries.count
            if isPreviewNode {
                let previewProgress = 0.20 + (Double(idx) / Double(totalPreviewNodes)) * 0.22
                progress?(previewProgress, "Streaming preview node \(idx + 1)/\(previewEntries.count)...")
            } else {
                let refineIdx = idx - previewEntries.count
                let refineProgress = 0.44 + (Double(refineIdx) / Double(totalRefinementNodes)) * 0.54
                progress?(refineProgress, "Refining detail \(refineIdx + 1)/\(refinementEntries.count)...")
            }

            let chunkLength = UInt64(max(0, entry.byteSize))
            guard chunkLength > 0 else { continue }

            let nodeCacheKey = copcNodeCacheKey(for: remoteURL, entry: entry)
            let snapshotData: Data
            if let cachedSnapshot = copcNodeSnapshotCache.object(forKey: nodeCacheKey) {
                snapshotData = Data(referencing: cachedSnapshot)
            } else {
                let chunkData: Data
                switch fetchRangeWithRetry(start: entry.offset, length: chunkLength, attempts: 3) {
                case .success(let data):
                    chunkData = data
                case .failure(let error):
                    if loadedAny {
                        continue
                    }
                    return .failure(error.localizedDescription)
                }

                switch decodeCOPCNodeChunk(chunkData, entry: entry, headerInfo: headerInfo) {
                case .success(let decodedSnapshot):
                    snapshotData = decodedSnapshot
                    copcNodeSnapshotCache.setObject(decodedSnapshot as NSData, forKey: nodeCacheKey, cost: decodedSnapshot.count)
                case .failure(let error):
                    if loadedAny {
                        continue
                    }
                    return .failure(error.localizedDescription)
                }
            }

            DispatchQueue.main.sync {
                let inserted = self.mergeCompactSnapshot(snapshotData)
                if inserted > 0 {
                    loadedAny = true
                    mergedPointCount += inserted
                }
            }

            if isPreviewNode {
                loadedPreviewNodes += 1
                if loadedPreviewNodes == 1 {
                    progress?(0.42, "Preview ready, refining detail...")
                }
            } else {
                loadedRefinementNodes += 1
                if loadedRefinementNodes % 6 == 0 || idx == selectedEntries.count - 1 {
                    let countString = NumberFormatter.localizedString(
                        from: NSNumber(value: max(0, mergedPointCount)),
                        number: .decimal
                    )
                    progress?(
                        0.44 + (Double(loadedRefinementNodes) / Double(totalRefinementNodes)) * 0.54,
                        "Refining detail \(loadedRefinementNodes)/\(refinementEntries.count) • \(countString) pts"
                    )
                }
            }
        }

        if !loadedAny {
            return .failure("Could not stream any COPC nodes.")
        }

        DispatchQueue.main.async {
            self.activePointCount = self.currentPointCount
        }

        let loadedCountString = NumberFormatter.localizedString(from: NSNumber(value: currentPointCount), number: .decimal)
        progress?(
            1.0,
            "High detail ready • \(loadedCountString) pts (\(loadedPreviewNodes + loadedRefinementNodes)/\(totalSelectedNodes) nodes)"
        )
        return .success
    }

    func importPointCloudFromURL(
        _ remoteURL: URL,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil,
        completion: @escaping (Bool, String?) -> Void
    ) {
        DispatchQueue.global(qos: .userInitiated).async {
            func finish(_ success: Bool, _ message: String?) {
                DispatchQueue.main.async {
                    completion(success, message)
                }
            }

            if isCancelled?() == true {
                finish(false, "Import cancelled.")
                return
            }

            switch self.tryImportCOPCStreaming(remoteURL, progress: progress, isCancelled: isCancelled) {
            case .success:
                finish(true, nil)
                return
            case .failure(let message):
                finish(false, message)
                return
            case .fallbackToFull:
                break
            }

            progress?(0.05, "Checking remote file...")

            var headRequest = URLRequest(url: remoteURL)
            headRequest.httpMethod = "HEAD"
            headRequest.timeoutInterval = 20

            let headSemaphore = DispatchSemaphore(value: 0)
            var headResponse: HTTPURLResponse?
            var headError: Error?
            let headSession = URLSession(configuration: .ephemeral)
            defer { headSession.invalidateAndCancel() }
            let headTask = headSession.dataTask(with: headRequest) { _, response, error in
                headResponse = response as? HTTPURLResponse
                headError = error
                headSemaphore.signal()
            }
            headTask.resume()
            headSemaphore.wait()

            // Some servers reject HEAD while allowing GET/range requests.
            // Continue to download unless HEAD clearly reports unsupported access.
            if let headResponse, [401, 403].contains(headResponse.statusCode) {
                finish(false, "Server denied access (HTTP \(headResponse.statusCode)).")
                return
            }
            if headError != nil {
                progress?(0.10, "HEAD unavailable; trying direct download...")
            }

            if isCancelled?() == true {
                finish(false, "Import cancelled.")
                return
            }

            progress?(0.20, "Downloading COPC/LAZ file...")
            let downloadSemaphore = DispatchSemaphore(value: 0)
            var downloadedURL: URL?
            var downloadResponse: HTTPURLResponse?
            var downloadError: Error?
            var lastDownloadPercent = -1
            let downloadDelegate = URLImportDownloadDelegate(
                progressHandler: { fraction, totalBytesWritten, _ in
                    if fraction >= 0 {
                        let percent = Int((fraction * 100).rounded())
                        guard percent != lastDownloadPercent else { return }
                        lastDownloadPercent = percent
                        let mapped = 0.20 + (fraction * 0.50)
                        progress?(mapped, "Downloading COPC/LAZ file... \(percent)%")
                    } else {
                        let writtenMB = Double(totalBytesWritten) / 1_048_576.0
                        progress?(0.24, String(format: "Downloading COPC/LAZ file... %.1f MB", writtenMB))
                    }
                },
                completionHandler: { localURL, response, error in
                    downloadedURL = localURL
                    downloadResponse = response as? HTTPURLResponse
                    downloadError = error
                    downloadSemaphore.signal()
                }
            )
            let downloadSession = URLSession(configuration: .ephemeral, delegate: downloadDelegate, delegateQueue: nil)
            let downloadTask = downloadSession.downloadTask(with: remoteURL)
            downloadTask.resume()

            while true {
                if downloadSemaphore.wait(timeout: .now() + 0.15) == .success {
                    break
                }
                if isCancelled?() == true {
                    downloadTask.cancel()
                    downloadSession.invalidateAndCancel()
                    finish(false, "Import cancelled.")
                    return
                }
            }
            downloadSession.finishTasksAndInvalidate()

            if let downloadError {
                let nsError = downloadError as NSError
                if nsError.domain == NSURLErrorDomain && nsError.code == NSURLErrorCancelled {
                    finish(false, "Import cancelled.")
                    return
                }
                finish(false, "Download failed: \(downloadError.localizedDescription)")
                return
            }
            guard let downloadResponse, (200...299).contains(downloadResponse.statusCode) else {
                finish(false, "Download failed due to server response.")
                return
            }
            guard let localURL = downloadedURL else {
                finish(false, "Download failed: no file received.")
                return
            }

            if isCancelled?() == true {
                try? FileManager.default.removeItem(at: localURL)
                finish(false, "Import cancelled.")
                return
            }

            progress?(0.70, "Decoding point cloud...")
            var snapshotPointer: UnsafeMutablePointer<UInt8>?
            var snapshotSize: UInt32 = 0
            var pointCount: UInt32 = 0
            var errorBuffer = [CChar](repeating: 0, count: 512)
            let decodeCallbacks = LAZDecodeProgressBox(
                onProgress: { fraction in
                    let mapped = 0.70 + (fraction * 0.20)
                    progress?(mapped, "Decoding point cloud... \(Int((fraction * 100).rounded()))%")
                },
                isCancelled: {
                    isCancelled?() == true
                }
            )
            let decodeCallbackContext = Unmanaged.passRetained(decodeCallbacks).toOpaque()
            defer {
                Unmanaged<LAZDecodeProgressBox>.fromOpaque(decodeCallbackContext).release()
            }

            let decodeOK = localURL.path.withCString { pathCString in
                pp_laz_create_snapshot(
                    pathCString,
                    UInt32(self.maxVoxels),
                    lazDecodeProgressThunk,
                    lazDecodeCancelThunk,
                    decodeCallbackContext,
                    &snapshotPointer,
                    &snapshotSize,
                    &pointCount,
                    &errorBuffer,
                    UInt32(errorBuffer.count)
                )
            }
            try? FileManager.default.removeItem(at: localURL)

            guard decodeOK, let snapshotPointer, snapshotSize > 0, pointCount > 0 else {
                let message = String(cString: errorBuffer)
                if isCancelled?() == true || message.localizedCaseInsensitiveContains("cancel") {
                    finish(false, "Import cancelled.")
                } else {
                    finish(false, message.isEmpty ? "Unsupported or invalid LAZ/COPC file." : message)
                }
                return
            }

            let snapshotData = Data(bytes: snapshotPointer, count: Int(snapshotSize))
            pp_free_buffer(snapshotPointer)

            if isCancelled?() == true {
                finish(false, "Import cancelled.")
                return
            }

            progress?(0.90, "Loading point cloud...")
            DispatchQueue.main.async {
                self.clearBuffer()
                self.loadSnapshot(snapshotData, pointCount: Int(pointCount))
                progress?(1.0, "Loaded successfully.")
                completion(true, nil)
            }
        }
    }

    func exportPLYFile(
        fromSnapshot data: Data,
        pointCount: Int,
        suggestedName: String,
        format: PLYExportFormat = .binaryLittleEndian,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> URL? {
        if isCancelled?() == true { return nil }
        progress?(0.0, "Preparing export...")
        let safeBase = sanitizeFileName(suggestedName).isEmpty ? "Scan" : sanitizeFileName(suggestedName)
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let timestamp = formatter.string(from: Date())
        let suffix = format == .binaryLittleEndian ? "binary" : "ascii"
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(safeBase)_\(timestamp)_\(suffix).ply")
        guard writePLY(
            to: url,
            fromSnapshot: data,
            pointCount: pointCount,
            format: format,
            progress: progress,
            isCancelled: isCancelled
        ) else {
            try? FileManager.default.removeItem(at: url)
            return nil
        }
        progress?(1.0, "Finalizing export...")
        return url
    }

    func exportLAZFile(
        fromSnapshot data: Data,
        pointCount: Int,
        suggestedName: String,
        session: ScanSession,
        captureMetadata: ScanCaptureMetadata?,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> ExportArtifact? {
        if isCancelled?() == true { return nil }
        progress?(0.0, "Preparing LAZ export...")
        let safeBase = sanitizeFileName(suggestedName).isEmpty ? "Scan" : sanitizeFileName(suggestedName)
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let timestamp = formatter.string(from: Date())
        let baseName = "\(safeBase)_\(timestamp)"

        let lazURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(baseName).laz")
        let sidecarURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(baseName).json")

        guard let lazResult = writeLAZ(
            to: lazURL,
            fromSnapshot: data,
            pointCount: pointCount,
            captureMetadata: captureMetadata,
            progress: progress,
            isCancelled: isCancelled
        ) else {
            try? FileManager.default.removeItem(at: lazURL)
            return nil
        }

        guard writeLAZSidecar(
            to: sidecarURL,
            lazURL: lazURL,
            session: session,
            captureMetadata: captureMetadata,
            pointCount: lazResult.pointCount,
            scale: lazResult.scale,
            offset: lazResult.offset,
            boundsMin: lazResult.boundsMin,
            boundsMax: lazResult.boundsMax,
            epsg: lazResult.epsg,
            crsName: lazResult.crsName,
            georefNote: lazResult.georefNote,
            headingDegrees: lazResult.headingDegrees
        ) else {
            try? FileManager.default.removeItem(at: lazURL)
            try? FileManager.default.removeItem(at: sidecarURL)
            return nil
        }

        progress?(1.0, "Finalizing export...")
        return ExportArtifact(primaryURL: lazURL, sidecarURL: sidecarURL)
    }

    func exportReportPDFOnly(
        fromSnapshot data: Data,
        pointCount: Int,
        suggestedName: String,
        session: ScanSession,
        captureMetadata: ScanCaptureMetadata?,
        measurements: [ScanReportMeasurement],
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> URL? {
        let safeBase = sanitizeFileName(suggestedName).isEmpty ? "Scan" : sanitizeFileName(suggestedName)
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let timestamp = formatter.string(from: Date())
        let reportURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("\(safeBase)_\(timestamp)_report.pdf")

        return generateExportReportPDF(
            fromSnapshot: data,
            pointCount: pointCount,
            session: session,
            captureMetadata: captureMetadata,
            exportFormat: .pdfReport,
            primaryURL: nil,
            sidecarURL: nil,
            reportURLOverride: reportURL,
            measurements: measurements,
            progress: progress,
            isCancelled: isCancelled
        )
    }

    func generateExportReportPDF(
        fromSnapshot data: Data,
        pointCount: Int,
        session: ScanSession,
        captureMetadata: ScanCaptureMetadata?,
        exportFormat: ExportFormat,
        primaryURL: URL?,
        sidecarURL: URL?,
        reportURLOverride: URL? = nil,
        measurements: [ScanReportMeasurement],
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> URL? {
        if isCancelled?() == true { return nil }
        progress?(0.02, "Analyzing point cloud...")

        guard let stats = computeSnapshotPointStats(
            fromSnapshot: data,
            fallbackPointCount: pointCount,
            exportFormat: exportFormat,
            captureMetadata: captureMetadata,
            isCancelled: isCancelled
        ) else { return nil }
        progress?(0.12, "Preparing report assets...")

        let reportURL: URL = {
            if let reportURLOverride { return reportURLOverride }
            if let primaryURL {
                let baseName = primaryURL.deletingPathExtension().lastPathComponent
                return primaryURL.deletingLastPathComponent()
                    .appendingPathComponent("\(baseName)_report.pdf")
            }
            let safeBase = sanitizeFileName(session.name).isEmpty ? "Scan" : sanitizeFileName(session.name)
            let formatter = DateFormatter()
            formatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
            let timestamp = formatter.string(from: Date())
            return FileManager.default.temporaryDirectory
                .appendingPathComponent("\(safeBase)_\(timestamp)_report.pdf")
        }()

        let previewPoints = extractReportPreviewPoints(
            fromSnapshot: data,
            exportFormat: exportFormat,
            captureMetadata: captureMetadata,
            targetMaxPoints: 60_000,
            isCancelled: isCancelled
        )
        let analysisPoints = extractReportPreviewPoints(
            fromSnapshot: data,
            exportFormat: exportFormat,
            captureMetadata: captureMetadata,
            targetMaxPoints: 180_000,
            isCancelled: isCancelled
        )
        progress?(0.24, "Rendering visual evidence...")
        let rgbPreviewPanel = previewPoints.flatMap {
            makeReportPreviewPanelImage(
                points: $0,
                measurements: measurements,
                exportFormat: exportFormat,
                captureMetadata: captureMetadata,
                style: .rgb
            )
        }
        progress?(0.36, "Rendering elevation visual evidence...")
        let elevationPreviewPanel = previewPoints.flatMap {
            makeReportPreviewPanelImage(
                points: $0,
                measurements: measurements,
                exportFormat: exportFormat,
                captureMetadata: captureMetadata,
                style: .elevation
            )
        }
        progress?(0.46, "Computing measurement analysis...")
        let measurementAnalysisByID: [String: MeasurementPointCloudAnalysis] = {
            guard let analysisPoints else { return [:] }
            return buildMeasurementAnalyses(
                measurements: measurements,
                cloudPoints: analysisPoints,
                exportFormat: exportFormat,
                captureMetadata: captureMetadata
            )
        }()
        progress?(0.56, "Generating report pages...")

        let pageRect = CGRect(x: 0, y: 0, width: 612, height: 792)
        let renderer = UIGraphicsPDFRenderer(bounds: pageRect)

        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd HH:mm:ss zzz"
        let numberFormatter = NumberFormatter()
        numberFormatter.maximumFractionDigits = 2
        numberFormatter.minimumFractionDigits = 2
        let oneDecimalFormatter = NumberFormatter()
        oneDecimalFormatter.maximumFractionDigits = 1
        oneDecimalFormatter.minimumFractionDigits = 1

        let byteFormatter = ByteCountFormatter()
        byteFormatter.allowedUnits = [.useKB, .useMB, .useGB]
        byteFormatter.countStyle = .file
        byteFormatter.isAdaptive = true

        let exportFormatLabel: String = {
            switch exportFormat {
            case .laz: return "LAZ"
            case .plyBinaryLittleEndian: return "PLY (Binary)"
            case .plyAscii: return "PLY (ASCII)"
            case .pdfReport: return "PDF Report"
            }
        }()

        let georefMode: String = {
            if captureMetadata?.location != nil && exportFormat == .laz { return "GPS Approximate" }
            if captureMetadata?.location != nil { return "GPS Captured (local coordinates)" }
            return "Local Only"
        }()

        let qualityStatus: String = {
            if stats.pointCount >= 200_000 { return "GOOD" }
            if stats.pointCount >= 50_000 { return "WARNING" }
            return "POOR"
        }()

        let qualityScore: Int = {
            if stats.pointCount <= 0 { return 0 }
            let normalized = min(1.0, max(0.0, log10(Double(stats.pointCount)) / 6.0))
            return Int((normalized * 100.0).rounded())
        }()

        progress?(0.62, "Composing report pages...")
        do {
            try renderer.writePDF(to: reportURL) { ctx in
                let brandBlue = UIColor.systemBlue
                let bodyColor = UIColor.black
                let mutedColor = UIColor(white: 0.25, alpha: 1.0)
                let cardStroke = UIColor(white: 0.86, alpha: 1.0)
                let margin: CGFloat = 36
                let contentWidth = pageRect.width - margin * 2
                var y: CGFloat = margin + 16
                var pageIndex = 1
                var sectionTopY: CGFloat?

                let shortDateFormatter = DateFormatter()
                shortDateFormatter.dateFormat = "yyyy-MM-dd HH:mm"

                func beginPage() {
                    ctx.beginPage()
                    y = margin + 16

                    let topRule = UIBezierPath()
                    topRule.move(to: CGPoint(x: margin, y: 30))
                    topRule.addLine(to: CGPoint(x: pageRect.width - margin, y: 30))
                    brandBlue.setStroke()
                    topRule.lineWidth = 1.0
                    topRule.stroke()

                    "PointPro Point Cloud Survey Report".draw(
                        at: CGPoint(x: margin, y: 14),
                        withAttributes: [
                            .font: UIFont.systemFont(ofSize: 10, weight: .semibold),
                            .foregroundColor: mutedColor
                        ]
                    )

                    let footerBrand = "POINTPRO"
                    footerBrand.draw(
                        at: CGPoint(x: margin, y: pageRect.height - 22),
                        withAttributes: [
                            .font: UIFont.systemFont(ofSize: 9, weight: .bold),
                            .foregroundColor: brandBlue
                        ]
                    )

                    let footerTagline = "Field Capture to Deliverable"
                    let tagSize = footerTagline.size(withAttributes: [.font: UIFont.systemFont(ofSize: 9)])
                    footerTagline.draw(
                        at: CGPoint(x: pageRect.midX - (tagSize.width / 2), y: pageRect.height - 22),
                        withAttributes: [
                            .font: UIFont.systemFont(ofSize: 9),
                            .foregroundColor: mutedColor
                        ]
                    )

                    let footerPage = "Page \(pageIndex)"
                    let footerPageSize = footerPage.size(withAttributes: [.font: UIFont.systemFont(ofSize: 9)])
                    footerPage.draw(
                        at: CGPoint(x: pageRect.width - margin - footerPageSize.width, y: pageRect.height - 22),
                        withAttributes: [
                            .font: UIFont.systemFont(ofSize: 9),
                            .foregroundColor: mutedColor
                        ]
                    )

                    pageIndex += 1
                }

                func finishSection() {
                    guard let top = sectionTopY else { return }
                    let rect = CGRect(x: margin, y: top, width: contentWidth, height: max(30, y - top + 8))
                    let path = UIBezierPath(roundedRect: rect, cornerRadius: 8)
                    cardStroke.setStroke()
                    path.lineWidth = 0.8
                    path.stroke()
                    sectionTopY = nil
                    y = rect.maxY + 10
                }

                func ensureSpace(_ height: CGFloat) {
                    if y + height > pageRect.height - margin - 34 {
                        finishSection()
                        beginPage()
                    }
                }

                func lineX() -> CGFloat { sectionTopY == nil ? margin : (margin + 12) }
                func lineWidth() -> CGFloat { sectionTopY == nil ? contentWidth : (contentWidth - 24) }

                @discardableResult
                func drawLine(_ text: String, font: UIFont, color: UIColor = .black, spacing: CGFloat = 4) -> CGFloat {
                    let attributes: [NSAttributedString.Key: Any] = [
                        .font: font,
                        .foregroundColor: color
                    ]
                    let width = lineWidth()
                    let x = lineX()
                    let rect = NSString(string: text).boundingRect(
                        with: CGSize(width: width, height: .greatestFiniteMagnitude),
                        options: [.usesLineFragmentOrigin, .usesFontLeading],
                        attributes: attributes,
                        context: nil
                    )
                    let drawRect = CGRect(x: x, y: y, width: width, height: ceil(rect.height))
                    ensureSpace(drawRect.height + spacing)
                    NSString(string: text).draw(in: drawRect, withAttributes: attributes)
                    y += ceil(rect.height) + spacing
                    return ceil(rect.height) + spacing
                }

                func startSection(_ title: String, minBodyHeight: CGFloat = 52) {
                    finishSection()
                    ensureSpace(minBodyHeight + 30)
                    sectionTopY = y
                    y += 10
                    _ = drawLine(title.uppercased(), font: UIFont.systemFont(ofSize: 12.5, weight: .bold), color: brandBlue, spacing: 6)
                }

                func drawKV(_ key: String, _ value: String) {
                    let line = "\(key): \(value)"
                    _ = drawLine(line, font: UIFont.monospacedSystemFont(ofSize: 10.5, weight: .regular), color: bodyColor, spacing: 3)
                }

                beginPage()
                _ = drawLine("Point Cloud Survey Report", font: UIFont.systemFont(ofSize: 24, weight: .bold), color: bodyColor, spacing: 8)
                _ = drawLine(session.name, font: UIFont.systemFont(ofSize: 16, weight: .semibold), color: bodyColor, spacing: 14)

                startSection("Document Control", minBodyHeight: 72)
                drawKV("Report ID", UUID().uuidString)
                drawKV("Generated At", dateFormatter.string(from: Date()))
                drawKV("Timezone", TimeZone.current.identifier)
                drawKV("Template Version", "enterprise-v1")
                drawKV("Export Format", exportFormatLabel)

                startSection("Executive Summary", minBodyHeight: 84)
                drawKV("Points", "\(stats.pointCount)")
                drawKV("Extent X", "\(numberFormatter.string(from: NSNumber(value: stats.extentX)) ?? "0.00") m")
                drawKV("Extent Y", "\(numberFormatter.string(from: NSNumber(value: stats.extentY)) ?? "0.00") m")
                drawKV("Extent Z", "\(numberFormatter.string(from: NSNumber(value: stats.extentZ)) ?? "0.00") m")
                drawKV("Georeference Mode", georefMode)
                drawKV("Measurements", "\(measurements.count)")

                startSection("Scan Metadata", minBodyHeight: 90)
                drawKV("Scan ID", session.id.uuidString)
                drawKV("Created", dateFormatter.string(from: session.createdAt))
                drawKV("Updated", dateFormatter.string(from: session.updatedAt))
                drawKV("Status", session.status.rawValue.uppercased())
                drawKV("App Version", captureMetadata?.appVersion ?? "Unknown")
                drawKV("Device", captureMetadata?.deviceModel ?? "Unknown")
                drawKV("OS", "\(captureMetadata?.systemName ?? "iOS") \(captureMetadata?.systemVersion ?? "")")

                startSection("Spatial Reference / Georeferencing", minBodyHeight: 88)
                let epsg = inferredEPSG(from: captureMetadata?.location)
                drawKV("Mode", georefMode)
                drawKV("EPSG", epsg.map(String.init) ?? "N/A")
                drawKV("CRS", inferredCRSName(from: captureMetadata?.location, epsg: epsg) ?? "Local Frame")
                if let location = captureMetadata?.location {
                    drawKV("Lat / Lon", String(format: "%.6f, %.6f", location.latitude, location.longitude))
                    drawKV("Altitude", String(format: "%.1f m", location.altitude))
                    if location.horizontalAccuracy > 0 {
                        drawKV("Horizontal Accuracy", "\(oneDecimalFormatter.string(from: NSNumber(value: location.horizontalAccuracy)) ?? "N/A") m")
                    } else {
                        drawKV("Horizontal Accuracy", "Unknown")
                    }
                    if let heading = location.headingDegrees {
                        drawKV("Heading", String(format: "%.1f°", heading))
                    } else {
                        drawKV("Heading", "N/A")
                    }
                } else {
                    drawKV("Anchor", "Not available")
                }

                startSection("Data Inventory", minBodyHeight: 72)
                if let primaryURL {
                    let primarySize = (try? FileManager.default.attributesOfItem(atPath: primaryURL.path)[.size] as? NSNumber)?.int64Value ?? 0
                    drawKV("Primary File", primaryURL.lastPathComponent)
                    drawKV("Primary Size", byteFormatter.string(fromByteCount: primarySize))
                } else {
                    drawKV("Primary File", "Report Only")
                    drawKV("Primary Size", "N/A")
                }
                if let sidecarURL {
                    let sidecarSize = (try? FileManager.default.attributesOfItem(atPath: sidecarURL.path)[.size] as? NSNumber)?.int64Value ?? 0
                    drawKV("Metadata File", sidecarURL.lastPathComponent)
                    drawKV("Metadata Size", byteFormatter.string(fromByteCount: sidecarSize))
                } else {
                    drawKV("Metadata File", "Not generated")
                }
                drawKV("Report File", reportURL.lastPathComponent)

                startSection("Point Cloud Statistics", minBodyHeight: 74)
                drawKV("Point Count", "\(stats.pointCount)")
                drawKV("Bounds Min", String(format: "[%.3f, %.3f, %.3f]", stats.boundsMin[0], stats.boundsMin[1], stats.boundsMin[2]))
                drawKV("Bounds Max", String(format: "[%.3f, %.3f, %.3f]", stats.boundsMax[0], stats.boundsMax[1], stats.boundsMax[2]))
                drawKV("Extent (X/Y/Z)", String(format: "%.3f / %.3f / %.3f m", stats.extentX, stats.extentY, stats.extentZ))
                drawKV("Color", "RGB")

                startSection("Quality and Risk", minBodyHeight: 48)
                drawKV("Quality Score", "\(qualityScore)/100")
                drawKV("Coverage Status", qualityStatus)
                drawKV("Risk Note", "GPS georeference is approximate; use GCP/checkpoints for high-accuracy workflows.")
                progress?(0.72, "Writing measurement sections...")

                startSection("Measurement Register", minBodyHeight: measurements.isEmpty ? 40 : 92)
                if measurements.isEmpty {
                    drawKV("Measurements", "No measurements recorded")
                } else {
                    let tableX = lineX()
                    let tableWidth = lineWidth()
                    let headerH: CGFloat = 18
                    let rowH: CGFloat = 18
                    let columns: [CGFloat] = [0.00, 0.09, 0.22, 0.31, 0.43, 0.55, 0.67, 1.00]

                    func cellRect(_ rowY: CGFloat, _ col: Int, _ height: CGFloat) -> CGRect {
                        let left = tableX + (columns[col] * tableWidth)
                        let right = tableX + (columns[col + 1] * tableWidth)
                        return CGRect(x: left + 3, y: rowY + 3, width: max(0, right - left - 6), height: max(0, height - 6))
                    }

                    func drawCellText(_ text: String, _ rect: CGRect, _ font: UIFont, _ color: UIColor = .black) {
                        NSString(string: text).draw(
                            in: rect,
                            withAttributes: [
                                .font: font,
                                .foregroundColor: color
                            ]
                        )
                    }

                    ensureSpace(headerH + rowH + 10)
                    let headerRect = CGRect(x: tableX, y: y, width: tableWidth, height: headerH)
                    UIColor(white: 0.95, alpha: 1).setFill()
                    UIRectFill(headerRect)
                    UIColor(white: 0.82, alpha: 1).setStroke()
                    UIBezierPath(rect: headerRect).stroke()

                    let headerFont = UIFont.systemFont(ofSize: 8.2, weight: .bold)
                    drawCellText("#", cellRect(y, 0, headerH), headerFont)
                    drawCellText("Type", cellRect(y, 1, headerH), headerFont)
                    drawCellText("Vtx", cellRect(y, 2, headerH), headerFont)
                    drawCellText("Len", cellRect(y, 3, headerH), headerFont)
                    drawCellText("Perim", cellRect(y, 4, headerH), headerFont)
                    drawCellText("Area", cellRect(y, 5, headerH), headerFont)
                    drawCellText("Created", cellRect(y, 6, headerH), headerFont)
                    y += headerH

                    for (idx, measurement) in measurements.enumerated() {
                        if y + rowH > pageRect.height - margin - 40 {
                            finishSection()
                            beginPage()
                            startSection("Measurement Register (cont.)", minBodyHeight: 92)
                            ensureSpace(headerH + rowH + 8)
                            let continuedHeaderRect = CGRect(x: tableX, y: y, width: tableWidth, height: headerH)
                            UIColor(white: 0.95, alpha: 1).setFill()
                            UIRectFill(continuedHeaderRect)
                            UIColor(white: 0.82, alpha: 1).setStroke()
                            UIBezierPath(rect: continuedHeaderRect).stroke()
                            drawCellText("#", cellRect(y, 0, headerH), headerFont)
                            drawCellText("Type", cellRect(y, 1, headerH), headerFont)
                            drawCellText("Vtx", cellRect(y, 2, headerH), headerFont)
                            drawCellText("Len", cellRect(y, 3, headerH), headerFont)
                            drawCellText("Perim", cellRect(y, 4, headerH), headerFont)
                            drawCellText("Area", cellRect(y, 5, headerH), headerFont)
                            drawCellText("Created", cellRect(y, 6, headerH), headerFont)
                            y += headerH
                        }

                        let rowRect = CGRect(x: tableX, y: y, width: tableWidth, height: rowH)
                        UIColor(white: 0.92, alpha: idx % 2 == 0 ? 0.0 : 0.35).setFill()
                        UIRectFill(rowRect)
                        UIColor(white: 0.87, alpha: 1).setStroke()
                        UIBezierPath(rect: rowRect).stroke()

                        let typeLabel: String = {
                            switch measurement.type {
                            case .distance: return "Distance"
                            case .polyline: return "Polyline"
                            case .closedArea: return "Area"
                            case .crossSection: return "Cross"
                            case .elevationProfile: return "Elev"
                            }
                        }()
                        let rowFont = UIFont.systemFont(ofSize: 8.2, weight: .regular)
                        drawCellText("M\(idx + 1)", cellRect(y, 0, rowH), rowFont)
                        drawCellText(typeLabel, cellRect(y, 1, rowH), rowFont)
                        drawCellText("\(measurement.vertexCount)", cellRect(y, 2, rowH), rowFont)
                        drawCellText(measurement.lengthMeters.map { String(format: "%.2f", $0) } ?? "-", cellRect(y, 3, rowH), rowFont)
                        drawCellText(measurement.perimeterMeters.map { String(format: "%.2f", $0) } ?? "-", cellRect(y, 4, rowH), rowFont)
                        drawCellText(measurement.areaSquareMeters.map { String(format: "%.2f", $0) } ?? "-", cellRect(y, 5, rowH), rowFont)
                        drawCellText(shortDateFormatter.string(from: measurement.createdAt), cellRect(y, 6, rowH), UIFont.systemFont(ofSize: 7.7))
                        y += rowH
                    }

                    y += 3
                    _ = drawLine("Legend: Preview image uses the same IDs (M1, M2, ...).", font: UIFont.systemFont(ofSize: 9.2), color: mutedColor, spacing: 2)
                }

                let indexedMeasurements: [(Int, ScanReportMeasurement)] = measurements.enumerated().map { ($0 + 1, $1) }
                let crossSections = indexedMeasurements.filter { $0.1.type == .crossSection }
                let elevationProfiles = indexedMeasurements.filter { $0.1.type == .elevationProfile }

                if !crossSections.isEmpty {
                    startSection("Cross Section Analysis", minBodyHeight: 74)
                    for (index, measurement) in crossSections {
                        let analysis = measurementAnalysisByID[measurement.id]
                        let minElevation = analysis?.minElevation ?? measurement.minElevationMeters
                        let maxElevation = analysis?.maxElevation ?? measurement.maxElevationMeters
                        let relief = analysis?.relief ?? measurement.reliefMeters
                        ensureSpace(214)
                        drawKV("ID", "M\(index)")
                        drawKV("Path Length", (analysis?.profileLength ?? measurement.lengthMeters).map { String(format: "%.2f m", $0) } ?? "N/A")
                        drawKV("Swath Width", measurement.swathWidthMeters.map { String(format: "%.2f m", $0) } ?? "N/A")
                        let binLabel = analysis?.profileBinWidthMeters.map { String(format: "%.3f m", $0) } ?? "N/A"
                        let windowLabel = analysis?.stationWindowMeters.map { String(format: "%.3f m", $0) } ?? "N/A"
                        drawKV("Sample Bin / Station Window", "\(binLabel) / \(windowLabel)")
                        let sliceLabel = analysis.map { String($0.sectionScatter.count) } ?? "N/A"
                        let corridorLabel = analysis.map { String($0.corridorPointCount) } ?? "N/A"
                        let cloudLabel = analysis.map { String($0.sourcePointCount) } ?? "N/A"
                        drawKV("Sampled Points (Slice / Corridor / Cloud)", "\(sliceLabel) / \(corridorLabel) / \(cloudLabel)")
                        if let minElevation, let maxElevation {
                            drawKV("Elevation Min / Max", String(format: "%.2f / %.2f m", minElevation, maxElevation))
                        } else {
                            drawKV("Elevation Min / Max", "N/A")
                        }
                        drawKV("Relief", relief.map { String(format: "%.2f m", $0) } ?? "N/A")
                        if let graphImage = makeMeasurementGraphImage(
                            measurement: measurement,
                            analysis: analysis,
                            title: "Cross Section M\(index)",
                            lineColor: .systemGreen
                        ) {
                            let graphHeight = min(140, max(108, lineWidth() * 0.30))
                            ensureSpace(graphHeight + 10)
                            graphImage.draw(in: CGRect(x: lineX(), y: y, width: lineWidth(), height: graphHeight))
                            y += graphHeight + 5
                        }
                        _ = drawLine(" ", font: UIFont.systemFont(ofSize: 4), color: .clear, spacing: 1)
                    }
                }

                if !elevationProfiles.isEmpty {
                    startSection("Elevation Profile Analysis", minBodyHeight: 84)
                    for (index, measurement) in elevationProfiles {
                        let analysis = measurementAnalysisByID[measurement.id]
                        let minElevation = analysis?.minElevation ?? measurement.minElevationMeters
                        let maxElevation = analysis?.maxElevation ?? measurement.maxElevationMeters
                        let totalRise = analysis?.totalRise ?? measurement.totalRiseMeters
                        let totalFall = analysis?.totalFall ?? measurement.totalFallMeters
                        let averageSlope = analysis?.averageSlopePercent ?? measurement.averageSlopePercent
                        let maxSlope = analysis?.maxSlopePercent ?? measurement.maxSlopePercent
                        ensureSpace(214)
                        drawKV("ID", "M\(index)")
                        drawKV("Profile Length", (analysis?.profileLength ?? measurement.lengthMeters).map { String(format: "%.2f m", $0) } ?? "N/A")
                        drawKV("Sample Bin Width", analysis?.profileBinWidthMeters.map { String(format: "%.3f m", $0) } ?? "N/A")
                        let corridorLabel = analysis.map { String($0.corridorPointCount) } ?? "N/A"
                        let cloudLabel = analysis.map { String($0.sourcePointCount) } ?? "N/A"
                        drawKV("Sampled Points (Corridor / Cloud)", "\(corridorLabel) / \(cloudLabel)")
                        if let minElevation, let maxElevation {
                            drawKV("Elevation Min / Max", String(format: "%.2f / %.2f m", minElevation, maxElevation))
                        } else {
                            drawKV("Elevation Min / Max", "N/A")
                        }
                        drawKV("Total Rise / Fall", String(
                            format: "%.2f / %.2f m",
                            totalRise ?? 0,
                            totalFall ?? 0
                        ))
                        drawKV("Average / Max Slope", String(
                            format: "%.1f%% / %.1f%%",
                            averageSlope ?? 0,
                            maxSlope ?? 0
                        ))
                        if let graphImage = makeMeasurementGraphImage(
                            measurement: measurement,
                            analysis: analysis,
                            title: "Elevation Profile M\(index)",
                            lineColor: .systemBlue
                        ) {
                            let graphHeight = min(150, max(116, lineWidth() * 0.32))
                            ensureSpace(graphHeight + 10)
                            graphImage.draw(in: CGRect(x: lineX(), y: y, width: lineWidth(), height: graphHeight))
                            y += graphHeight + 5
                        }
                        _ = drawLine(" ", font: UIFont.systemFont(ofSize: 4), color: .clear, spacing: 1)
                    }
                }
                progress?(0.84, "Writing visual evidence...")

                startSection("Visual Evidence", minBodyHeight: 340)
                if let rgbPreviewPanel {
                    _ = drawLine("RGB Multi-view (Front / Back / Top / Side)", font: UIFont.systemFont(ofSize: 9.8, weight: .semibold), color: bodyColor, spacing: 4)
                    let ratio = rgbPreviewPanel.size.height / max(rgbPreviewPanel.size.width, 1)
                    let imageHeight = min(300, max(190, lineWidth() * ratio))
                    ensureSpace(imageHeight + 12)
                    rgbPreviewPanel.draw(in: CGRect(x: lineX(), y: y, width: lineWidth(), height: imageHeight))
                    y += imageHeight + 6
                    _ = drawLine("Point cloud rendered in captured RGB with measurement overlays.", font: UIFont.systemFont(ofSize: 9.2), color: mutedColor, spacing: 5)
                }
                if let elevationPreviewPanel {
                    _ = drawLine("Elevation Gradient Multi-view (Front / Back / Top / Side)", font: UIFont.systemFont(ofSize: 9.8, weight: .semibold), color: bodyColor, spacing: 4)
                    let ratio = elevationPreviewPanel.size.height / max(elevationPreviewPanel.size.width, 1)
                    let imageHeight = min(300, max(190, lineWidth() * ratio))
                    ensureSpace(imageHeight + 12)
                    elevationPreviewPanel.draw(in: CGRect(x: lineX(), y: y, width: lineWidth(), height: imageHeight))
                    y += imageHeight + 6
                    _ = drawLine("Point cloud rendered by elevation gradient (low to high).", font: UIFont.systemFont(ofSize: 9.2), color: mutedColor, spacing: 2)
                }
                if rgbPreviewPanel == nil && elevationPreviewPanel == nil {
                    drawKV("Preview", "Unavailable")
                } else {
                    _ = drawLine("Legend IDs map to Measurement Register (M1, M2, ...).", font: UIFont.systemFont(ofSize: 9.2), color: mutedColor, spacing: 2)
                }

                startSection("Deliverables Checklist", minBodyHeight: 40)
                drawKV("Primary Export", primaryURL == nil ? "Not Included (Report Only)" : "Included")
                drawKV("Metadata Sidecar", sidecarURL == nil ? "Not Included" : "Included")
                drawKV("PDF Report", "Included")

                startSection("Compliance + Disclaimer", minBodyHeight: 36)
                _ = drawLine("GPS approximate only. Not survey-grade georeferencing.", font: UIFont.systemFont(ofSize: 10.5, weight: .regular), color: bodyColor, spacing: 2)
                _ = drawLine("Use GCP/checkpoints for high-accuracy workflows.", font: UIFont.systemFont(ofSize: 10.5, weight: .regular), color: bodyColor, spacing: 8)
                progress?(0.93, "Finalizing annex...")

                startSection("Technical Annex", minBodyHeight: 44)
                drawKV("Depth Resolution", captureMetadata?.depthResolution.map(String.init).joined(separator: "x") ?? "Unknown")
                drawKV("Color Resolution", captureMetadata?.colorResolution.map(String.init).joined(separator: "x") ?? "Unknown")
                if let transform = captureMetadata?.worldOriginTransform {
                    let preview = transform.prefix(8).map { String(format: "%.4f", $0) }.joined(separator: ", ")
                    drawKV("World Origin Matrix (partial)", preview)
                } else {
                    drawKV("World Origin Matrix", "Not available")
                }

                finishSection()
            }
        } catch {
            try? FileManager.default.removeItem(at: reportURL)
            return nil
        }

        progress?(1.0, "Finalizing report...")
        return reportURL
    }

    private struct LAZWriteResult {
        let pointCount: Int
        let scale: [Double]
        let offset: [Double]
        let boundsMin: [Double]
        let boundsMax: [Double]
        let epsg: Int?
        let crsName: String?
        let georefNote: String?
        let headingDegrees: Double?
    }

    private struct SnapshotPointStats {
        let pointCount: Int
        let boundsMin: [Double]
        let boundsMax: [Double]
        let extentX: Double
        let extentY: Double
        let extentZ: Double
    }

    private func inferredEPSG(from location: CaptureLocationMetadata?) -> Int? {
        guard let location else { return nil }
        let latitude = location.latitude
        let longitude = location.longitude
        guard latitude.isFinite, longitude.isFinite else { return nil }
        guard abs(latitude) <= 84.0, abs(longitude) <= 180.0 else { return nil }
        let zone = max(1, min(60, Int(floor((longitude + 180.0) / 6.0)) + 1))
        return (latitude >= 0 ? 32600 : 32700) + zone
    }

    private func inferredCRSName(from location: CaptureLocationMetadata?, epsg: Int?) -> String? {
        guard let location, epsg != nil else { return nil }
        let zone = max(1, min(60, Int(floor((location.longitude + 180.0) / 6.0)) + 1))
        let hemisphere = location.latitude >= 0 ? "N" : "S"
        return "WGS 84 / UTM zone \(zone)\(hemisphere)"
    }

    private func computeSnapshotPointStats(
        fromSnapshot data: Data,
        fallbackPointCount: Int,
        exportFormat: ExportFormat,
        captureMetadata: ScanCaptureMetadata?,
        isCancelled: (() -> Bool)? = nil
    ) -> SnapshotPointStats? {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        let crsContext = exportFormat == .laz ? makeApproximateCRSContext(captureMetadata: captureMetadata) : nil
        func transformedPoint(_ voxel: Voxel) -> (x: Double, y: Double, z: Double) {
            let px = Double(voxel.positionAndConfidence.x)
            let py = Double(voxel.positionAndConfidence.y)
            let pz = Double(voxel.positionAndConfidence.z)
            if let crsContext {
                return crsContext.transform(localX: px, localY: py, localZ: pz)
            }
            return (px, py, pz)
        }

        var minX = Double.greatestFiniteMagnitude
        var minY = Double.greatestFiniteMagnitude
        var minZ = Double.greatestFiniteMagnitude
        var maxX = -Double.greatestFiniteMagnitude
        var maxY = -Double.greatestFiniteMagnitude
        var maxZ = -Double.greatestFiniteMagnitude
        var counted = 0

        let isCompact = data.count >= headerSize &&
            readUInt32(at: 0) == 0x50435331 &&
            readUInt32(at: MemoryLayout<UInt32>.size) == 1

        if isCompact, let storedCount = readUInt32(at: 2 * MemoryLayout<UInt32>.size) {
            let count = min(Int(storedCount), max(0, (data.count - headerSize) / recordSize))
            var offset = headerSize
            for _ in 0..<count {
                if isCancelled?() == true { return nil }
                guard offset + recordSize <= data.count else { break }
                let idx = readUInt32(at: offset) ?? UInt32.max
                offset += MemoryLayout<UInt32>.size
                guard idx < UInt32(maxVoxels), let voxel = readVoxel(at: offset) else {
                    offset += voxelStride
                    continue
                }
                offset += voxelStride
                let point = transformedPoint(voxel)
                minX = min(minX, point.x)
                minY = min(minY, point.y)
                minZ = min(minZ, point.z)
                maxX = max(maxX, point.x)
                maxY = max(maxY, point.y)
                maxZ = max(maxZ, point.z)
                counted += 1
            }
        } else if data.count == voxelBuffer.length {
            for offset in stride(from: 0, to: data.count, by: voxelStride) {
                if isCancelled?() == true { return nil }
                guard let voxel = readVoxel(at: offset) else { break }
                if voxel.colorAndSampleCount.w <= 0 {
                    continue
                }
                let point = transformedPoint(voxel)
                minX = min(minX, point.x)
                minY = min(minY, point.y)
                minZ = min(minZ, point.z)
                maxX = max(maxX, point.x)
                maxY = max(maxY, point.y)
                maxZ = max(maxZ, point.z)
                counted += 1
            }
        } else {
            return nil
        }

        let finalCount = max(counted, max(0, fallbackPointCount))
        guard finalCount > 0 else { return nil }
        guard minX.isFinite, minY.isFinite, minZ.isFinite, maxX.isFinite, maxY.isFinite, maxZ.isFinite else {
            return nil
        }
        return SnapshotPointStats(
            pointCount: finalCount,
            boundsMin: [minX, minY, minZ],
            boundsMax: [maxX, maxY, maxZ],
            extentX: max(0, maxX - minX),
            extentY: max(0, maxY - minY),
            extentZ: max(0, maxZ - minZ)
        )
    }

    private enum ReportPreviewStyle {
        case rgb
        case elevation
    }

    private enum ReportPreviewViewKind: CaseIterable {
        case front
        case back
        case top
        case side

        var title: String {
            switch self {
            case .front: return "Front"
            case .back: return "Back"
            case .top: return "Top"
            case .side: return "Side"
            }
        }

        func project(_ p: SIMD3<Double>) -> CGPoint {
            switch self {
            case .front:
                return CGPoint(x: p.x, y: p.y)
            case .back:
                return CGPoint(x: -p.x, y: p.y)
            case .top:
                return CGPoint(x: p.x, y: p.z)
            case .side:
                return CGPoint(x: p.z, y: p.y)
            }
        }
    }

    private struct ReportPreviewPoint {
        let position: SIMD3<Double>
        let color: UIColor
    }

    private func extractReportPreviewPoints(
        fromSnapshot data: Data,
        exportFormat: ExportFormat,
        captureMetadata: ScanCaptureMetadata?,
        targetMaxPoints: Int = 60_000,
        isCancelled: (() -> Bool)? = nil
    ) -> [ReportPreviewPoint]? {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        let crsContext = exportFormat == .laz ? makeApproximateCRSContext(captureMetadata: captureMetadata) : nil
        func transformedPoint(_ x: Double, _ y: Double, _ z: Double) -> SIMD3<Double> {
            if let crsContext {
                let t = crsContext.transform(localX: x, localY: y, localZ: z)
                return SIMD3<Double>(t.x, t.y, t.z)
            }
            return SIMD3<Double>(x, y, z)
        }

        func voxelColor(_ voxel: Voxel) -> UIColor {
            func normalized(_ value: Float) -> CGFloat {
                let scaled = value <= 1.0 ? value * 255.0 : value
                return CGFloat(max(0, min(255, Int(scaled.rounded())))) / 255.0
            }
            let r = normalized(voxel.colorAndSampleCount.x)
            let g = normalized(voxel.colorAndSampleCount.y)
            let b = normalized(voxel.colorAndSampleCount.z)
            return UIColor(red: r, green: g, blue: b, alpha: 1.0)
        }

        let targetCount = max(5_000, targetMaxPoints)
        var samples: [ReportPreviewPoint] = []
        samples.reserveCapacity(min(targetCount + 5_000, max(10_000, targetCount)))

        let isCompact = data.count >= headerSize &&
            readUInt32(at: 0) == 0x50435331 &&
            readUInt32(at: MemoryLayout<UInt32>.size) == 1

        if isCompact, let storedCount = readUInt32(at: 2 * MemoryLayout<UInt32>.size) {
            let count = min(Int(storedCount), max(0, (data.count - headerSize) / recordSize))
            let sampleStride = max(1, count / targetCount)
            var offset = headerSize
            for i in 0..<count {
                if isCancelled?() == true { return nil }
                guard offset + recordSize <= data.count else { break }
                let idx = readUInt32(at: offset) ?? UInt32.max
                offset += MemoryLayout<UInt32>.size
                guard idx < UInt32(maxVoxels), let voxel = readVoxel(at: offset) else {
                    offset += voxelStride
                    continue
                }
                offset += voxelStride
                guard i % sampleStride == 0 else { continue }
                let p = transformedPoint(
                    Double(voxel.positionAndConfidence.x),
                    Double(voxel.positionAndConfidence.y),
                    Double(voxel.positionAndConfidence.z)
                )
                samples.append(ReportPreviewPoint(position: p, color: voxelColor(voxel)))
                if samples.count >= targetCount { break }
            }
        } else if data.count == voxelBuffer.length {
            let total = data.count / voxelStride
            let sampleStride = max(1, total / targetCount)
            var recordIndex = 0
            for offset in stride(from: 0, to: data.count, by: voxelStride) {
                if isCancelled?() == true { return nil }
                defer { recordIndex += 1 }
                guard recordIndex % sampleStride == 0 else { continue }
                guard let voxel = readVoxel(at: offset), voxel.colorAndSampleCount.w > 0 else { continue }
                let p = transformedPoint(
                    Double(voxel.positionAndConfidence.x),
                    Double(voxel.positionAndConfidence.y),
                    Double(voxel.positionAndConfidence.z)
                )
                samples.append(ReportPreviewPoint(position: p, color: voxelColor(voxel)))
                if samples.count >= targetCount { break }
            }
        } else {
            return nil
        }

        return samples.isEmpty ? nil : samples
    }

    private func makeReportPreviewPanelImage(
        points: [ReportPreviewPoint],
        measurements: [ScanReportMeasurement],
        exportFormat: ExportFormat,
        captureMetadata: ScanCaptureMetadata?,
        style: ReportPreviewStyle,
    ) -> UIImage? {
        let crsContext = exportFormat == .laz ? makeApproximateCRSContext(captureMetadata: captureMetadata) : nil
        func transformedMeasurementVertex(_ vertex: ScanReportVertex) -> SIMD3<Double> {
            if let crsContext {
                let t = crsContext.transform(localX: vertex.x, localY: vertex.y, localZ: vertex.z)
                return SIMD3<Double>(t.x, t.y, t.z)
            }
            return SIMD3<Double>(vertex.x, vertex.y, vertex.z)
        }

        let measurementPaths3D: [[SIMD3<Double>]] = measurements
            .filter { $0.vertices.count >= 2 }
            .map { measurement in
                measurement.vertices.map(transformedMeasurementVertex(_:))
            }
        let measurementTypes = measurements.filter { $0.vertices.count >= 2 }.map(\.type)

        let minElevation = points.map(\.position.y).min() ?? 0
        let maxElevation = points.map(\.position.y).max() ?? 1
        let elevationSpan = max(maxElevation - minElevation, 0.001)

        func elevationColor(_ y: Double) -> UIColor {
            let t = max(0.0, min(1.0, (y - minElevation) / elevationSpan))
            let hue = CGFloat((0.67 - (0.67 * t)))
            return UIColor(hue: hue, saturation: 0.88, brightness: 0.90, alpha: 1.0)
        }

        func measurementColor(_ index: Int) -> UIColor {
            let hue = CGFloat((index % 8)) / 8.0
            return UIColor(hue: hue, saturation: 0.9, brightness: 0.85, alpha: 1.0)
        }

        let imageSize = CGSize(width: 960, height: 700)
        let renderer = UIGraphicsImageRenderer(size: imageSize)
        return renderer.image { context in
            let cg = context.cgContext
            cg.setFillColor(UIColor.white.cgColor)
            cg.fill(CGRect(origin: .zero, size: imageSize))

            let outerMargin: CGFloat = 20
            let panelGap: CGFloat = 10
            let titleBand: CGFloat = 20
            let content = CGRect(
                x: outerMargin,
                y: outerMargin,
                width: imageSize.width - outerMargin * 2,
                height: imageSize.height - outerMargin * 2
            )

            let panelW = (content.width - panelGap) / 2
            let panelH = (content.height - panelGap) / 2
            let views = ReportPreviewViewKind.allCases

            for (index, viewKind) in views.enumerated() {
                let row = index / 2
                let col = index % 2
                let origin = CGPoint(
                    x: content.minX + CGFloat(col) * (panelW + panelGap),
                    y: content.minY + CGFloat(row) * (panelH + panelGap)
                )
                let panelRect = CGRect(origin: origin, size: CGSize(width: panelW, height: panelH))
                let drawingRect = panelRect.insetBy(dx: 8, dy: 8 + titleBand)

                cg.setFillColor(UIColor(white: 0.985, alpha: 1.0).cgColor)
                cg.fill(panelRect)
                cg.setStrokeColor(UIColor(white: 0.82, alpha: 1.0).cgColor)
                cg.setLineWidth(0.9)
                cg.stroke(panelRect)

                NSString(string: viewKind.title).draw(
                    in: CGRect(x: panelRect.minX + 8, y: panelRect.minY + 5, width: panelRect.width - 16, height: 14),
                    withAttributes: [
                        .font: UIFont.systemFont(ofSize: 10, weight: .semibold),
                        .foregroundColor: UIColor.darkGray
                    ]
                )

                let projectedPoints = points.map { (p: viewKind.project($0.position), color: $0.color, elevation: $0.position.y) }
                var projectedMeasurementPaths: [[CGPoint]] = []
                projectedMeasurementPaths.reserveCapacity(measurementPaths3D.count)
                for path in measurementPaths3D {
                    projectedMeasurementPaths.append(path.map { viewKind.project($0) })
                }

                var minX = Double.greatestFiniteMagnitude
                var minY = Double.greatestFiniteMagnitude
                var maxX = -Double.greatestFiniteMagnitude
                var maxY = -Double.greatestFiniteMagnitude

                for point in projectedPoints {
                    minX = min(minX, point.p.x)
                    minY = min(minY, point.p.y)
                    maxX = max(maxX, point.p.x)
                    maxY = max(maxY, point.p.y)
                }
                for path in projectedMeasurementPaths {
                    for p in path {
                        minX = min(minX, p.x)
                        minY = min(minY, p.y)
                        maxX = max(maxX, p.x)
                        maxY = max(maxY, p.y)
                    }
                }
                guard minX.isFinite, minY.isFinite, maxX.isFinite, maxY.isFinite else { continue }
                let spanX = max(maxX - minX, 0.001)
                let spanY = max(maxY - minY, 0.001)

                func map(_ p: CGPoint) -> CGPoint {
                    let nx = (p.x - minX) / spanX
                    let ny = (p.y - minY) / spanY
                    return CGPoint(
                        x: drawingRect.minX + (CGFloat(nx) * drawingRect.width),
                        y: drawingRect.maxY - (CGFloat(ny) * drawingRect.height)
                    )
                }

                for point in projectedPoints {
                    let mapped = map(point.p)
                    let color: UIColor = {
                        switch style {
                        case .rgb:
                            return point.color.withAlphaComponent(0.42)
                        case .elevation:
                            return elevationColor(point.elevation).withAlphaComponent(0.45)
                        }
                    }()
                    cg.setFillColor(color.cgColor)
                    cg.fillEllipse(in: CGRect(x: mapped.x - 0.8, y: mapped.y - 0.8, width: 1.6, height: 1.6))
                }

                for (idx, path) in projectedMeasurementPaths.enumerated() where path.count >= 2 {
                    let color = measurementColor(idx)
                    cg.setStrokeColor(color.cgColor)
                    cg.setLineWidth(1.8)
                    cg.beginPath()
                    cg.move(to: map(path[0]))
                    for p in path.dropFirst() {
                        cg.addLine(to: map(p))
                    }
                    if measurementTypes[idx] == .closedArea {
                        cg.closePath()
                    }
                    cg.strokePath()

                    cg.setFillColor(color.cgColor)
                    for p in path {
                        let mp = map(p)
                        cg.fillEllipse(in: CGRect(x: mp.x - 1.9, y: mp.y - 1.9, width: 3.8, height: 3.8))
                    }
                }
            }

            let legendCount = min(measurementPaths3D.count, 8)
            if legendCount > 0 {
                let legendRect = CGRect(x: content.maxX - 178, y: content.minY + 6, width: 170, height: CGFloat(28 + (legendCount * 14)))
                cg.setFillColor(UIColor.white.withAlphaComponent(0.90).cgColor)
                cg.fill(legendRect)
                cg.setStrokeColor(UIColor(white: 0.76, alpha: 1.0).cgColor)
                cg.setLineWidth(0.8)
                cg.stroke(legendRect)

                NSString(string: "Legend").draw(
                    in: CGRect(x: legendRect.minX + 8, y: legendRect.minY + 5, width: legendRect.width - 16, height: 12),
                    withAttributes: [
                        .font: UIFont.systemFont(ofSize: 9, weight: .semibold),
                        .foregroundColor: UIColor.black
                    ]
                )

                for i in 0..<legendCount {
                    let y = legendRect.minY + 18 + CGFloat(i) * 14
                    let color = measurementColor(i)
                    cg.setFillColor(color.cgColor)
                    cg.fill(CGRect(x: legendRect.minX + 8, y: y + 2, width: 8, height: 8))
                    let typeLabel: String = {
                        switch measurementTypes[i] {
                        case .distance: return "Distance"
                        case .polyline: return "Polyline"
                        case .closedArea: return "Area"
                        case .crossSection: return "Cross"
                        case .elevationProfile: return "Elev"
                        }
                    }()
                    NSString(string: "M\(i + 1)  \(typeLabel)").draw(
                        in: CGRect(x: legendRect.minX + 22, y: y, width: legendRect.width - 28, height: 12),
                        withAttributes: [
                            .font: UIFont.systemFont(ofSize: 8.4, weight: .regular),
                            .foregroundColor: UIColor.black
                        ]
                    )
                }
            }

            if style == .elevation {
                let gradRect = CGRect(x: content.minX + 8, y: content.maxY - 20, width: 180, height: 10)
                let colors = stride(from: 0.0, through: 1.0, by: 0.05).map { elevationColor(minElevation + ($0 * elevationSpan)).cgColor } as CFArray
                if let gradient = CGGradient(colorsSpace: CGColorSpaceCreateDeviceRGB(), colors: colors, locations: nil) {
                    cg.saveGState()
                    cg.clip(to: gradRect)
                    cg.drawLinearGradient(gradient, start: CGPoint(x: gradRect.minX, y: gradRect.midY), end: CGPoint(x: gradRect.maxX, y: gradRect.midY), options: [])
                    cg.restoreGState()
                }
                cg.setStrokeColor(UIColor(white: 0.7, alpha: 1.0).cgColor)
                cg.stroke(gradRect)
                NSString(string: "Low").draw(
                    in: CGRect(x: gradRect.minX, y: gradRect.maxY + 1, width: 40, height: 10),
                    withAttributes: [.font: UIFont.systemFont(ofSize: 8), .foregroundColor: UIColor.darkGray]
                )
                NSString(string: "High").draw(
                    in: CGRect(x: gradRect.maxX - 34, y: gradRect.maxY + 1, width: 34, height: 10),
                    withAttributes: [.font: UIFont.systemFont(ofSize: 8), .foregroundColor: UIColor.darkGray]
                )
            }
        }
    }

    private struct ProfileSamplePoint {
        let distance: Double
        let elevation: Double
    }

    private struct MeasurementSectionPoint {
        let x: Double
        let y: Double
        let color: UIColor
    }

    private struct MeasurementPointCloudAnalysis {
        let profileSamples: [ProfileSamplePoint]
        let sectionScatter: [MeasurementSectionPoint]
        let sourcePointCount: Int
        let corridorPointCount: Int
        let profileBinWidthMeters: Double?
        let stationWindowMeters: Double?
        let profileLength: Double?
        let minElevation: Double?
        let maxElevation: Double?
        let relief: Double?
        let totalRise: Double?
        let totalFall: Double?
        let averageSlopePercent: Double?
        let maxSlopePercent: Double?
    }

    private func buildMeasurementAnalyses(
        measurements: [ScanReportMeasurement],
        cloudPoints: [ReportPreviewPoint],
        exportFormat: ExportFormat,
        captureMetadata: ScanCaptureMetadata?
    ) -> [String: MeasurementPointCloudAnalysis] {
        let crsContext = exportFormat == .laz ? makeApproximateCRSContext(captureMetadata: captureMetadata) : nil
        let useProjectedAxes = (crsContext != nil)

        func transformedVertex(_ vertex: ScanReportVertex) -> SIMD3<Double> {
            if let crsContext {
                let t = crsContext.transform(localX: vertex.x, localY: vertex.y, localZ: vertex.z)
                return SIMD3<Double>(t.x, t.y, t.z)
            }
            return SIMD3<Double>(vertex.x, vertex.y, vertex.z)
        }

        func horizontal(_ p: SIMD3<Double>) -> SIMD2<Double> {
            useProjectedAxes ? SIMD2<Double>(p.x, p.y) : SIMD2<Double>(p.x, p.z)
        }

        func elevation(_ p: SIMD3<Double>) -> Double {
            useProjectedAxes ? p.z : p.y
        }

        struct SegmentInfo {
            let a: SIMD2<Double>
            let v: SIMD2<Double>
            let len: Double
            let invLen2: Double
            let startDistance: Double
        }

        struct CorridorPoint {
            let alongDistance: Double
            let lateralDistanceSigned: Double
            let elevation: Double
            let color: UIColor
        }

        func median(_ values: [Double]) -> Double {
            let sorted = values.sorted()
            if sorted.isEmpty { return 0 }
            if sorted.count % 2 == 1 { return sorted[sorted.count / 2] }
            return (sorted[(sorted.count / 2) - 1] + sorted[sorted.count / 2]) * 0.5
        }

        func percentile(_ values: [Double], fraction: Double) -> Double {
            let sorted = values.sorted()
            guard !sorted.isEmpty else { return 0 }
            let p = max(0.0, min(1.0, fraction))
            let idx = Int(round(p * Double(sorted.count - 1)))
            return sorted[max(0, min(sorted.count - 1, idx))]
        }

        func fillMissing(_ values: inout [Double?]) {
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
                            let t = Double(g) / Double(gap)
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

        func smoothSeries(_ values: [Double], radius: Int) -> [Double] {
            guard !values.isEmpty, radius > 0 else { return values }
            var result = values
            for idx in values.indices {
                var weightedSum = 0.0
                var weightTotal = 0.0
                let start = max(values.startIndex, idx - radius)
                let end = min(values.endIndex - 1, idx + radius)
                for sampleIdx in start...end {
                    let offset = abs(sampleIdx - idx)
                    let weight = Double(radius - offset + 1)
                    weightedSum += values[sampleIdx] * weight
                    weightTotal += weight
                }
                result[idx] = weightTotal > 0 ? (weightedSum / weightTotal) : values[idx]
            }
            return result
        }

        func fallbackProfile(vertices: [SIMD3<Double>]) -> [ProfileSamplePoint] {
            guard vertices.count >= 2 else { return [] }
            var result: [ProfileSamplePoint] = [ProfileSamplePoint(distance: 0, elevation: elevation(vertices[0]))]
            var total: Double = 0
            for idx in 1..<vertices.count {
                let hA = horizontal(vertices[idx - 1])
                let hB = horizontal(vertices[idx])
                total += simd_length(hB - hA)
                result.append(ProfileSamplePoint(distance: total, elevation: elevation(vertices[idx])))
            }
            return result
        }

        func stats(from profile: [ProfileSamplePoint]) -> (
            length: Double?,
            minElevation: Double?,
            maxElevation: Double?,
            relief: Double?,
            rise: Double?,
            fall: Double?,
            avgSlope: Double?,
            maxSlope: Double?
        ) {
            guard profile.count >= 2 else {
                return (nil, nil, nil, nil, nil, nil, nil, nil)
            }
            let elevations = profile.map(\.elevation)
            guard let minE = elevations.min(), let maxE = elevations.max() else {
                return (nil, nil, nil, nil, nil, nil, nil, nil)
            }

            var rise = 0.0
            var fall = 0.0
            var maxSlope = 0.0
            var totalDistance = 0.0
            var totalAbsDelta = 0.0
            for idx in 1..<profile.count {
                let d = profile[idx].distance - profile[idx - 1].distance
                guard d > 1e-6 else { continue }
                let dz = profile[idx].elevation - profile[idx - 1].elevation
                if dz >= 0 {
                    rise += dz
                } else {
                    fall += abs(dz)
                }
                totalDistance += d
                totalAbsDelta += abs(dz)
                maxSlope = max(maxSlope, abs(dz / d) * 100.0)
            }
            let avgSlope = totalDistance > 1e-6 ? (totalAbsDelta / totalDistance * 100.0) : 0.0
            let length = profile.last!.distance - profile.first!.distance
            return (length, minE, maxE, max(0, maxE - minE), rise, fall, avgSlope, maxSlope)
        }

        func analyze(measurement: ScanReportMeasurement, path: [SIMD3<Double>]) -> MeasurementPointCloudAnalysis? {
            guard path.count >= 2 else { return nil }

            var segments: [SegmentInfo] = []
            segments.reserveCapacity(path.count - 1)
            var cumulativeDistance = 0.0
            for idx in 1..<path.count {
                let a = horizontal(path[idx - 1])
                let b = horizontal(path[idx])
                let v = b - a
                let len = simd_length(v)
                guard len > 1e-6 else { continue }
                segments.append(SegmentInfo(a: a, v: v, len: len, invLen2: 1.0 / simd_length_squared(v), startDistance: cumulativeDistance))
                cumulativeDistance += len
            }
            guard !segments.isEmpty, cumulativeDistance > 1e-6 else { return nil }

            let crossSwath = max(0.04, measurement.swathWidthMeters ?? 0.12)
            let profileSwath = measurement.type == .crossSection ? crossSwath : max(crossSwath, 0.12)
            let profileHalf = profileSwath * 0.5
            let sectionHalfWindow = max(0.06, min(1.2, crossSwath * 0.9))
            let sectionMid = cumulativeDistance * 0.5

            var corridorPoints: [CorridorPoint] = []
            corridorPoints.reserveCapacity(10_000)

            for sample in cloudPoints {
                let point = sample.position
                let hPoint = horizontal(point)
                let pointElevation = elevation(point)
                var bestDistance = Double.greatestFiniteMagnitude
                var bestAlong = 0.0
                var bestSigned = 0.0

                for segment in segments {
                    let w = hPoint - segment.a
                    let rawT = simd_dot(w, segment.v) * segment.invLen2
                    let t = max(0.0, min(1.0, rawT))
                    let projected = segment.a + (segment.v * t)
                    let delta = hPoint - projected
                    let lateralDistance = simd_length(delta)
                    if lateralDistance < bestDistance {
                        bestDistance = lateralDistance
                        bestAlong = segment.startDistance + (t * segment.len)
                        let cross = (segment.v.x * w.y) - (segment.v.y * w.x)
                        let sign = cross >= 0 ? 1.0 : -1.0
                        bestSigned = lateralDistance * sign
                    }
                }

                if bestDistance <= profileHalf {
                    corridorPoints.append(CorridorPoint(
                        alongDistance: bestAlong,
                        lateralDistanceSigned: bestSigned,
                        elevation: pointElevation,
                        color: sample.color
                    ))
                }
            }

            var profile: [ProfileSamplePoint] = []
            var profileBinWidth: Double?
            if !corridorPoints.isEmpty {
                let targetBinWidth = max(0.01, min(0.05, profileSwath * 0.35))
                let desiredSamples = max(32, min(420, Int(ceil(cumulativeDistance / targetBinWidth)) + 1))
                let binCount = max(2, desiredSamples)
                let binWidth = cumulativeDistance / Double(binCount - 1)
                profileBinWidth = binWidth
                var bins = Array(repeating: [Double](), count: binCount)
                bins.withUnsafeMutableBufferPointer { binsBuffer in
                    for sample in corridorPoints {
                        let idx = max(0, min(binCount - 1, Int(floor(sample.alongDistance / max(binWidth, 1e-6)))))
                        binsBuffer[idx].append(sample.elevation)
                    }
                }
                var binElevations = Array<Double?>(repeating: nil, count: binCount)
                for idx in 0..<binCount where !bins[idx].isEmpty {
                    if measurement.type == .elevationProfile {
                        binElevations[idx] = percentile(bins[idx], fraction: 0.35)
                    } else {
                        binElevations[idx] = median(bins[idx])
                    }
                }
                fillMissing(&binElevations)
                if binElevations.contains(where: { $0 != nil }) {
                    var values = binElevations.map { $0 ?? 0 }
                    if measurement.type == .elevationProfile {
                        values = smoothSeries(values, radius: 2)
                        values = smoothSeries(values, radius: 2)
                    }
                    for idx in 0..<binCount {
                        profile.append(ProfileSamplePoint(distance: Double(idx) * binWidth, elevation: values[idx]))
                    }
                }
            }
            if profile.count < 2 {
                profile = fallbackProfile(vertices: path)
                if profile.count >= 2 {
                    profileBinWidth = max(1e-6, cumulativeDistance / Double(profile.count - 1))
                }
            }

            let sectionScatter: [MeasurementSectionPoint] = measurement.type == .crossSection
                ? corridorPoints.filter {
                    abs($0.alongDistance - sectionMid) <= sectionHalfWindow && abs($0.lateralDistanceSigned) <= (crossSwath * 0.5)
                }.map {
                    MeasurementSectionPoint(x: $0.lateralDistanceSigned, y: $0.elevation, color: $0.color)
                }
                : []

            let s = stats(from: profile)
            return MeasurementPointCloudAnalysis(
                profileSamples: profile,
                sectionScatter: sectionScatter,
                sourcePointCount: cloudPoints.count,
                corridorPointCount: corridorPoints.count,
                profileBinWidthMeters: profileBinWidth,
                stationWindowMeters: sectionHalfWindow * 2.0,
                profileLength: s.length,
                minElevation: s.minElevation,
                maxElevation: s.maxElevation,
                relief: s.relief,
                totalRise: s.rise,
                totalFall: s.fall,
                averageSlopePercent: s.avgSlope,
                maxSlopePercent: s.maxSlope
            )
        }

        var result: [String: MeasurementPointCloudAnalysis] = [:]
        for measurement in measurements where measurement.type == .crossSection || measurement.type == .elevationProfile {
            let transformedPath = measurement.vertices.map(transformedVertex(_:))
            if let analysis = analyze(measurement: measurement, path: transformedPath) {
                result[measurement.id] = analysis
            }
        }
        return result
    }

    private func makeMeasurementGraphImage(
        measurement: ScanReportMeasurement,
        analysis: MeasurementPointCloudAnalysis?,
        title: String,
        lineColor: UIColor
    ) -> UIImage? {
        guard measurement.vertices.count >= 2 else { return nil }

        let size = CGSize(width: 900, height: 260)
        let renderer = UIGraphicsImageRenderer(size: size)
        return renderer.image { context in
            let cg = context.cgContext
            let rect = CGRect(origin: .zero, size: size)
            cg.setFillColor(UIColor.white.cgColor)
            cg.fill(rect)
            cg.setStrokeColor(UIColor(white: 0.84, alpha: 1).cgColor)
            cg.setLineWidth(0.8)
            cg.stroke(rect.insetBy(dx: 0.5, dy: 0.5))

            let chartRect = CGRect(x: 54, y: 28, width: size.width - 78, height: size.height - 66)

            cg.setStrokeColor(UIColor(white: 0.9, alpha: 1.0).cgColor)
            cg.setLineWidth(0.8)
            for i in 0...4 {
                let t = CGFloat(i) / 4.0
                let y = chartRect.minY + t * chartRect.height
                cg.move(to: CGPoint(x: chartRect.minX, y: y))
                cg.addLine(to: CGPoint(x: chartRect.maxX, y: y))
                cg.strokePath()
            }

            NSString(string: title).draw(
                in: CGRect(x: 10, y: 8, width: size.width - 20, height: 16),
                withAttributes: [
                    .font: UIFont.systemFont(ofSize: 11, weight: .semibold),
                    .foregroundColor: UIColor.black
                ]
            )

            if measurement.type == .crossSection, let analysis, analysis.sectionScatter.count >= 4 {
                let xs = analysis.sectionScatter.map(\.x)
                let ys = analysis.sectionScatter.map(\.y)
                guard let minX = xs.min(), let maxX = xs.max(), let minY = ys.min(), let maxY = ys.max() else { return }
                let spanX = max(maxX - minX, 0.001)
                let spanY = max(maxY - minY, 0.001)
                func map(_ point: MeasurementSectionPoint) -> CGPoint {
                    let nx = (point.x - minX) / spanX
                    let ny = (point.y - minY) / spanY
                    return CGPoint(
                        x: chartRect.minX + CGFloat(nx) * chartRect.width,
                        y: chartRect.maxY - CGFloat(ny) * chartRect.height
                    )
                }
                for point in analysis.sectionScatter {
                    let p = map(point)
                    cg.setFillColor(point.color.withAlphaComponent(0.82).cgColor)
                    cg.fillEllipse(in: CGRect(x: p.x - 1.5, y: p.y - 1.5, width: 3.0, height: 3.0))
                }
                let axisAttrs: [NSAttributedString.Key: Any] = [
                    .font: UIFont.systemFont(ofSize: 8, weight: .regular),
                    .foregroundColor: UIColor.darkGray
                ]
                NSString(string: String(format: "%.2f m", minY)).draw(in: CGRect(x: 4, y: chartRect.maxY - 6, width: 46, height: 12), withAttributes: axisAttrs)
                NSString(string: String(format: "%.2f m", maxY)).draw(in: CGRect(x: 4, y: chartRect.minY - 6, width: 46, height: 12), withAttributes: axisAttrs)
                NSString(string: String(format: "%.2f", minX)).draw(in: CGRect(x: chartRect.minX - 2, y: chartRect.maxY + 2, width: 40, height: 12), withAttributes: axisAttrs)
                NSString(string: String(format: "%.2f", maxX)).draw(in: CGRect(x: chartRect.maxX - 40, y: chartRect.maxY + 2, width: 40, height: 12), withAttributes: axisAttrs)
                return
            }

            var profile = analysis?.profileSamples ?? []
            if profile.count < 2 {
                var distances: [Double] = [0]
                distances.reserveCapacity(measurement.vertices.count)
                var totalDistance = 0.0
                for idx in 1..<measurement.vertices.count {
                    let a = measurement.vertices[idx - 1]
                    let b = measurement.vertices[idx]
                    let dx = b.x - a.x
                    let dy = b.y - a.y
                    let dz = b.z - a.z
                    totalDistance += sqrt(dx * dx + dy * dy + dz * dz)
                    distances.append(totalDistance)
                }
                profile = measurement.vertices.enumerated().map {
                    ProfileSamplePoint(distance: distances[$0.offset], elevation: $0.element.y)
                }
            }
            guard profile.count >= 2 else { return }

            let minElevation = profile.map(\.elevation).min() ?? 0
            let maxElevation = profile.map(\.elevation).max() ?? 1
            let spanY = max(maxElevation - minElevation, 0.001)
            let totalDistance = max(profile.last!.distance - profile.first!.distance, 0.001)

            func map(_ sample: ProfileSamplePoint) -> CGPoint {
                let nx = (sample.distance - profile.first!.distance) / totalDistance
                let ny = (sample.elevation - minElevation) / spanY
                return CGPoint(
                    x: chartRect.minX + CGFloat(nx) * chartRect.width,
                    y: chartRect.maxY - CGFloat(ny) * chartRect.height
                )
            }

            let fillPath = UIBezierPath()
            let linePath = UIBezierPath()
            let first = map(profile[0])
            fillPath.move(to: CGPoint(x: first.x, y: chartRect.maxY))
            fillPath.addLine(to: first)
            linePath.move(to: first)
            for sample in profile.dropFirst() {
                let p = map(sample)
                fillPath.addLine(to: p)
                linePath.addLine(to: p)
            }
            if let last = linePath.currentPoint as CGPoint? {
                fillPath.addLine(to: CGPoint(x: last.x, y: chartRect.maxY))
                fillPath.close()
            }

            lineColor.withAlphaComponent(0.15).setFill()
            fillPath.fill()
            lineColor.setStroke()
            linePath.lineWidth = 2.0
            linePath.stroke()

            let axisAttrs: [NSAttributedString.Key: Any] = [
                .font: UIFont.systemFont(ofSize: 8, weight: .regular),
                .foregroundColor: UIColor.darkGray
            ]
            NSString(string: String(format: "%.2f m", minElevation)).draw(in: CGRect(x: 4, y: chartRect.maxY - 6, width: 46, height: 12), withAttributes: axisAttrs)
            NSString(string: String(format: "%.2f m", maxElevation)).draw(in: CGRect(x: 4, y: chartRect.minY - 6, width: 46, height: 12), withAttributes: axisAttrs)
            NSString(string: "0m").draw(in: CGRect(x: chartRect.minX - 2, y: chartRect.maxY + 2, width: 30, height: 12), withAttributes: axisAttrs)
            NSString(string: String(format: "%.2fm", totalDistance)).draw(in: CGRect(x: chartRect.maxX - 40, y: chartRect.maxY + 2, width: 40, height: 12), withAttributes: axisAttrs)
        }
    }

    private struct ExportCRSContext {
        let epsg: Int
        let crsName: String
        let wkt: String
        let originLatitude: Double
        let originLongitude: Double
        let originAltitude: Double
        let originEasting: Double
        let originNorthing: Double
        let headingDegrees: Double?
        let note: String

        func transform(localX: Double, localY: Double, localZ: Double) -> (x: Double, y: Double, z: Double) {
            let eastLocal = localX
            let northLocal = -localZ

            let east: Double
            let north: Double
            if let headingDegrees {
                let h = headingDegrees * .pi / 180.0
                east = (eastLocal * cos(h)) + (northLocal * sin(h))
                north = (-eastLocal * sin(h)) + (northLocal * cos(h))
            } else {
                east = eastLocal
                north = northLocal
            }

            return (
                originEasting + east,
                originNorthing + north,
                originAltitude + localY
            )
        }
    }

    private func makeApproximateCRSContext(captureMetadata: ScanCaptureMetadata?) -> ExportCRSContext? {
        guard let location = captureMetadata?.location else { return nil }
        let lat = location.latitude
        let lon = location.longitude
        guard lat.isFinite, lon.isFinite else { return nil }
        guard abs(lat) <= 84.0, abs(lon) <= 180.0 else { return nil }

        let zone = max(1, min(60, Int(floor((lon + 180.0) / 6.0)) + 1))
        let isNorth = lat >= 0
        let epsg = (isNorth ? 32600 : 32700) + zone
        let falseNorthing = isNorth ? 0.0 : 10_000_000.0
        let hemisphereSuffix = isNorth ? "N" : "S"
        let centralMeridian = Double(zone * 6 - 183)
        let crsName = "WGS 84 / UTM zone \(zone)\(hemisphereSuffix)"
        let wkt = """
        PROJCS["\(crsName)",GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",\(centralMeridian)],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",\(falseNorthing)],UNIT["metre",1],AXIS["Easting",EAST],AXIS["Northing",NORTH],AUTHORITY["EPSG","\(epsg)"]]
        """

        guard let utmOrigin = latLonToUTM(latitude: lat, longitude: lon, zone: zone, isNorthernHemisphere: isNorth) else {
            return nil
        }

        let heading = location.headingDegrees.flatMap { value -> Double? in
            guard value.isFinite else { return nil }
            let normalized = value.truncatingRemainder(dividingBy: 360.0)
            return normalized >= 0 ? normalized : (normalized + 360.0)
        }
        let note = heading != nil ?
            "Approximate georeferencing anchored by device GPS and heading." :
            "Approximate georeferencing anchored by device GPS; orientation uses local AR frame."

        return ExportCRSContext(
            epsg: epsg,
            crsName: crsName,
            wkt: wkt,
            originLatitude: lat,
            originLongitude: lon,
            originAltitude: location.altitude,
            originEasting: utmOrigin.easting,
            originNorthing: utmOrigin.northing,
            headingDegrees: heading,
            note: note
        )
    }

    private func latLonToUTM(
        latitude: Double,
        longitude: Double,
        zone: Int,
        isNorthernHemisphere: Bool
    ) -> (easting: Double, northing: Double)? {
        let a = 6_378_137.0
        let f = 1.0 / 298.257_223_563
        let k0 = 0.9996
        let e2 = f * (2 - f)
        let ePrime2 = e2 / (1 - e2)

        let latRad = latitude * .pi / 180.0
        let lonRad = longitude * .pi / 180.0
        let lonOrigin = Double(zone * 6 - 183) * .pi / 180.0

        let sinLat = sin(latRad)
        let cosLat = cos(latRad)
        let tanLat = tan(latRad)

        let n = a / sqrt(1 - e2 * sinLat * sinLat)
        let t = tanLat * tanLat
        let c = ePrime2 * cosLat * cosLat
        let aTerm = cosLat * (lonRad - lonOrigin)

        let m = a * (
            (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * pow(e2, 3) / 256) * latRad
            - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * pow(e2, 3) / 1024) * sin(2 * latRad)
            + (15 * e2 * e2 / 256 + 45 * pow(e2, 3) / 1024) * sin(4 * latRad)
            - (35 * pow(e2, 3) / 3072) * sin(6 * latRad)
        )

        let easting = k0 * n * (
            aTerm
            + (1 - t + c) * pow(aTerm, 3) / 6
            + (5 - 18 * t + t * t + 72 * c - 58 * ePrime2) * pow(aTerm, 5) / 120
        ) + 500_000.0

        var northing = k0 * (
            m
            + n * tanLat * (
                aTerm * aTerm / 2
                + (5 - t + 9 * c + 4 * c * c) * pow(aTerm, 4) / 24
                + (61 - 58 * t + t * t + 600 * c - 330 * ePrime2) * pow(aTerm, 6) / 720
            )
        )

        if !isNorthernHemisphere {
            northing += 10_000_000.0
        }

        guard easting.isFinite, northing.isFinite else { return nil }
        return (easting, northing)
    }

    private func writeLAZ(
        to url: URL,
        fromSnapshot data: Data,
        pointCount: Int,
        captureMetadata: ScanCaptureMetadata?,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> LAZWriteResult? {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride
        let progressUpdateStride = 8_192

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        func clampedColor16(_ value: Float) -> UInt16 {
            let scaled8 = value <= 1.0 ? value * 255.0 : value
            let clamped8 = max(0, min(255, Int(scaled8.rounded())))
            return UInt16(clamped8 * 257)
        }

        func intensityFromConfidence(_ value: Float) -> UInt16 {
            let normalized = value <= 1.0 ? value : min(1.0, value / 255.0)
            let scaled = max(0.0, min(1.0, Double(normalized))) * 65535.0
            return UInt16(scaled.rounded())
        }

        let isCompact = data.count >= headerSize &&
            readUInt32(at: 0) == 0x50435331 &&
            readUInt32(at: 1 * MemoryLayout<UInt32>.size) == 1
        let crsContext = makeApproximateCRSContext(captureMetadata: captureMetadata)

        func transformedPoint(_ voxel: Voxel) -> (x: Double, y: Double, z: Double) {
            let px = Double(voxel.positionAndConfidence.x)
            let py = Double(voxel.positionAndConfidence.y)
            let pz = Double(voxel.positionAndConfidence.z)
            if let crsContext {
                return crsContext.transform(localX: px, localY: py, localZ: pz)
            }
            return (px, py, pz)
        }

        var compactRecordCount = 0
        var vertexCount = 0
        var minX = Double.greatestFiniteMagnitude
        var minY = Double.greatestFiniteMagnitude
        var minZ = Double.greatestFiniteMagnitude
        var maxX = -Double.greatestFiniteMagnitude
        var maxY = -Double.greatestFiniteMagnitude
        var maxZ = -Double.greatestFiniteMagnitude

        if isCompact, let storedCount = readUInt32(at: 2 * MemoryLayout<UInt32>.size) {
            compactRecordCount = min(Int(storedCount), max(0, (data.count - headerSize) / recordSize))
            var offset = headerSize
            for i in 0..<compactRecordCount {
                if isCancelled?() == true { return nil }
                guard offset + recordSize <= data.count else { break }
                let idx = readUInt32(at: offset) ?? UInt32.max
                offset += MemoryLayout<UInt32>.size
                guard idx < UInt32(maxVoxels), let voxel = readVoxel(at: offset) else {
                    offset += voxelStride
                    continue
                }
                offset += voxelStride
                let projected = transformedPoint(voxel)
                minX = min(minX, projected.x)
                minY = min(minY, projected.y)
                minZ = min(minZ, projected.z)
                maxX = max(maxX, projected.x)
                maxY = max(maxY, projected.y)
                maxZ = max(maxZ, projected.z)
                vertexCount += 1
                if i % progressUpdateStride == 0 {
                    let fraction = compactRecordCount > 0 ? Double(i) / Double(compactRecordCount) : 1.0
                    progress?(0.15 * fraction, "Scanning points...")
                }
            }
        } else if data.count == voxelBuffer.length {
            let totalRecords = data.count / voxelStride
            var processedRecords = 0
            for i in stride(from: 0, to: data.count, by: voxelStride) {
                if isCancelled?() == true { return nil }
                guard let voxel = readVoxel(at: i) else { break }
                if voxel.colorAndSampleCount.w <= 0 {
                    processedRecords += 1
                    continue
                }
                let projected = transformedPoint(voxel)
                minX = min(minX, projected.x)
                minY = min(minY, projected.y)
                minZ = min(minZ, projected.z)
                maxX = max(maxX, projected.x)
                maxY = max(maxY, projected.y)
                maxZ = max(maxZ, projected.z)
                vertexCount += 1
                processedRecords += 1
                if processedRecords % progressUpdateStride == 0 {
                    let fraction = totalRecords > 0 ? Double(processedRecords) / Double(totalRecords) : 1.0
                    progress?(0.15 * fraction, "Scanning points...")
                }
            }
        } else {
            return nil
        }

        if vertexCount == 0 {
            vertexCount = max(0, pointCount)
        }
        guard vertexCount > 0 else { return nil }

        if !minX.isFinite || !minY.isFinite || !minZ.isFinite ||
            !maxX.isFinite || !maxY.isFinite || !maxZ.isFinite {
            return nil
        }

        let scale = [0.001, 0.001, 0.001]
        let offset = [minX, minY, minZ]
        let wkt = crsContext?.wkt ?? ""

        let writer = url.path.withCString { pathCString in
            wkt.withCString { wktCString in
                pp_laz_writer_create(
                    pathCString,
                    scale[0], scale[1], scale[2],
                    offset[0], offset[1], offset[2],
                    50_000,
                    wktCString
                )
            }
        }
        guard let writer else { return nil }
        defer {
            pp_laz_writer_destroy(writer)
        }

        progress?(0.18, "Writing LAZ points...")
        var pointsWritten = 0

        if isCompact {
            var offsetBytes = headerSize
            for _ in 0..<compactRecordCount {
                if isCancelled?() == true { return nil }
                guard offsetBytes + recordSize <= data.count else { break }
                let idx = readUInt32(at: offsetBytes) ?? UInt32.max
                offsetBytes += MemoryLayout<UInt32>.size
                guard idx < UInt32(maxVoxels), let voxel = readVoxel(at: offsetBytes) else {
                    offsetBytes += voxelStride
                    continue
                }
                offsetBytes += voxelStride
                let projected = transformedPoint(voxel)
                let ok = pp_laz_writer_write_point(
                    writer,
                    projected.x,
                    projected.y,
                    projected.z,
                    clampedColor16(voxel.colorAndSampleCount.x),
                    clampedColor16(voxel.colorAndSampleCount.y),
                    clampedColor16(voxel.colorAndSampleCount.z),
                    intensityFromConfidence(voxel.positionAndConfidence.w),
                    1
                )
                guard ok else { return nil }
                pointsWritten += 1
                if pointsWritten % progressUpdateStride == 0 {
                    let fraction = vertexCount > 0 ? Double(pointsWritten) / Double(vertexCount) : 1.0
                    progress?(0.18 + (0.80 * min(1.0, fraction)), "Writing LAZ points...")
                }
            }
        } else {
            for i in stride(from: 0, to: data.count, by: voxelStride) {
                if isCancelled?() == true { return nil }
                guard let voxel = readVoxel(at: i) else { break }
                if voxel.colorAndSampleCount.w <= 0 {
                    continue
                }
                let projected = transformedPoint(voxel)
                let ok = pp_laz_writer_write_point(
                    writer,
                    projected.x,
                    projected.y,
                    projected.z,
                    clampedColor16(voxel.colorAndSampleCount.x),
                    clampedColor16(voxel.colorAndSampleCount.y),
                    clampedColor16(voxel.colorAndSampleCount.z),
                    intensityFromConfidence(voxel.positionAndConfidence.w),
                    1
                )
                guard ok else { return nil }
                pointsWritten += 1
                if pointsWritten % progressUpdateStride == 0 {
                    let fraction = vertexCount > 0 ? Double(pointsWritten) / Double(vertexCount) : 1.0
                    progress?(0.18 + (0.80 * min(1.0, fraction)), "Writing LAZ points...")
                }
            }
        }

        guard pointsWritten > 0 else { return nil }
        guard pp_laz_writer_close(writer) else { return nil }
        progress?(0.98, "Finishing LAZ file...")

        return LAZWriteResult(
            pointCount: pointsWritten,
            scale: scale,
            offset: offset,
            boundsMin: [minX, minY, minZ],
            boundsMax: [maxX, maxY, maxZ],
            epsg: crsContext?.epsg,
            crsName: crsContext?.crsName,
            georefNote: crsContext?.note,
            headingDegrees: crsContext?.headingDegrees
        )
    }

    private func writeLAZSidecar(
        to sidecarURL: URL,
        lazURL: URL,
        session: ScanSession,
        captureMetadata: ScanCaptureMetadata?,
        pointCount: Int,
        scale: [Double],
        offset: [Double],
        boundsMin: [Double],
        boundsMax: [Double],
        epsg: Int?,
        crsName: String?,
        georefNote: String?,
        headingDegrees: Double?
    ) -> Bool {
        let georef = ScanExportSidecar.Georeference(
            mode: epsg != nil ? "device_gps_approx_projected" : "local_only",
            epsg: epsg,
            crsName: crsName,
            latitude: captureMetadata?.location?.latitude,
            longitude: captureMetadata?.location?.longitude,
            altitude: captureMetadata?.location?.altitude,
            horizontalAccuracy: captureMetadata?.location?.horizontalAccuracy,
            verticalAccuracy: captureMetadata?.location?.verticalAccuracy,
            headingDegrees: headingDegrees ?? captureMetadata?.location?.headingDegrees,
            note: georefNote,
            timestamp: captureMetadata?.location?.timestamp
        )

        let sidecar = ScanExportSidecar(
            schemaVersion: 1,
            exportedAt: Date(),
            scanID: session.id,
            scanName: session.name,
            sessionCreatedAt: session.createdAt,
            sessionUpdatedAt: session.updatedAt,
            files: .init(laz: lazURL.lastPathComponent, metadata: sidecarURL.lastPathComponent),
            coordinateFrame: .init(
                name: crsName ?? "ARKit Local Frame",
                units: "meters",
                worldOriginTransform: captureMetadata?.worldOriginTransform ?? [
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1,
                ]
            ),
            pointSchema: .init(
                format: "LAS 1.4 / Point Format 2 (LAZ compressed)",
                coordinateType: "float64 -> int32 quantized",
                colorType: "uint16 RGB",
                intensityType: "uint16",
                classificationType: "uint8",
                scale: scale,
                offset: offset,
                pointCount: pointCount
            ),
            georeference: georef,
            capture: captureMetadata,
            stats: .init(pointCount: pointCount, boundsMin: boundsMin, boundsMax: boundsMax)
        )

        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        encoder.dateEncodingStrategy = .iso8601
        guard let jsonData = try? encoder.encode(sidecar) else { return false }
        do {
            try jsonData.write(to: sidecarURL, options: .atomic)
            return true
        } catch {
            return false
        }
    }

    private func writePLY(
        to url: URL,
        fromSnapshot data: Data,
        pointCount: Int,
        format: PLYExportFormat,
        progress: ((Double, String) -> Void)? = nil,
        isCancelled: (() -> Bool)? = nil
    ) -> Bool {
        let voxelStride = MemoryLayout<Voxel>.stride
        let headerSize = MemoryLayout<UInt32>.size * 3
        let recordSize = MemoryLayout<UInt32>.size + voxelStride
        let progressUpdateStride = 8_192

        func readUInt32(at offset: Int) -> UInt32? {
            guard offset + MemoryLayout<UInt32>.size <= data.count else { return nil }
            var value: UInt32 = 0
            _ = withUnsafeMutableBytes(of: &value) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + MemoryLayout<UInt32>.size))
            }
            return value
        }

        func readVoxel(at offset: Int) -> Voxel? {
            guard offset + voxelStride <= data.count else { return nil }
            var voxel = Voxel(positionAndConfidence: .zero, colorAndSampleCount: .zero)
            _ = withUnsafeMutableBytes(of: &voxel) { dst in
                data.copyBytes(to: dst, from: offset..<(offset + voxelStride))
            }
            return voxel
        }

        func writeLine(_ file: FileHandle, _ line: String) throws {
            if let data = line.data(using: .utf8) {
                try file.write(contentsOf: data)
            }
        }

        func writeFloatLE(_ file: FileHandle, _ value: Float) throws {
            var bits = value.bitPattern.littleEndian
            try withUnsafeBytes(of: &bits) { bytes in
                try file.write(contentsOf: bytes)
            }
        }

        func clampedColor(_ value: Float) -> Int {
            let scaled = value <= 1.0 ? value * 255.0 : value
            return max(0, min(255, Int(scaled.rounded())))
        }

        var vertexCount = 0
        let isCompact = data.count >= headerSize && readUInt32(at: 0) == 0x50435331 && readUInt32(at: 1 * MemoryLayout<UInt32>.size) == 1
        var compactRecordCount = 0

        if isCompact, let storedCount = readUInt32(at: 2 * MemoryLayout<UInt32>.size) {
            compactRecordCount = min(Int(storedCount), max(0, (data.count - headerSize) / recordSize))
            var offset = headerSize
            for i in 0..<compactRecordCount {
                if isCancelled?() == true { return false }
                guard offset + recordSize <= data.count else { break }
                let idx = readUInt32(at: offset) ?? UInt32.max
                offset += MemoryLayout<UInt32>.size
                if idx < UInt32(maxVoxels) {
                    vertexCount += 1
                }
                offset += voxelStride
                if i % progressUpdateStride == 0 {
                    let fraction = compactRecordCount > 0 ? Double(i) / Double(compactRecordCount) : 1.0
                    progress?(0.15 * fraction, "Counting vertices...")
                }
            }
        } else if data.count == voxelBuffer.length {
            let totalRecords = data.count / voxelStride
            var processedRecords = 0
            for i in stride(from: 0, to: data.count, by: voxelStride) {
                if isCancelled?() == true { return false }
                guard let v = readVoxel(at: i) else { break }
                if v.colorAndSampleCount.w > 0 {
                    vertexCount += 1
                }
                processedRecords += 1
                if processedRecords % progressUpdateStride == 0 {
                    let fraction = totalRecords > 0 ? Double(processedRecords) / Double(totalRecords) : 1.0
                    progress?(0.15 * fraction, "Counting vertices...")
                }
            }
        } else {
            return false
        }

        if vertexCount == 0 {
            vertexCount = max(0, pointCount)
        }
        progress?(0.15, "Writing PLY header...")

        FileManager.default.createFile(atPath: url.path, contents: nil)
        guard let file = try? FileHandle(forWritingTo: url) else { return false }
        defer {
            try? file.close()
        }

        do {
            try writeLine(file, "ply\n")
            if format == .binaryLittleEndian {
                try writeLine(file, "format binary_little_endian 1.0\n")
            } else {
                try writeLine(file, "format ascii 1.0\n")
            }
            try writeLine(file, "element vertex \(vertexCount)\n")
            try writeLine(file, "property float x\n")
            try writeLine(file, "property float y\n")
            try writeLine(file, "property float z\n")
            try writeLine(file, "property uchar red\n")
            try writeLine(file, "property uchar green\n")
            try writeLine(file, "property uchar blue\n")
            try writeLine(file, "end_header\n")

            var linesWritten = 0
            progress?(0.18, "Writing vertex data...")

            if isCompact {
                var offset = headerSize
                for _ in 0..<compactRecordCount {
                    if isCancelled?() == true { return false }
                    guard offset + recordSize <= data.count else { break }
                    let idx = readUInt32(at: offset) ?? UInt32.max
                    offset += MemoryLayout<UInt32>.size
                    guard idx < UInt32(maxVoxels), let voxel = readVoxel(at: offset) else {
                        offset += voxelStride
                        continue
                    }
                    offset += voxelStride
                    let px = voxel.positionAndConfidence.x
                    let py = voxel.positionAndConfidence.y
                    let pz = voxel.positionAndConfidence.z
                    let r = clampedColor(voxel.colorAndSampleCount.x)
                    let g = clampedColor(voxel.colorAndSampleCount.y)
                    let b = clampedColor(voxel.colorAndSampleCount.z)
                    if format == .binaryLittleEndian {
                        try writeFloatLE(file, px)
                        try writeFloatLE(file, py)
                        try writeFloatLE(file, pz)
                        var rr = UInt8(r), gg = UInt8(g), bb = UInt8(b)
                        try withUnsafeBytes(of: &rr) { bytes in try file.write(contentsOf: bytes) }
                        try withUnsafeBytes(of: &gg) { bytes in try file.write(contentsOf: bytes) }
                        try withUnsafeBytes(of: &bb) { bytes in try file.write(contentsOf: bytes) }
                    } else {
                        try writeLine(file, "\(px) \(py) \(pz) \(r) \(g) \(b)\n")
                    }
                    linesWritten += 1
                    if linesWritten % progressUpdateStride == 0 {
                        let fraction = vertexCount > 0 ? Double(linesWritten) / Double(vertexCount) : 1.0
                        progress?(0.18 + (0.80 * min(1.0, fraction)), "Writing vertices...")
                    }
                }
            } else {
                for i in stride(from: 0, to: data.count, by: voxelStride) {
                    if isCancelled?() == true { return false }
                    guard let voxel = readVoxel(at: i) else { break }
                    if voxel.colorAndSampleCount.w <= 0 {
                        continue
                    }
                    let px = voxel.positionAndConfidence.x
                    let py = voxel.positionAndConfidence.y
                    let pz = voxel.positionAndConfidence.z
                    let r = clampedColor(voxel.colorAndSampleCount.x)
                    let g = clampedColor(voxel.colorAndSampleCount.y)
                    let b = clampedColor(voxel.colorAndSampleCount.z)
                    if format == .binaryLittleEndian {
                        try writeFloatLE(file, px)
                        try writeFloatLE(file, py)
                        try writeFloatLE(file, pz)
                        var rr = UInt8(r), gg = UInt8(g), bb = UInt8(b)
                        try withUnsafeBytes(of: &rr) { bytes in try file.write(contentsOf: bytes) }
                        try withUnsafeBytes(of: &gg) { bytes in try file.write(contentsOf: bytes) }
                        try withUnsafeBytes(of: &bb) { bytes in try file.write(contentsOf: bytes) }
                    } else {
                        try writeLine(file, "\(px) \(py) \(pz) \(r) \(g) \(b)\n")
                    }
                    linesWritten += 1
                    if linesWritten % progressUpdateStride == 0 {
                        let fraction = vertexCount > 0 ? Double(linesWritten) / Double(vertexCount) : 1.0
                        progress?(0.18 + (0.80 * min(1.0, fraction)), "Writing vertices...")
                    }
                }
            }

            progress?(0.98, "Finishing file...")
            return linesWritten > 0
        } catch {
            return false
        }
    }

    private func sanitizeFileName(_ input: String) -> String {
        let forbidden = CharacterSet(charactersIn: "/\\?%*|\"<>:")
        let cleaned = input.components(separatedBy: forbidden).joined(separator: "_")
        return cleaned.trimmingCharacters(in: .whitespacesAndNewlines)
    }
    
    private var currentPointCount: Int = 0  // Track accumulated points for motion logic
    
    private func updateUniforms(frame: ARFrame, depthMap: CVPixelBuffer) {
        let uniformsPtr = uniformBuffer.contents().assumingMemoryBound(to: Uniforms.self)

        // Convert camera-space unprojected points (+Y down, +Z forward)
        // to ARKit camera space (+Y up, +Z backward), then to world space.
        let flipYZ = simd_float4x4(
            SIMD4<Float>(1, 0, 0, 0),
            SIMD4<Float>(0, -1, 0, 0),
            SIMD4<Float>(0, 0, -1, 0),
            SIMD4<Float>(0, 0, 0, 1)
        )

        let localToWorld = frame.camera.transform * flipYZ

        // Scale intrinsics from full camera image resolution to depth resolution.
        // Without this, unprojection is offset and points won't line up with camera view.
        let depthWidth = Float(CVPixelBufferGetWidth(depthMap))
        let depthHeight = Float(CVPixelBufferGetHeight(depthMap))
        let imageWidth = Float(frame.camera.imageResolution.width)
        let imageHeight = Float(frame.camera.imageResolution.height)

        var scaledIntrinsics = frame.camera.intrinsics
        let sx = imageWidth / depthWidth
        let sy = imageHeight / depthHeight
        scaledIntrinsics[0][0] /= sx
        scaledIntrinsics[1][1] /= sy
        scaledIntrinsics[2][0] /= sx
        scaledIntrinsics[2][1] /= sy
        let cameraIntrinsicsInversed = simd_inverse(scaledIntrinsics)
        
        uniformsPtr.pointee.localToWorld = localToWorld
        uniformsPtr.pointee.cameraIntrinsicsInversed = cameraIntrinsicsInversed
        uniformsPtr.pointee.depthResolution = SIMD2<Float>(depthWidth, depthHeight)
        uniformsPtr.pointee.imageResolution = SIMD2<Float>(imageWidth, imageHeight)
        uniformsPtr.pointee.voxelSize = voxelSize
        uniformsPtr.pointee.maxVoxels = UInt32(maxVoxels)
    }
    
    private func createTexture(from pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> MTLTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        
        var cvMetalTexture: CVMetalTexture?
        CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault, cvTextureCache!, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &cvMetalTexture)
        
        if let cvMetalTexture = cvMetalTexture {
            return CVMetalTextureGetTexture(cvMetalTexture)
        }
        return nil
    }
    
    // MARK: - Metal-Agnostic Types (Must match Shaders.metal alignment)
    private struct Voxel {
        var positionAndConfidence: SIMD4<Float> // 16 bytes (xyz, w=confidence)
        var colorAndSampleCount: SIMD4<Float>    // 16 bytes (rgb, w=sampleCount)
    }
    
    private struct Uniforms {
        var localToWorld: simd_float4x4           // 64 bytes - Camera-to-world transform
        var cameraIntrinsicsInversed: simd_float3x3 // 48 bytes - Inverse of camera intrinsics
        var depthResolution: SIMD2<Float>         // 8 bytes
        var imageResolution: SIMD2<Float>         // 8 bytes
        var voxelSize: Float                      // 4 bytes
        var maxVoxels: UInt32                     // 4 bytes
    }
}
