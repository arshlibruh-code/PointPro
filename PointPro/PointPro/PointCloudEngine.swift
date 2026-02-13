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

class PointCloudEngine: ObservableObject {
    enum PLYExportFormat {
        case binaryLittleEndian
        case ascii
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
    private let inFlightSemaphore = DispatchSemaphore(value: 1)
    private var lastProcessedTimestamp: TimeInterval = 0
    private let minProcessInterval: TimeInterval = 1.0 / 15.0
    
    // ARSession reference (for anchor management)
    private weak var arSession: ARSession?
    
    init() {
        self.device = MTLCreateSystemDefaultDevice()!
        self.commandQueue = device.makeCommandQueue()!
        
        setupMetal()
        setupBuffers()
        
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
    
    func processFrame(_ frame: ARFrame) {
        guard isScanning else { return }
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
            let count = self.counterBuffer.contents().assumingMemoryBound(to: UInt32.self).pointee
            DispatchQueue.main.async {
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
