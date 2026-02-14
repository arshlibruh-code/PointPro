//
//  MetalPointCloudView.swift
//  PointPro
//

import SwiftUI
import MetalKit
import ARKit

struct MetalPointCloudView: UIViewRepresentable {
    let device: MTLDevice
    let engine: PointCloudEngine
    let session: ARSession
    
    func makeUIView(context: Context) -> MTKView {
        let mtkView = MTKView()
        mtkView.device = device
        mtkView.delegate = context.coordinator
        mtkView.backgroundColor = .clear
        mtkView.isOpaque = false
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.preferredFramesPerSecond = 30
        
        // --- CRITICAL: Set clear color to transparent so ARView shows through ---
        mtkView.clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 0)
        
        return mtkView
    }
    
    func updateUIView(_ uiView: MTKView, context: Context) {}
    
    func makeCoordinator() -> Coordinator {
        Coordinator(device: device, engine: engine, session: session)
    }
    
    class Coordinator: NSObject, MTKViewDelegate {
        private struct RenderColorUniforms {
            var mode: UInt32
            var hasUsableRGB: UInt32
            var minElevation: Float
            var maxElevation: Float
            var minIntensity: Float
            var maxIntensity: Float
        }

        private let device: MTLDevice
        private let engine: PointCloudEngine
        private let session: ARSession
        private let commandQueue: MTLCommandQueue
        private var pipelineState: MTLRenderPipelineState!
        private var depthStencilState: MTLDepthStencilState!
        
        init(device: MTLDevice, engine: PointCloudEngine, session: ARSession) {
            self.device = device
            self.engine = engine
            self.session = session
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
            
            // --- CRITICAL: Enable Alpha Blending ---
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
            guard let frame = session.currentFrame,
                  let renderPassDescriptor = view.currentRenderPassDescriptor,
                  let commandBuffer = commandQueue.makeCommandBuffer() else { return }

            // Always submit a pass so the overlay can be cleared when idle.
            // If we return early here, the previous drawable can linger and look like
            // points are stuck to the screen.
            if !engine.isScanning && engine.activePointCount == 0 {
                if let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {
                    encoder.endEncoding()
                }
                if let drawable = view.currentDrawable {
                    commandBuffer.present(drawable)
                }
                commandBuffer.commit()
                return
            }

            guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
            
            // 1. Calculate View-Projection Matrix (PORTRAIT)
            let orientation: UIInterfaceOrientation = .portrait
            let viewMatrix = frame.camera.viewMatrix(for: orientation)
            let projectionMatrix = frame.camera.projectionMatrix(for: orientation, viewportSize: view.bounds.size, zNear: 0.01, zFar: 100.0)
            
            // 2. Points are already in world space from the shader's localToWorld transform
            // We just need to apply camera view and projection
            var vpMatrix = projectionMatrix * viewMatrix
            
            // 3. Render
            encoder.setRenderPipelineState(pipelineState)
            encoder.setDepthStencilState(depthStencilState)
            
            encoder.setVertexBuffer(engine.voxelBuffer, offset: 0, index: 0)
            encoder.setVertexBytes(&vpMatrix, length: MemoryLayout<simd_float4x4>.stride, index: 1)
            var colorUniforms = RenderColorUniforms(
                mode: 1,          // RGB
                hasUsableRGB: 1,  // AR capture path has camera RGB
                minElevation: -1,
                maxElevation: 1,
                minIntensity: 0,
                maxIntensity: 1
            )
            encoder.setVertexBytes(&colorUniforms, length: MemoryLayout<RenderColorUniforms>.stride, index: 2)
            
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: engine.maxVoxels)
            
            encoder.endEncoding()
            if let drawable = view.currentDrawable {
                commandBuffer.present(drawable)
            }
            commandBuffer.commit()
        }
        
        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}
    }
}
