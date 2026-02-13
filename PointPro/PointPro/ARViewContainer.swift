//
//  ARViewContainer.swift
//  PointPro
//

import SwiftUI
import RealityKit
import ARKit

/// Simplified ARViewContainer that only provides the world-tracking camera feed.
/// All point cloud rendering is managed by the MetalPointCloudView.
struct ARViewContainer: UIViewRepresentable {
    let session: ARSession
    
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        arView.session = session
        
        arView.environment.background = .cameraFeed()
        arView.renderOptions = [
            .disablePersonOcclusion,
            .disableMotionBlur,
        ]
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {}
}
