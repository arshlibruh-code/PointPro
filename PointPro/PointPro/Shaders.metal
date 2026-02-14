#include <metal_stdlib>
using namespace metal;

struct Voxel {
    float4 positionAndConfidence; // xyz = position, w = confidence
    float4 colorAndSampleCount;   // rgb = color, w = sampleCount
};

struct Uniforms {
    float4x4 localToWorld;              // Camera-to-world transform (viewMatrix.inverse * rotateToARCamera)
    float3x3 cameraIntrinsicsInversed;  // Inverse of camera intrinsics for unprojection
    float2 depthResolution;
    float2 imageResolution;
    float voxelSize;
    uint32_t maxVoxels;
};

// Spatial Hash Function (3D Object Space -> 1D Buffer Index)
uint32_t spatialHash(float3 p, float voxelSize, uint32_t maxVoxels) {
    int3 x = int3(floor(p / voxelSize));
    // Standard spatial hash constants
    uint3 h = uint3(x) * uint3(73856093, 19349663, 83492791);
    return (h.x ^ h.y ^ h.z) % maxVoxels;
}

// MARK: - Compute Kernel

kernel void accumulatePoints(
    texture2d<float, access::read> depthTexture [[texture(0)]],
    texture2d<float, access::sample> yTexture [[texture(1)]],
    texture2d<float, access::sample> cbcrTexture [[texture(2)]],
    texture2d<uint, access::read> confidenceTexture [[texture(3)]],
    device Voxel* voxelGrid [[buffer(0)]],
    constant Uniforms& uniforms [[buffer(1)]],
    device atomic_uint* pointCounter [[buffer(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    constexpr float kMinDepthMeters = 0.14;

    if (gid.x >= uint(uniforms.depthResolution.x) || gid.y >= uint(uniforms.depthResolution.y)) return;

    // 1. Read Depth
    float depth = depthTexture.read(gid).r;
    if (depth <= kMinDepthMeters) return;
    
    // 2. Read Confidence (0=low, 1=medium, 2=high)
    uint confidence = confidenceTexture.read(gid).r;
    if (confidence == 0) return; // reject low-confidence samples

    // 2. Unproject to Camera Space using inverse intrinsics (Apple reference approach)
    // This is the CORRECT way to unproject - no manual axis flips needed
    float2 cameraPoint = float2(gid);
    float3 localPoint = uniforms.cameraIntrinsicsInversed * float3(cameraPoint, 1.0) * depth;
    
    // 3. Camera Space -> World Space using localToWorld transform
    float4 worldPos = uniforms.localToWorld * float4(localPoint, 1.0);
    float3 p = worldPos.xyz / worldPos.w;  // Homogeneous divide

    // 4. Sample RGB using bilinear sampler
    // This handles resolution differences and orientation mapping automatically
    constexpr sampler s(address::clamp_to_edge, filter::linear, coord::normalized);
    float2 uv = cameraPoint / uniforms.depthResolution;
    
    float y = yTexture.sample(s, uv).r;
    float2 cbcr = cbcrTexture.sample(s, uv).rg - 0.5;
    
    float3 rgb;
    rgb.r = y + 1.402 * cbcr.y;
    rgb.g = y - 0.344136 * cbcr.x - 0.714136 * cbcr.y;
    rgb.b = y + 1.772 * cbcr.x;
    rgb = clamp(rgb, 0.0, 1.0);

    // 5. Build Spatial Hash
    uint32_t idx = spatialHash(p, uniforms.voxelSize, uniforms.maxVoxels);

    // 6. Write to Voxel Grid (NO AVERAGING - prevents ghosting)
    Voxel v = voxelGrid[idx];
    if (v.colorAndSampleCount.w == 0) {
        // Empty slot - write new voxel
        voxelGrid[idx].positionAndConfidence = float4(p, 1.0);
        voxelGrid[idx].colorAndSampleCount = float4(rgb, 1.0);
        atomic_fetch_add_explicit(pointCounter, 1, memory_order_relaxed);
    } else {
        // Occupied slot - check if it's the SAME voxel cell or a hash collision
        float3 existingPos = v.positionAndConfidence.xyz;
        float distToExisting = length(existingPos - p);
        
        // If positions are within the same voxel cell, increment sample count but DON'T average position
        // This preserves spatial accuracy while tracking confidence
        if (distToExisting < uniforms.voxelSize * 0.9) {
            // Same voxel - update color but keep position fixed
            float n = v.colorAndSampleCount.w;
            float3 avgColor = (v.colorAndSampleCount.xyz * n + rgb) / (n + 1.0);
            
            // CRITICAL: Keep position as-is (first sample), only average color
            voxelGrid[idx].colorAndSampleCount = float4(avgColor, min(n + 1.0, 1000.0));
        } else {
            // Hash collision from different location - overwrite with newer point
            voxelGrid[idx].positionAndConfidence = float4(p, 1.0);
            voxelGrid[idx].colorAndSampleCount = float4(rgb, 1.0);
        }
    }
}

// MARK: - Render Shaders

struct VertexOut {
    float4 position [[position]];
    float3 color;
    float pointSize [[point_size]];
};

struct RenderColorUniforms {
    uint mode;           // 0:auto, 1:rgb, 2:elevation, 3:intensity, 4:classification
    uint hasUsableRGB;   // 0/1
    float minElevation;
    float maxElevation;
    float minIntensity;
    float maxIntensity;
};

float3 viridis(float t)
{
    t = clamp(t, 0.0, 1.0);
    const float3 c0 = float3(0.267, 0.005, 0.329);
    const float3 c1 = float3(0.230, 0.322, 0.546);
    const float3 c2 = float3(0.128, 0.567, 0.551);
    const float3 c3 = float3(0.369, 0.789, 0.383);
    const float3 c4 = float3(0.993, 0.906, 0.144);

    if (t < 0.25) {
        return mix(c0, c1, t / 0.25);
    } else if (t < 0.5) {
        return mix(c1, c2, (t - 0.25) / 0.25);
    } else if (t < 0.75) {
        return mix(c2, c3, (t - 0.5) / 0.25);
    }
    return mix(c3, c4, (t - 0.75) / 0.25);
}

float3 classificationColor(float c)
{
    int cls = clamp(int(round(c * 255.0)), 0, 255);
    switch (cls)
    {
        case 1: return float3(0.85, 0.85, 0.85); // Unclassified
        case 2: return float3(0.70, 0.50, 0.30); // Ground
        case 3: return float3(0.40, 0.75, 0.30); // Low vegetation
        case 4: return float3(0.20, 0.65, 0.25); // Medium vegetation
        case 5: return float3(0.10, 0.45, 0.18); // High vegetation
        case 6: return float3(0.90, 0.55, 0.20); // Building
        case 7: return float3(0.88, 0.25, 0.25); // Low noise
        case 9: return float3(0.20, 0.45, 0.90); // Water
        case 17: return float3(0.76, 0.66, 0.50); // Bridge deck
        case 18: return float3(1.00, 0.10, 0.70); // High noise
        default: return float3(0.62, 0.62, 0.62);
    }
}

vertex VertexOut pointVertex(
    uint vid [[vertex_id]],
    device const Voxel* voxelGrid [[buffer(0)]],
    constant float4x4& vpMatrix [[buffer(1)]],
    constant RenderColorUniforms& colorUniforms [[buffer(2)]]
) {
    VertexOut out;
    Voxel v = voxelGrid[vid];
    
    if (v.colorAndSampleCount.w == 0) {
        out.position = float4(0, 0, 0, 0); 
        out.pointSize = 0;
    } else {
        out.position = vpMatrix * float4(v.positionAndConfidence.xyz, 1.0);
        uint mode = colorUniforms.mode;
        if (mode == 0) {
            mode = (colorUniforms.hasUsableRGB > 0) ? 1 : 2;
        }

        float3 color = v.colorAndSampleCount.xyz;
        if (mode == 2) {
            float span = max(0.0001, colorUniforms.maxElevation - colorUniforms.minElevation);
            float t = (v.positionAndConfidence.y - colorUniforms.minElevation) / span;
            color = viridis(t);
        } else if (mode == 3) {
            float span = max(0.0001, colorUniforms.maxIntensity - colorUniforms.minIntensity);
            float t = (v.positionAndConfidence.w - colorUniforms.minIntensity) / span;
            color = float3(t, t, t);
        } else if (mode == 4) {
            float clsNormalized = clamp((v.colorAndSampleCount.w - 1.0) / 255.0, 0.0, 1.0);
            color = classificationColor(clsNormalized);
        }

        out.color = clamp(color, 0.0, 1.0);
        out.pointSize = 5.0; // Optimized for high-density capture
    }
    return out;
}

// MARK: - Fragment Shader (Round Dots)
fragment float4 pointFragmentRound(VertexOut in [[stage_in]], float2 pointCoord [[point_coord]]) {
    if (in.pointSize == 0) discard_fragment();
    
    // Calculate distance from center of point (0.5, 0.5)
    float dist = length(pointCoord - float2(0.5));
    if (dist > 0.5) discard_fragment();
    
    // Subtle additive glow/shading
    float alpha = smoothstep(0.5, 0.45, dist);
    return float4(in.color, alpha);
}
