# Critical LiDAR Point Cloud Fixes

**Date:** 2026-02-13  
**Status:** Complete Rewrite Based on Apple Reference Implementation

---

## 🔍 Root Cause Analysis

After analyzing the Apple WWDC20-10611 reference implementation (`Waley-Z/ios-depth-point-cloud`), we identified **fundamental architectural errors** in our initial approach that caused:

1. ❌ Points placed in incorrect world positions (ghosting, floating)
2. ❌ Temporal averaging causing point clouds to drift
3. ❌ Wrong motion detection logic (backward from Apple's approach)

---

## ✅ Fixes Implemented

### 1. **Fixed Unprojection Math** (Core Issue)

**Problem:**  
Manual intrinsics decomposition with hardcoded axis flips:
```metal
float fx = intrinsics[0][0] * scale;
float xCam = (x - cx) * depth / fx;
float yCam = -(y - cy) * depth / fy;  // Manual Y flip
float zCam = -depth;                   // Manual Z flip
```

**Solution:**  
Use inverse intrinsics matrix (Apple reference approach):
```metal
float3 localPoint = cameraIntrinsicsInversed * float3(cameraPoint, 1.0) * depth;
float4 worldPos = localToWorld * float4(localPoint, 1.0);
float3 p = worldPos.xyz / worldPos.w;  // Homogeneous divide
```

**Why this works:**  
- Matrix inversion handles all coordinate system transformations correctly
- No manual axis flips needed - transforms are mathematically sound
- Matches proven production implementations (Polycam, etc.)

---

### 2. **Fixed `localToWorld` Transform**

**Problem:**  
Used `camera.transform` directly with anchor offset:
```swift
let anchorInverse = simd_inverse(anchor.transform)
cameraTransform = anchorInverse * frame.camera.transform
```

**Solution:**  
Compute proper `localToWorld` with coordinate flips:
```swift
let viewMatrixInversed = simd_inverse(camera.viewMatrix(for: .portrait))
let flipYZ = matrix4x4(flipY: -1, flipZ: -1)
let portraitRotation = matrix4x4(rotateZ: 90°)
let rotateToARCamera = flipYZ * portraitRotation
localToWorld = viewMatrixInversed * rotateToARCamera
```

**Why this matters:**  
- LiDAR depth buffer is in **landscape-right sensor space**, not camera display space
- `rotateToARCamera` handles the coordinate system transformation
- Without this, points are placed with wrong orientations

---

### 3. **Removed Position Averaging** (Anti-Ghosting)

**Problem:**  
Averaging positions across frames when voxels were re-observed:
```metal
float3 avgPos = (existingPos * n + newPos) / (n + 1.0);  // CAUSES GHOSTING
voxelGrid[idx].position = avgPos;
```

**Solution:**  
Keep first position, only average color:
```metal
// CRITICAL: Keep position fixed (first sample)
float3 avgColor = (existingColor * n + newColor) / (n + 1.0);
voxelGrid[idx].color = avgColor;
// Position remains unchanged from first sample
```

**Why this works:**  
- When camera moves, same physical point appears at slightly different unprojected positions due to LiDAR parallax
- Averaging those creates a "cloud" of points floating around the real surface
- First sample is just as accurate as averaged samples, but stable

---

### 4. **Fixed Motion Detection Logic**

**Problem (Our Old Code):**  
Skip frames when camera is moving, accumulate when still:
```swift
if movement > 0.02 {
    return  // Skip frame - camera moving
}
// Accumulate when still
```

**Solution (Apple Approach):**  
Accumulate when camera **moves**, skip when **still**:
```swift
let hasMovedEnough = movement > 0.02
if !hasMovedEnough && currentPointCount > 0 {
    return  // Skip - redundant data from same viewpoint
}
// Accumulate - new angle = new data
```

**Why this is correct:**  
- **Moving camera** = new viewing angle = new surfaces visible = valuable data
- **Still camera** = same viewpoint = redundant pixels = wasted processing
- Professional scanning workflow: move continuously, app captures from many angles

---

### 5. **Added 3D Viewport** (New Feature)

Created `PointCloudViewer.swift` with:
- ✅ Fullscreen black viewport
- ✅ Orbit camera (drag to rotate, pinch to zoom, two-finger pan)
- ✅ Reads from same `voxelBuffer` as AR view
- ✅ "VIEW 3D" button appears when not scanning

**Usage:**  
1. Scan object
2. Stop scan
3. Tap "VIEW 3D"
4. Interact with point cloud using touch gestures

---

## 📊 Expected Results

### Before (Broken):
- ❌ Points floating 10-50cm from actual surfaces
- ❌ "Ghost clouds" when camera moves
- ❌ Points drift over time as averaging corrupts positions
- ❌ Objects look "fuzzy" with cloud-like appearance

### After (Fixed):
- ✅ Points placed within ±1cm of actual surfaces (LiDAR accuracy)
- ✅ No ghosting - points stay locked to surfaces
- ✅ Clean, stable point clouds
- ✅ Can view/inspect captured data in 3D orbit view

---

## 🧪 Testing Checklist

1. **Static Object Test:**
   - [ ] Place iPhone on tripod, scan a static object
   - [ ] Points should be stable, no drift over 30 seconds
   - [ ] Stop scan, tap "VIEW 3D" - points should look crisp

2. **Movement Test:**
   - [ ] Walk around an object while scanning
   - [ ] Points should accumulate as you move
   - [ ] No "trailing" or floating points behind you

3. **Accuracy Test:**
   - [ ] Scan a flat surface (table, wall)
   - [ ] In 3D view, points should form a clean plane
   - [ ] Max deviation < 2cm for surfaces at 1-3m range

4. **3D Viewer Test:**
   - [ ] Drag to rotate - smooth orbit
   - [ ] Pinch to zoom - scales correctly
   - [ ] Two-finger pan - translates view
   - [ ] Close and reopen - state preserved

---

## 📚 Reference Implementation

All fixes based on Apple's official sample code:
- **Project:** `SceneDepthPointCloud` (WWDC20-10611)
- **GitHub:** [Waley-Z/ios-depth-point-cloud](https://github.com/Waley-Z/ios-depth-point-cloud)
- **Key Files:**
  - `Shaders.metal` → `worldPoint()` function (unprojection)
  - `Renderer.swift` → `updateUniforms()` (transform computation)

---

## 🚀 Next Steps

After verifying these fixes work:

1. **Dual-Stream Architecture** (from PRD):
   - Stream A: Archive full 49K points/frame to disk → LAZ export
   - Stream B: GPU voxel grid (current implementation) for preview

2. **Advanced Features:**
   - Measurement tool (tap-to-measure)
   - Surface deviation heatmap
   - Volume estimation
   - LAZ export

3. **Optimization:**
   - Test with continuous 10+ minute scans
   - Verify no memory leaks
   - Optimize voxel hash collisions

---

## 🛠️ Files Changed

- `Shaders.metal` - Unprojection logic, removed averaging
- `PointCloudEngine.swift` - Transform computation, motion logic
- `PointCloudViewer.swift` - NEW - 3D viewport
- `ContentView.swift` - Added "VIEW 3D" button
- `FIXES_2026-02-13.md` - This document
