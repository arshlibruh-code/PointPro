# PointPro — Product Requirements Document

**Version:** 1.0  
**Date:** 2026-02-13  
**Author:** Arshad + AI  
**Status:** Draft  

---

## 1. Product Vision

### One-Liner
A field-grade LiDAR capture app that lets a solo professional scan anything, analyze it on-device, and share a single link that gives the recipient everything — the 3D view, the report, and the raw LAZ file.

### Who Is This For?
**The Field Pro** — surveyors, site engineers, architects, construction managers, GIS analysts. People who need spatial data captured fast, analyzed on-site, and delivered without touching a laptop.

### What Makes PointPro Different?
| Existing Apps | PointPro |
|---|---|
| Choose between pretty mesh OR raw data | **Both** — dual-stream capture (RAW archive + GPU preview) |
| Require internet for core features | **100% offline** — internet is optional delivery |
| Export files, figure out the rest | **One link** — 3D viewer + report + LAZ download |
| Generic scanner UI | **Template-driven** — app configures itself for the job |

---

## 2. Target Device

- **iPhone 17 Pro / Pro Max** (LiDAR equipped)
- iOS 18+
- ARKit 7+ (sceneDepth API)

### Hardware Capabilities
| Spec | Value |
|------|-------|
| Depth Resolution | 256 × 192 (~49K points/frame) |
| Frame Rate | Up to 60 Hz |
| Optimal Range | 0.5m – 5m |
| Extended Range | Up to 60-70m (degraded accuracy) |
| Accuracy | ±1-3 cm at <20m |
| Point Density | ~8,000 points/m² |

---

## 3. Core Features (10 User Stories)

### Phase 1 — Capture Engine

#### Feature 1: Dual-Stream Capture
> *As a field professional, I want every raw LiDAR point preserved in a LAZ archive while simultaneously seeing a smooth, optimized 3D preview on my screen, so I never sacrifice data quality for performance.*

**Implementation Notes:**
- **Stream A (Archive):** Raw depth frames + RGB + confidence + IMU → buffered to disk in real-time → compressed to LAZ on scan completion
- **Stream B (GPU Proxy):** Real-time voxel grid (configurable 1cm/5cm/10cm cells) for Metal rendering
- Archive runs on a background thread; GPU proxy runs on the render thread
- User never interacts with this choice — it's automatic

**Acceptance Criteria:**
- [ ] Raw data loss = 0% (every frame captured)
- [ ] GPU preview maintains 30fps+ during active scanning
- [ ] LAZ file is generated on scan completion without user action
- [ ] File size indicator visible during scan ("4.2 GB archived")

---

#### Feature 2: Live Coverage HUD
> *As a surveyor, I want to see a real-time AR overlay showing which surfaces have been captured and which are missed, so I can confirm 100% coverage before leaving the site.*

**Implementation Notes:**
- Color-coded overlay on the camera feed:
  - **Green** = captured with high confidence
  - **Yellow** = captured with low confidence (scan again)
  - **Transparent** = not yet scanned
- Coverage percentage displayed as a number ("78% captured")
- Optional audio ping when a new surface region is captured

**Acceptance Criteria:**
- [ ] Overlay renders in real-time without blocking the camera feed
- [ ] Coverage percentage updates live
- [ ] User can toggle HUD on/off

---

#### Feature 3: Reference Points (Geolocation)
> *As a surveyor, I want to drop reference markers — either improvised pins or known survey coordinates — so the scan is georeferenced and compatible with GIS workflows.*

**Implementation Notes:**
- **Improvised Pin:** User taps a surface in AR → pin drops at that 3D coordinate → optional label + photo
- **Known Survey Marker:** User enters WGS84 or local CRS coordinates manually → places pin at a physical survey marker in the scene → scan is georeferenced to that point
- GPS from iPhone auto-captured as fallback (±3-5m accuracy)
- At least 1 reference point recommended per scan; 3+ for proper georeferencing

**Acceptance Criteria:**
- [ ] Pins persist visually in the AR scene during scanning
- [ ] Coordinates are embedded in the exported LAZ metadata
- [ ] GPS fallback captured automatically even with no manual pins

---

### Phase 2 — On-Device Analysis

#### Feature 4: Measurement Tool
> *As an architect, I want to snap a tape measure between any two points in the LiDAR cloud to get sub-centimeter distance, so I can verify dimensions without physical tools.*

**Implementation Notes:**
- Tap-to-place two endpoints on the point cloud
- Distance calculated against the RAW archive (not the downsampled GPU proxy) for maximum precision
- Displays: distance in meters + feet, horizontal distance, vertical distance
- Multiple measurements can be active at once (labeled A, B, C...)
- Measurements persist and are included in the report

**Acceptance Criteria:**
- [ ] Measurement snaps to nearest point (not arbitrary space)
- [ ] Accuracy within ±1cm at <5m range
- [ ] Measurements saved with the scan session

---

#### Feature 5: Surface Deviation Heatmap
> *As a construction manager, I want to select a surface and see a color-coded heatmap showing where it deviates from flat, so I can identify pooling, bowing, or leveling issues on-site.*

**Implementation Notes:**
- User taps a surface → app fits a best-fit plane (least squares)
- Points colored by deviation from that plane:
  - **Blue** = below plane (dips/pooling)
  - **Green** = within tolerance (flat)
  - **Red** = above plane (bumps)
- Tolerance is configurable (default: ±2cm)
- Metal shader applied in real-time over the point cloud
- Screenshot of heatmap auto-included in report

**Acceptance Criteria:**
- [ ] Plane fitting runs in <1 second
- [ ] Heatmap renders in real-time on the GPU proxy
- [ ] Min/max/average deviation displayed as numbers
- [ ] Configurable tolerance threshold

---

#### Feature 6: Volume Estimation
> *As a site manager, I want to lasso a pile of material and get an instant cubic-volume estimate, so I can track stockpile inventory without complex software.*

**Implementation Notes:**
- Depends on plane-fitting from Feature 5 (ground plane detection)
- User draws a boundary (lasso) around the region of interest on screen
- App calculates volume between the ground plane and the LiDAR surface
- Displays result in m³ and optionally converts to tonnes (if material density is known from template)

**Acceptance Criteria:**
- [ ] Lasso tool is intuitive (draw with finger)
- [ ] Volume calculation completes in <2 seconds
- [ ] Result included in scan report

---

### Phase 3 — Project & Session Management

#### Feature 7: Project Templates
> *As a field professional, I want to select a job type before scanning so the app auto-configures the right tools and settings, so I spend zero time in settings menus.*

**Implementation Notes:**
- Templates available at scan start:
  - **"Floor Flatness Check"** → enables heatmap, sets ±2cm tolerance, suggests top-down scan pattern
  - **"Stockpile Volume"** → enables volume tool, asks for material type
  - **"As-Built Documentation"** → full scan, all tools available, photo annotations enabled
  - **"Custom"** → manual configuration
- Templates are extensible — new ones can be added in future updates
- Template selection is the FIRST thing the user sees when starting a new scan

**Acceptance Criteria:**
- [ ] At least 3 templates + Custom available at launch
- [ ] Template auto-enables relevant tools and hides irrelevant ones
- [ ] User can switch to Custom mid-scan if needed

---

#### Feature 8: Multi-Scan Sessions (Sites)
> *As a surveyor working on a large site, I want to organize multiple discrete scans under a single "Site" project, so all data from one job stays together.*

**Implementation Notes:**
- Hierarchy: **Site → Scan(s)**
- A Site has: name, date, GPS location, notes
- Each Scan within a Site is independent but grouped
- Site-level report aggregates all scans
- Future: alignment/stitching of multiple scans (not V1)

**Acceptance Criteria:**
- [ ] User can create a Site and add multiple Scans to it
- [ ] Each Scan is independently viewable and exportable
- [ ] Site-level metadata (name, location, notes) editable

---

### Phase 4 — Share & Deliver

#### Feature 9: One-Link Publishing
> *As a project lead, I want to tap "Publish" and get a single URL that my office team can open to view the 3D scan, read the report, and download the LAZ file — no app install required.*

**Implementation Notes:**
- Upload is **progressive** (3 stages):
  1. Report + metadata (~1MB) → link goes live immediately
  2. Optimized point cloud (~30-50MB) → 3D web viewer activates
  3. Full LAZ archive (~0.5-2GB) → download button activates
- Upload is **resumable** (chunked, survives connection drops)
- Upload policy: WiFi Only (default) / WiFi + Cellular / Manual
- Web page uses **Potree** (open-source WebGL point cloud viewer) or similar
- Backend: FastAPI endpoint receiving uploads, serving the viewer page

**The Link Page Contains:**
```
pointpro.app/s/abc123
├── 3D Viewer (interactive, WebGL)
├── Report Summary (metadata, measurements, heatmap screenshots)
├── Download LAZ button
└── Share button (copy link)
```

**Acceptance Criteria:**
- [ ] Link is shareable within 1 minute of tapping Publish (report-only stage)
- [ ] 3D viewer loads without any app/plugin install
- [ ] Full LAZ upload resumes after connection loss
- [ ] Upload queue works — multiple scans can be queued

---

#### Feature 10: AirDrop & Local Export
> *As a field professional with no internet, I want to share the LAZ file directly to a colleague's device via AirDrop, or save it to Files, so I can deliver data without any server dependency.*

**Implementation Notes:**
- Standard iOS Share Sheet integration
- Export options:
  - **LAZ** (primary — raw archive with full metadata)
  - **PLY** (compatibility — for tools that don't read LAZ)
  - **Report PDF** (standalone, auto-generated)
- AirDrop sends the LAZ file directly
- "Save to Files" puts it in the iOS Files app (iCloud Drive, local, external drive)

**Acceptance Criteria:**
- [ ] Share sheet accessible from scan detail screen
- [ ] LAZ and PLY export both functional
- [ ] PDF report exportable independently
- [ ] Works with zero internet connectivity

---

## 4. Technical Architecture

### On-Device Stack
```
┌──────────────────────────────────────────────┐
│                 PointPro App                  │
├──────────────────────────────────────────────┤
│  UI Layer          │  SwiftUI                 │
│  AR Rendering      │  ARKit + RealityKit      │
│  Point Cloud GPU   │  Metal (custom shaders)  │
│  Depth Processing  │  ARKit sceneDepth API    │
│  Storage           │  Core Data + File System │
│  Compression       │  LAZ encoder (LASzip)    │
│  Export            │  ShareSheet + FileManager │
│  Networking        │  URLSession (background)  │
└──────────────────────────────────────────────┘
```

### Backend Stack (Minimal)
```
┌──────────────────────────────────────────────┐
│              pointpro.app                     │
├──────────────────────────────────────────────┤
│  API               │  FastAPI (Python)        │
│  Storage           │  S3-compatible (LAZ)     │
│  3D Viewer         │  Potree (WebGL)          │
│  Report Renderer   │  Static HTML/PDF         │
│  Database          │  PostgreSQL (metadata)   │
└──────────────────────────────────────────────┘
```

### Dual-Stream Data Flow
```
ARKit Frame (60Hz)
    │
    ├──▶ Stream A: RAW Archive
    │       │
    │       ├── Depth buffer (CVPixelBuffer)
    │       ├── RGB image (camera frame)
    │       ├── Confidence map
    │       ├── Camera intrinsics + extrinsics
    │       ├── IMU data (accelerometer + gyro)
    │       └── GPS coordinate (when available)
    │       │
    │       ▼
    │    Disk buffer → LAZ compression on scan end
    │
    └──▶ Stream B: GPU Proxy
            │
            ├── Unproject depth → 3D points
            ├── Voxel grid filtering (configurable density)
            ├── Color from RGB camera
            └── Metal render pipeline (point cloud + HUD)
```

### Upload Pipeline
```
Scan Complete
    │
    ▼
Local Storage (always)
    │
    ├── LAZ file (raw archive)
    ├── Report PDF (auto-generated)
    └── Optimized point cloud (voxel grid export)
    │
    ▼
Upload Queue (when network available)
    │
    ├── Stage 1: Report + metadata    (~1MB)     → Link live
    ├── Stage 2: Optimized cloud      (~30-50MB) → Viewer live  
    └── Stage 3: Full LAZ             (~0.5-2GB) → Download live
    │
    ▼
pointpro.app/s/{scan_id}
```

---

## 5. Offline-First Principle

**PointPro is a field tool. Internet is a luxury, not a requirement.**

| Capability | Offline | Online |
|---|---|---|
| Scanning | ✅ | ✅ |
| Measurements | ✅ | ✅ |
| Heatmaps | ✅ | ✅ |
| Volume Estimation | ✅ | ✅ |
| Report Generation | ✅ | ✅ |
| AirDrop / Local Export | ✅ | ✅ |
| One-Link Publishing | ❌ (queued) | ✅ |
| GPS Georeferencing | Partial (cached) | ✅ |

---

## 6. Development Phases & Timeline

### Phase 1: Capture Engine (Weeks 1-4)
- ARKit setup + sceneDepth pipeline
- Dual-stream architecture (raw buffer + voxel grid)
- Metal point cloud renderer
- Live Coverage HUD
- Reference point dropping
- LAZ export (using LASzip or custom encoder)

### Phase 2: Analysis Tools (Weeks 5-7)
- Measurement tool (tap-to-measure on point cloud)
- Plane fitting (least squares)
- Surface deviation heatmap (Metal shader)
- Volume estimation (lasso + ground plane)

### Phase 3: Project Management (Weeks 8-9)
- Site / Scan data model (Core Data)
- Project templates system
- Multi-scan sessions
- Auto-generated PDF reports

### Phase 4: Share & Deliver (Weeks 10-12)
- Upload queue (background URLSession)
- Progressive upload (3-stage)
- FastAPI backend endpoint
- Potree web viewer integration
- AirDrop / Share Sheet

---

## 7. File & Storage Strategy

### On-Device Storage
- **LAZ archives** stored in app's Documents directory
- **Optimized previews** stored as lightweight binary blobs
- **Reports** generated as PDF on demand
- **Clear storage indicators** in the UI ("This scan: 1.2 GB")
- **Future:** Archive to iCloud / external drive

### Export Formats
| Format | Purpose | When |
|---|---|---|
| **LAZ** | Primary archive, GIS-compatible | Always available |
| **PLY** | Compatibility export | On demand |
| **PDF** | Report with metadata + screenshots | Auto-generated |

---

## 8. UI/UX Principles

1. **Template-first:** The first screen on new scan = pick your job type
2. **One-hand operable:** Primary actions reachable with thumb (scanning is often one-handed)
3. **Glanceable HUD:** Coverage %, point count, file size — visible without tapping
4. **No settings rabbit holes:** Templates handle configuration; Custom mode for power users
5. **Dark theme default:** Field Pros work in bright sunlight; dark UI reduces glare on the AR view
6. **Haptic feedback:** Subtle taps when pins are dropped, measurements confirmed, scan boundaries reached

---

## 9. Success Metrics (V1)

- [ ] Scan-to-export time < 5 minutes for a standard room
- [ ] LAZ file opens correctly in QGIS and CloudCompare
- [ ] Measurement accuracy within ±1cm at 5m range
- [ ] Heatmap renders in real-time (30fps+)
- [ ] One-link page loads in < 3 seconds
- [ ] App runs stable for 10+ minute continuous scans without crash
- [ ] App size < 100MB (no bloated dependencies)

---

## 10. Open Questions

- [ ] LASzip library availability for iOS (Swift wrapper needed? or use C library directly?)
- [ ] Potree vs. alternative web viewers for the share page
- [ ] Core Data vs. SwiftData for session management
- [ ] Metal vs. RealityKit for primary rendering (Metal gives more shader control)
- [ ] TestFlight distribution strategy during development
