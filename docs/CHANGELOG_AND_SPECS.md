# PointPro — Changelog & Specs

Merged reference for fixes, critical changes, and PDF report spec/plan.

---

# Part 1: Point Cloud Quality Fixes (2026-02-13)

## Problems Identified

### 1. Points Filling Room (Not Sticking to Surfaces)
**Root Cause:** Spatial hash collisions were averaging positions from completely different surfaces, creating phantom points floating in mid-air between actual surfaces.

### 2. No Point Count Displayed
**Root Cause:** The HUD showed hardcoded text "1.0M VOXELS RESERVED" instead of the actual number of active points.

### 3. Low-Confidence Noise
**Root Cause:** The confidence map from ARKit was completely ignored, allowing noisy depth readings to corrupt the point cloud.

### 4. Far-Field Noise
**Root Cause:** Depth cutoff was set to 8 meters; LiDAR accuracy degrades significantly past ~5m.

## Fixes Implemented (Quality)

- **Shaders.metal:** Confidence filtering, collision detection before averaging, depth range 5.0m, atomic point counter.
- **PointCloudEngine.swift:** ObservableObject, counter buffer, confidence map to shader, read counter after each frame.
- **ContentView.swift:** Live point count display, number formatting.

---

# Part 2: Critical LiDAR Point Cloud Fixes (2026-02-13)

**Status:** Complete rewrite based on Apple reference implementation (WWDC20-10611).

## Root Cause

- Points in wrong world positions (ghosting, floating)
- Temporal averaging causing drift
- Wrong motion detection (accumulate when still instead of when moving)

## Fixes Implemented (Critical)

1. **Unprojection math** — Use inverse intrinsics matrix (Apple approach); no manual axis flips.
2. **localToWorld transform** — Proper viewMatrix inverse + rotateToARCamera for landscape-right sensor space.
3. **Removed position averaging** — Keep first position, average color only (prevents ghosting).
4. **Motion detection** — Accumulate when camera **moves**, skip when still (matches Apple).
5. **3D viewport** — PointCloudViewer with orbit, pinch zoom, two-finger pan.

## Reference

- [Waley-Z/ios-depth-point-cloud](https://github.com/Waley-Z/ios-depth-point-cloud)

---

# Part 3: PDF Report Spec (v1)

**Version:** 1.0 · **Date:** 2026-02-14

## Purpose

One PDF per export: scan metadata, georeference, point cloud stats, QA. Measurement table if present; otherwise "No measurements recorded."

## File Naming

`ScanName_yyyy-MM-dd_HH-mm-ss_report.pdf`

## Section Order

1. Cover  
2. Document Control  
3. Executive Summary  
4. Scan Metadata  
5. Spatial Reference / Georeferencing  
6. Data Inventory  
7. Point Cloud Statistics  
8. Quality and Risk  
9. Measurement Register  
10. Visual Evidence  
11. Deliverables Checklist  
12. Compliance + Disclaimer  
13. Technical Annex  

## Georeference

- Mode: `local_only` or `gps_approx`
- EPSG/CRS, lat/lon/alt, heading, accuracy as available.
- Compliance: "GPS approximate only. Not survey-grade." / "Use GCP/checkpoints for high-accuracy workflows."

## Formatting

- Distances: meters, 2 decimals. Area: m², 2 decimals. Coordinates: 6 decimals.

---

# Part 4: PDF Report Implementation Plan (v1)

## Deliverables

1. `ScanReportModel` (data contract)
2. PDF renderer (UIGraphicsPDFRenderer)
3. Export integration (PDF on every export)
4. Measurement table in report
5. Share sheet: report PDF + export file(s)

## Work Breakdown

- **P0:** ScanReportModel, ReportPDFRenderer, no-measurement and with-measurement paths.
- **P1:** Generate report after LAZ/PLY; include in share items; reuse progress/cancel flow.
- **P2:** Snapshot image section; quality/risk block; typography for A4/Letter.

## Acceptance Criteria

- PDF for LAZ and both PLY exports; correct point count and bounds; georef mode correct; report when no measurements; share sheet includes PDF; no export regressions.

## Risks

- Measurement persistence (viewer-scoped vs report); snapshot path; keep report generation off main thread.
