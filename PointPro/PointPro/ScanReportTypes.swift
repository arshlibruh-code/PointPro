import Foundation

enum ScanReportMeasurementType: String, Codable {
    case distance
    case polyline
    case closedArea = "closed_area"
    case crossSection = "cross_section"
    case elevationProfile = "elevation_profile"
}

struct ScanReportVertex: Codable, Hashable {
    let x: Double
    let y: Double
    let z: Double
}

struct ScanReportMeasurement: Identifiable, Codable, Hashable {
    let id: String
    let type: ScanReportMeasurementType
    let createdAt: Date
    let vertexCount: Int
    let vertices: [ScanReportVertex]
    let lengthMeters: Double?
    let perimeterMeters: Double?
    let areaSquareMeters: Double?
    let swathWidthMeters: Double?
    let minElevationMeters: Double?
    let maxElevationMeters: Double?
    let reliefMeters: Double?
    let totalRiseMeters: Double?
    let totalFallMeters: Double?
    let averageSlopePercent: Double?
    let maxSlopePercent: Double?
    let notes: String?
}
