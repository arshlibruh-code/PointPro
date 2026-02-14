import Foundation

struct CaptureLocationMetadata: Codable, Hashable {
    let latitude: Double
    let longitude: Double
    let altitude: Double
    let horizontalAccuracy: Double
    let verticalAccuracy: Double
    let headingDegrees: Double?
    let timestamp: Date
}

struct ScanCaptureMetadata: Codable, Hashable {
    let capturedAt: Date
    let worldOriginTransform: [Float]
    let location: CaptureLocationMetadata?
    let deviceModel: String
    let systemName: String
    let systemVersion: String
    let appVersion: String?
    let depthResolution: [Int]
    let colorResolution: [Int]
}

struct ScanExportSidecar: Codable {
    struct CoordinateFrame: Codable {
        let name: String
        let units: String
        let worldOriginTransform: [Float]
    }

    struct ExportFiles: Codable {
        let laz: String
        let metadata: String
    }

    struct PointSchema: Codable {
        let format: String
        let coordinateType: String
        let colorType: String
        let intensityType: String
        let classificationType: String
        let scale: [Double]
        let offset: [Double]
        let pointCount: Int
    }

    struct Georeference: Codable {
        let mode: String
        let epsg: Int?
        let crsName: String?
        let latitude: Double?
        let longitude: Double?
        let altitude: Double?
        let horizontalAccuracy: Double?
        let verticalAccuracy: Double?
        let headingDegrees: Double?
        let note: String?
        let timestamp: Date?
    }

    struct Stats: Codable {
        let pointCount: Int
        let boundsMin: [Double]
        let boundsMax: [Double]
    }

    let schemaVersion: Int
    let exportedAt: Date
    let scanID: UUID
    let scanName: String
    let sessionCreatedAt: Date
    let sessionUpdatedAt: Date
    let files: ExportFiles
    let coordinateFrame: CoordinateFrame
    let pointSchema: PointSchema
    let georeference: Georeference
    let capture: ScanCaptureMetadata?
    let stats: Stats
}
