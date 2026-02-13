//
//  ScanSession.swift
//  PointPro
//

import Foundation

enum ScanSessionStatus: String, Codable, CaseIterable {
    case ready
    case scanning
    case completed
}

struct ScanSession: Identifiable, Codable, Hashable {
    let id: UUID
    var name: String
    let createdAt: Date
    var updatedAt: Date
    var status: ScanSessionStatus
    var pointCount: Int
    var dataSizeBytes: Int64

    init(
        id: UUID,
        name: String,
        createdAt: Date,
        updatedAt: Date,
        status: ScanSessionStatus,
        pointCount: Int,
        dataSizeBytes: Int64 = 0
    ) {
        self.id = id
        self.name = name
        self.createdAt = createdAt
        self.updatedAt = updatedAt
        self.status = status
        self.pointCount = pointCount
        self.dataSizeBytes = dataSizeBytes
    }

    private enum CodingKeys: String, CodingKey {
        case id, name, createdAt, updatedAt, status, pointCount, dataSizeBytes
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        id = try c.decode(UUID.self, forKey: .id)
        name = try c.decode(String.self, forKey: .name)
        createdAt = try c.decode(Date.self, forKey: .createdAt)
        updatedAt = try c.decode(Date.self, forKey: .updatedAt)
        status = try c.decode(ScanSessionStatus.self, forKey: .status)
        pointCount = try c.decodeIfPresent(Int.self, forKey: .pointCount) ?? 0
        dataSizeBytes = try c.decodeIfPresent(Int64.self, forKey: .dataSizeBytes) ?? 0
    }
}
