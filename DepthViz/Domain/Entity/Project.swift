//
//  Project.swift
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

import Foundation
import CoreLocation

/// Represents a scanning project containing multiple scans and metadata
struct Project: Identifiable, Codable {
    let id: UUID
    var name: String
    var description: String
    let createdAt: Date
    var updatedAt: Date
    
    // Location Data
    var location: LocationMarker?
    var latitude: Double?
    var longitude: Double?
    var address: String?
    
    // Scan Data
    var scanIDs: [UUID] // IDs of scans belonging to this project
    var pointCount: Int
    var fileSize: Int64
    
    var thumbnailPath: String?
    
    init(id: UUID = UUID(), name: String, description: String = "", location: LocationMarker? = nil) {
        self.id = id
        self.name = name
        self.description = description
        self.createdAt = Date()
        self.updatedAt = Date()
        self.location = location
        self.latitude = location?.latitude
        self.longitude = location?.longitude
        self.address = location?.address
        self.scanIDs = []
        self.pointCount = 0
        self.fileSize = 0
    }
}
