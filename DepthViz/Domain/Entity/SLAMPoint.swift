//
//  SLAMPoint.swift
//  DepthViz
//
//  Point structure matching C++ Point3D from ISLAMSystem.h
//

import Foundation
import simd

/// Matches C++ Point3D struct:
/// struct Point3D {
///     float x, y, z;
///     float intensity;
///     uint8_t r, g, b;  // RGB color from camera
///     double timestamp;
/// };
struct SLAMPoint {
    var x: Float
    var y: Float
    var z: Float
    var intensity: Float
    var r: UInt8
    var g: UInt8
    var b: UInt8
    private var _padding: UInt8 = 0  // Alignment padding before double
    private var _padding2: UInt32 = 0 // Additional padding for 8-byte alignment
    var timestamp: Double

    init(x: Float, y: Float, z: Float, intensity: Float, r: UInt8 = 128, g: UInt8 = 128, b: UInt8 = 128, timestamp: Double = 0) {
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity
        self.r = r
        self.g = g
        self.b = b
        self.timestamp = timestamp
    }
}

// Type alias for backward compatibility
typealias AppPoint3D = SLAMPoint
