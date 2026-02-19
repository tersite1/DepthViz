//
//  ScanSettings.swift
//  DepthViz
//
//  Unified scan settings: algorithms, confidence, distance, file format.
//

import Foundation
import Combine

// MARK: - SLAM Algorithm

public enum SLAMAlgorithm: String, CaseIterable, Codable {
    case depthViz = "DepthViz"
    case arkit = "ARKit"

    public var description: String {
        switch self {
        case .depthViz: return "LIO Optimization, Bundle & Discard"
        case .arkit: return "ARKit pose as-is, confidence filtering only."
        }
    }

    public var badge: String {
        switch self {
        case .depthViz: return "Mobile-LIO"
        case .arkit: return "ARKit"
        }
    }
}

// MARK: - Confidence Level

public enum ConfidenceLevel: String, CaseIterable, Codable {
    case low = "Low"
    case medium = "Medium"
    case high = "High"

    public var confidenceValue: Int {
        switch self {
        case .low: return 0
        case .medium: return 1
        case .high: return 2
        }
    }

    /// Threshold used by the Metal shader (uniforms.confidenceThreshold).
    /// ARKit confidence: 0=low, 1=medium, 2=high.
    public var shaderThreshold: Float {
        switch self {
        case .low:    return 0.5   // admit medium(1) + high(2) — 무난한 품질
        case .medium: return 1.5   // admit high(2) only — 엄격한 품질
        case .high:   return 1.5   // admit high(2) only + depth edge rejection — 최고 품질
        }
    }

    /// Depth edge rejection threshold (meters).
    /// Rejects points where neighboring depth differs sharply — multipath artifact hotspot.
    /// 0 = disabled.
    public var depthEdgeThreshold: Float {
        switch self {
        case .low:    return 0      // 비활성화
        case .medium: return 0.10   // 10cm — 극심한 depth edge만 거부
        case .high:   return 0.04   // 4cm — 공격적 edge 거부 (multipath 최소화)
        }
    }

    /// Temporal voting threshold: number of observations in same voxel before accepting.
    /// Ghost points jitter between voxels (multipath instability) → never confirmed.
    /// Real surfaces stay stable → quickly confirmed.
    public var temporalThreshold: Int {
        switch self {
        case .low:    return 1   // 즉시 수용 (기존 동작)
        case .medium: return 2   // 2회 관측 확인
        case .high:   return 3   // 3회 관측 확인 (TSDF식 temporal voting)
        }
    }

    /// Value stored for SLAMService (0.0-1.0 range, scaled by 2x in engine).
    public var slamThreshold: Float {
        switch self {
        case .low:    return 0.0
        case .medium: return 0.5
        case .high:   return 1.0
        }
    }
}

// MARK: - Distance Limit

public enum DistanceLimit: String, CaseIterable, Codable {
    case oneM = "1m"
    case twoM = "2m"
    case threeM = "3m"
    case fourM = "4m"
    case fiveM = "5m"
    case noLimit = "No Limit"

    public var distanceValue: Float {
        switch self {
        case .oneM:    return 1.0
        case .twoM:    return 2.0
        case .threeM:  return 3.0
        case .fourM:   return 4.0
        case .fiveM:   return 5.0
        case .noLimit: return .greatestFiniteMagnitude
        }
    }

    public var displayName: String {
        return self.rawValue
    }
}

// MARK: - File Format

public enum FileFormat: String, CaseIterable, Codable {
    case plyAscii = "PLY ASCII"
    case plyBinary = "PLY Binary"
    case xyz = "XYZ"
    case las = "LAS"

    public var fileExtension: String {
        switch self {
        case .plyAscii, .plyBinary: return "ply"
        case .xyz: return "xyz"
        case .las: return "las"
        }
    }

    public var displayName: String {
        return self.rawValue
    }
}

// MARK: - Settings Manager

public class ScanSettings: ObservableObject {
    public static let shared = ScanSettings()

    // UserDefaults keys
    private static let algorithmKey = "ScanAlgorithm"
    private static let confidenceKey = "scan_confidence_level"
    private static let confidenceThresholdKey = "ConfidenceThreshold"
    private static let distanceKey = "scan_distance_limit"
    private static let slamDistanceKey = "ScanDistanceLimit"
    private static let fileFormatKey = "scan_file_format"

    @Published public var algorithm: SLAMAlgorithm {
        didSet {
            UserDefaults.standard.set(algorithm.rawValue, forKey: Self.algorithmKey)
        }
    }

    @Published public var confidenceLevel: ConfidenceLevel {
        didSet {
            UserDefaults.standard.set(confidenceLevel.rawValue, forKey: Self.confidenceKey)
            // Also save numeric threshold for SLAMService (reads "ConfidenceThreshold")
            UserDefaults.standard.set(confidenceLevel.slamThreshold, forKey: Self.confidenceThresholdKey)
        }
    }

    @Published public var distanceLimit: DistanceLimit {
        didSet {
            UserDefaults.standard.set(distanceLimit.rawValue, forKey: Self.distanceKey)
            // Also save numeric value for SLAMService (reads "ScanDistanceLimit")
            UserDefaults.standard.set(distanceLimit.distanceValue, forKey: Self.slamDistanceKey)
        }
    }

    @Published public var fileFormat: FileFormat {
        didSet {
            UserDefaults.standard.set(fileFormat.rawValue, forKey: Self.fileFormatKey)
        }
    }

    // Additional parameters
    @Published public var voxelSize: Float = 0.2
    @Published public var enableRelocalization: Bool = false

    private init() {
        // Load algorithm
        let savedAlgo = UserDefaults.standard.string(forKey: Self.algorithmKey) ?? SLAMAlgorithm.depthViz.rawValue
        self.algorithm = SLAMAlgorithm(rawValue: savedAlgo) ?? .depthViz

        // Load confidence level
        if let savedConf = UserDefaults.standard.string(forKey: Self.confidenceKey),
           let level = ConfidenceLevel(rawValue: savedConf) {
            self.confidenceLevel = level
        } else {
            self.confidenceLevel = .high
        }

        // Load distance limit
        if let savedDist = UserDefaults.standard.string(forKey: Self.distanceKey),
           let limit = DistanceLimit(rawValue: savedDist) {
            self.distanceLimit = limit
        } else {
            self.distanceLimit = .threeM
        }

        // Load file format
        if let savedFormat = UserDefaults.standard.string(forKey: Self.fileFormatKey),
           let format = FileFormat(rawValue: savedFormat) {
            self.fileFormat = format
        } else {
            self.fileFormat = .plyAscii
        }
    }
}
