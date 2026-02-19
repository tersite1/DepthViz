//
//  ExperimentConfig.swift
//  DepthViz
//
//  실험용 프리셋. 런타임에서 config 전환하여 ablation 수행.
//

import Foundation

// MARK: - Experiment Preset

enum ExperimentPreset: String, CaseIterable {
    case full           = "DV-SLAM (Full)"
    case noConfidence   = "No Confidence (w=1)"
    case uniformDS      = "Uniform Downsample"
    case sparse100      = "~100 pts"
    case dense3000      = "~3000 pts (all)"
    case arkitBaseline  = "ARKit Baseline"

    var description: String {
        switch self {
        case .full:          return "전체 파이프라인 (Table III/IV 기준)"
        case .noConfidence:  return "confidence weighting 비활성화 (w_i=1)"
        case .uniformDS:     return "CGD → 균일 voxel 다운샘플링"
        case .sparse100:     return "극단적 희소: ~100 pts/frame"
        case .dense3000:     return "전체 포인트: ~3000 pts/frame"
        case .arkitBaseline: return "ARKit VIO only, 후처리 없음"
        }
    }
}

// MARK: - Experiment Config

struct ExperimentConfig {
    let preset: ExperimentPreset

    // --- 알고리즘 ---
    let algorithm: SLAMAlgorithm

    // --- 포인트 수 제어 ---
    let numGridPoints: Int              // GPU 그리드 포인트 수
    let rotationThresholdDeg: Float     // 카메라 회전 임계값 (도)
    let translationThresholdM: Float    // 카메라 이동 임계값 (m)

    // --- Confidence ---
    let confidenceLevel: ConfidenceLevel
    let useConfidenceWeighting: Bool    // ICP에서 w_conf 적용 여부

    // --- 후처리 ---
    let enableSurfaceThinning: Bool
    let enableOutlierRemoval: Bool
    let exportVoxelSize: Float          // mm 단위

    // --- 메타데이터 ---
    let tag: String                     // 파일명에 붙을 태그
}

// MARK: - Presets

extension ExperimentConfig {

    /// Table III/IV 기준선: 전체 파이프라인
    static let full = ExperimentConfig(
        preset: .full,
        algorithm: .depthViz,
        numGridPoints: 4096,
        rotationThresholdDeg: 2.0,
        translationThresholdM: 0.015,
        confidenceLevel: .high,
        useConfidenceWeighting: true,
        enableSurfaceThinning: true,
        enableOutlierRemoval: true,
        exportVoxelSize: 0.012,
        tag: "full"
    )

    /// Ablation: confidence weighting 끔 (w_i = 1 for all)
    static let noConfidence = ExperimentConfig(
        preset: .noConfidence,
        algorithm: .depthViz,
        numGridPoints: 4096,
        rotationThresholdDeg: 2.0,
        translationThresholdM: 0.015,
        confidenceLevel: .low,           // 전부 수용
        useConfidenceWeighting: false,    // w=1 uniform
        enableSurfaceThinning: true,
        enableOutlierRemoval: true,
        exportVoxelSize: 0.012,
        tag: "no_conf"
    )

    /// Ablation: CGD → 균일 다운샘플링 (confidence 기반 필터링 없이)
    static let uniformDS = ExperimentConfig(
        preset: .uniformDS,
        algorithm: .depthViz,
        numGridPoints: 4096,
        rotationThresholdDeg: 2.0,
        translationThresholdM: 0.015,
        confidenceLevel: .low,           // 전부 수용 (필터링 없음)
        useConfidenceWeighting: true,    // weighting은 유지
        enableSurfaceThinning: true,
        enableOutlierRemoval: true,
        exportVoxelSize: 0.012,
        tag: "uniform_ds"
    )

    /// Ablation: ~100 pts/frame (극단적 희소)
    static let sparse100 = ExperimentConfig(
        preset: .sparse100,
        algorithm: .depthViz,
        numGridPoints: 256,              // ~100 usable pts after filtering
        rotationThresholdDeg: 2.0,
        translationThresholdM: 0.015,
        confidenceLevel: .high,
        useConfidenceWeighting: true,
        enableSurfaceThinning: true,
        enableOutlierRemoval: true,
        exportVoxelSize: 0.012,
        tag: "sparse100"
    )

    /// Ablation: ~3000 pts/frame (전체 포인트, stride=1)
    static let dense3000 = ExperimentConfig(
        preset: .dense3000,
        algorithm: .depthViz,
        numGridPoints: 8192,             // 최대 밀도
        rotationThresholdDeg: 2.0,
        translationThresholdM: 0.015,
        confidenceLevel: .high,
        useConfidenceWeighting: true,
        enableSurfaceThinning: true,
        enableOutlierRemoval: true,
        exportVoxelSize: 0.012,
        tag: "dense3000"
    )

    /// Table III baseline: ARKit VIO + 기본 다운샘플링만
    static let arkitBaseline = ExperimentConfig(
        preset: .arkitBaseline,
        algorithm: .arkit,
        numGridPoints: 2048,
        rotationThresholdDeg: 5.0,
        translationThresholdM: 0.03,
        confidenceLevel: .low,           // 전부 수용
        useConfidenceWeighting: false,
        enableSurfaceThinning: false,
        enableOutlierRemoval: false,
        exportVoxelSize: 0.020,
        tag: "arkit"
    )
}

// MARK: - Experiment Manager

class ExperimentManager {
    static let shared = ExperimentManager()

    /// 현재 활성 실험 config (nil이면 일반 모드)
    var activeConfig: ExperimentConfig?

    /// 실험 모드 활성화 여부
    var isExperimentMode: Bool { activeConfig != nil }

    /// 현재 실험의 numGridPoints (nil이면 ScanSettings 기본값 사용)
    var overrideGridPoints: Int? { activeConfig?.numGridPoints }

    /// 현재 실험의 rotation threshold (nil이면 ScanSettings 기본값 사용)
    var overrideRotationDeg: Float? { activeConfig?.rotationThresholdDeg }

    /// 현재 실험의 translation threshold (nil이면 ScanSettings 기본값 사용)
    var overrideTranslationM: Float? { activeConfig?.translationThresholdM }

    /// config 적용
    func activate(_ config: ExperimentConfig) {
        activeConfig = config
        // ScanSettings도 같이 업데이트
        ScanSettings.shared.algorithm = config.algorithm
        ScanSettings.shared.confidenceLevel = config.confidenceLevel
        print("🧪 실험 모드: \(config.preset.rawValue) [\(config.tag)]")
    }

    /// 실험 모드 해제
    func deactivate() {
        activeConfig = nil
        print("🧪 실험 모드 해제")
    }

    /// 현재 config의 메타데이터 (파일 저장용)
    func metadataJSON() -> [String: Any] {
        guard let c = activeConfig else { return ["mode": "normal"] }
        return [
            "preset": c.preset.rawValue,
            "tag": c.tag,
            "algorithm": c.algorithm.rawValue,
            "numGridPoints": c.numGridPoints,
            "rotationThresholdDeg": c.rotationThresholdDeg,
            "translationThresholdM": c.translationThresholdM,
            "confidenceLevel": c.confidenceLevel.rawValue,
            "useConfidenceWeighting": c.useConfidenceWeighting,
            "enableSurfaceThinning": c.enableSurfaceThinning,
            "enableOutlierRemoval": c.enableOutlierRemoval,
            "exportVoxelSize": c.exportVoxelSize
        ]
    }

    /// 메타데이터를 JSON 파일로 저장
    func saveMetadata(to directory: URL, sequenceID: String) {
        let meta = metadataJSON()
        let filename = "experiment_\(activeConfig?.tag ?? "normal")_\(sequenceID).json"
        let url = directory.appendingPathComponent(filename)
        if let data = try? JSONSerialization.data(withJSONObject: meta, options: .prettyPrinted) {
            try? data.write(to: url)
            print("📝 실험 메타데이터 저장: \(filename)")
        }
    }
}
