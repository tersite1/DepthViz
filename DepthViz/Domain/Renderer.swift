//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//

import Foundation
import Metal
import MetalKit
import ARKit
import simd
import CoreMotion
import Combine

struct IMUDisplayData {
    let roll: Float, pitch: Float, yaw: Float    // degrees
    let ax: Float, ay: Float, az: Float          // g
    let gx: Float, gy: Float, gz: Float          // rad/s
}

/// IMU 로그 (CSV 내보내기용)
struct IMULogEntry {
    let timestamp: Double   // 녹화 시작부터 경과 시간 (초)
    let roll: Float, pitch: Float, yaw: Float
    let ax: Float, ay: Float, az: Float
    let gx: Float, gy: Float, gz: Float
}

/// 트라젝토리 로그 (CSV 내보내기용)
struct TrajectoryLogEntry {
    let timestamp: Double
    let x: Float, y: Float, z: Float
}

/// SLAM 포즈 보정 엔트리: 특정 포인트 인덱스 시점에서의 ARKit↔SLAM 포즈 쌍
struct PoseCorrectionEntry {
    let pointIndex: Int
    let arkitPose: simd_float4x4
    let slamPose: simd_float4x4
}

final class Renderer: NSObject, SLAMDelegate {
    // Maximum number of points we store in the point cloud
    // 시뮬레이터: 2백만, 실제 기기: 8백만
    #if targetEnvironment(simulator)
    private var maxPoints = 2000000
    #else
    private var maxPoints = 3000000
    #endif
    // Number of sample points on the grid
    private var numGridPoints: Int {
        ScanSettings.shared.algorithm == .depthViz ? 8192 : 2048
    }
    // Particle's size in pixels (small for detailed visualization)
    private let particleSize: Float = 1.4  // 화면에 표시될 점 크기
    // We only use portrait orientation in this app
    private let orientation = UIInterfaceOrientation.portrait
    // Camera's threshold values for detecting when the camera moves so that we can accumulate the points
    // DV-SLAM: 더 공격적 취득 (SLAM이 포즈 보정하므로 자주 샘플링해도 안전)
    // ARKit: 보수적 취득
    private var cameraRotationThreshold: Float {
        ScanSettings.shared.algorithm == .depthViz
            ? cos(2.0 * .degreesToRadian)   // DV-SLAM: 2도 회전
            : cos(5.0 * .degreesToRadian)   // ARKit: 5도 회전
    }
    private var cameraTranslationThreshold: Float {
        ScanSettings.shared.algorithm == .depthViz
            ? pow(0.015, 2)   // DV-SLAM: 1.5cm 이동
            : pow(0.03, 2)    // ARKit: 3cm 이동
    }
    // The max number of command buffers in flight
    private let maxInFlightBuffers = 3
    
    private lazy var rotateToARCamera = Self.makeRotateToARCameraMatrix(orientation: orientation)
    private let session: ARSession
    var arSession: ARSession { session }
    
    // Debug counters
    private var frameNilCount = 0
    private var firstFrameReceived = false
    private var cameraBackgroundRendered = false
    
    // Metal objects and textures
    private let device: MTLDevice
    private let library: MTLLibrary
    var renderDestination: RenderDestinationProvider
    private let relaxedStencilState: MTLDepthStencilState
    private let depthStencilState: MTLDepthStencilState
    private var commandQueue: MTLCommandQueue
    private lazy var unprojectPipelineState = makeUnprojectionPipelineState()!
    private lazy var rgbPipelineState = makeRGBPipelineState()!
    private lazy var particlePipelineState = makeParticlePipelineState()!
    // Texture manager for handling all Metal textures
    private lazy var textureManager = MetalTextureManager(device: device)
    
    // Multi-buffer rendering pipeline
    private let inFlightSemaphore: DispatchSemaphore
    private var currentBufferIndex = 0
    
    // The current viewport size
    private var viewportSize = CGSize()
    // The grid of sample points (알고리즘에 따라 numGridPoints가 다르므로 재생성 가능)
    private lazy var gridPointsBuffer = MetalBuffer<Float2>(device: device,
                                                            array: makeGridPoints(),
                                                            index: kGridPoints.rawValue, options: [])

    /// 그리드 포인트 버퍼를 현재 알고리즘 설정에 맞게 재생성
    private func rebuildGridPointsBuffer() {
        gridPointsBuffer = MetalBuffer<Float2>(device: device,
                                               array: makeGridPoints(),
                                               index: kGridPoints.rawValue, options: [])
        print("📐 그리드 포인트 재생성: \(gridPointsBuffer.count)개 (\(ScanSettings.shared.algorithm.badge) 모드)")
    }
    
    private var rgbUniformsBuffers = [MetalBuffer<RGBUniforms>]()
    // Point Cloud buffer
    // This is not the point cloud data, but some parameters
    private lazy var pointCloudUniforms: PointCloudUniforms = {
        var uniforms = PointCloudUniforms()
        uniforms.maxPoints = Int32(self.maxPoints)
        uniforms.confidenceThreshold = ScanSettings.shared.confidenceLevel.shaderThreshold
        uniforms.particleSize = particleSize
        uniforms.cameraResolution = Float2(1920, 1440)
        uniforms.voxelSize = voxelSize
        uniforms.voxelGridSize = Int32(voxelGridSize)
        uniforms.depthEdgeThreshold = ScanSettings.shared.confidenceLevel.depthEdgeThreshold
        uniforms.temporalThreshold = Int32(ScanSettings.shared.confidenceLevel.temporalThreshold)
        let initDist = Float(UserDefaults.standard.double(forKey: "ScanDistanceLimit"))
        uniforms.maxDistance = initDist > 0 ? initDist : ScanSettings.shared.distanceLimit.distanceValue
        return uniforms
    }()
    private var pointCloudUniformsBuffers = [MetalBuffer<PointCloudUniforms>]()
    // Particles buffer
    // Saves the point cloud data, filled by unprojectVertex func in Shaders.metal
    var particlesBuffer: MetalBuffer<ParticleUniforms>  // internal로 변경 (ScanPreviewVC에서 접근 필요)
    private var currentPointIndex = 0
    @Published private(set) var currentPointCount = 0
    // Export 무효화용 세대 카운터 (reset 시 증가 → 이전 export 결과 무시)
    private var exportGeneration = 0
    // Voxel occupancy grid for deduplication
    private var voxelGridSize: Int {
        ScanSettings.shared.algorithm == .depthViz ? (1 << 24) : (1 << 21)  // 16M vs 2M
    }
    private var voxelSize: Float {
        ScanSettings.shared.algorithm == .depthViz ? 0.005 : 0.020  // 5mm vs 20mm
    }
    // 복셀 그리드 주기적 초기화 (격자 패턴 방지)
    private var voxelResetCounter = 0
    // DV-SLAM: 60프레임 (temporal voting 관측 시간 확보)
    // ARKit: 30프레임
    private var voxelResetInterval: Int {
        ScanSettings.shared.algorithm == .depthViz ? 60 : 30
    }
    private var voxelGridBuffer: MTLBuffer!
    /// 스캔 FOV 비율 (0.0~1.0). 1.0=전체 LiDAR FOV, 0.5=중앙 50%만 스캔
    private var scanFOVScale: Float = 0.6
    // Camera data
    private var cameraResolution: Float2?
    private var lastCameraTransform: simd_float4x4?
    private var startCameraTransform: simd_float4x4?  // 스캔 시작 시점의 카메라 위치
    // DV-SLAM refined pose (set by didUpdatePose delegate callback)
    private var latestSLAMPose: simd_float4x4?
    // DV-SLAM 진단 카운터
    private var slamPoseUpdateCount = 0
    private var slamFrameFeedCount = 0
    private var imuSampleCount = 0
    private var slamDiagTimer: Timer?
    // IMU for DV-SLAM (LIO requires high-frequency inertial data)
    private let motionManager = CMMotionManager()
    private let imuQueue = OperationQueue()
    
    var confidenceThreshold: Float = 0.5 {
        didSet {
            pointCloudUniforms.confidenceThreshold = confidenceThreshold
        }
    }
    
    // Whether recording is on
    var isRecording = false {
        didSet {
            if isRecording != oldValue {
                print("🔴 isRecording: \(oldValue) → \(isRecording)")
                Thread.callStackSymbols.prefix(6).forEach { print("   \($0)") }
            }
            if isRecording && !oldValue {
                // 녹화 시작 시 현재 카메라 위치 저장
                if let currentFrame = session.currentFrame {
                    startCameraTransform = currentFrame.camera.transform
                    print("📍 스캔 시작 위치 저장: \(startCameraTransform!.columns.3)")
                }
            }
        }
    }
    // Pick every n frames (~1/sampling frequency)
    public var pickFrames = 24 // 정지 시에도 주기적으로 누적 (감도 낮춤)
    public var currentFrameIndex = 0;
    // Task delegate for informing ViewController of tasks
    public weak var delegate: TaskDelegate?
    
    @Published private(set) var lidarRawStringData: String?
    @Published private(set) var lidarRawData: Data?
    @Published var imuDisplayData: IMUDisplayData?
    private var clearing: Bool = false

    // 트라젝토리 기록 (카메라 이동 경로)
    private var trajectoryPoses: [SIMD3<Float>] = []
    // IMU + 트라젝토리 로그 (CSV 내보내기용)
    private var imuLog: [IMULogEntry] = []
    private var trajectoryLog: [TrajectoryLogEntry] = []
    private var recordingStartTime: Date?
    // SLAM 포즈 보정 로그: 각 키프레임에서 (ARKit포즈, SLAM포즈) 쌍 기록
    // 후처리에서 GPU 포인트의 ARKit 드리프트를 SLAM으로 보정하는 데 사용
    private var poseCorrectionLog: [PoseCorrectionEntry] = []

    /// 트라젝토리 포인트 반환
    func getTrajectoryPoints() -> [SIMD3<Float>] {
        return trajectoryPoses
    }

    /// IMU 로그 반환 (CSV용)
    func getIMULog() -> [IMULogEntry] {
        return imuLog
    }

    /// 트라젝토리 로그 반환 (CSV용)
    func getTrajectoryLog() -> [TrajectoryLogEntry] {
        return trajectoryLog
    }

    private static let absoluteMaxPoints = 50_000_000  // 50M hard cap

    private func ensureParticleCapacity(additionalPoints: Int) {
        let required = currentPointIndex + additionalPoints
        guard required < maxPoints else {
            var newCapacity = maxPoints
            while required >= newCapacity && newCapacity < Self.absoluteMaxPoints {
                newCapacity = Int(Double(newCapacity) * 1.5)
            }
            newCapacity = min(newCapacity, Self.absoluteMaxPoints)
            guard newCapacity > maxPoints else { return }  // At hard cap
            resizeParticleBuffer(to: newCapacity)
            return
        }
    }
    
    private func resizeParticleBuffer(to newCapacity: Int) {
        let newBuffer = MetalBuffer<ParticleUniforms>(device: device, count: newCapacity, index: kParticleUniforms.rawValue)
        particlesBuffer = newBuffer
        maxPoints = newCapacity
        pointCloudUniforms.maxPoints = Int32(newCapacity)
    }
    
    init(session: ARSession, metalDevice device: MTLDevice, renderDestination: RenderDestinationProvider) {
        print("🔧 Renderer 초기화 시작...")
        print("   - 최대 포인트 수: \(maxPoints)")
        #if targetEnvironment(simulator)
        print("   - 실행 환경: 시뮬레이터")
        #else
        print("   - 실행 환경: 실제 기기")
        #endif
        
        self.session = session
        self.device = device
        self.renderDestination = renderDestination
        // MTLibrary를 생성
        self.library = device.makeDefaultLibrary()!
        self.commandQueue = device.makeCommandQueue()!
        
        print("🔧 Metal 버퍼 생성 시작...")
        // initialize our buffers
        for i in 0 ..< maxInFlightBuffers {
            print("   - RGB/PointCloud 버퍼 \(i+1)/\(maxInFlightBuffers)")
            rgbUniformsBuffers.append(.init(device: device, count: 1, index: 0))
            pointCloudUniformsBuffers.append(.init(device: device, count: 1, index: kPointCloudUniforms.rawValue))
        }
        print("🔧 Particles 버퍼 생성 중... (가장 큰 버퍼)")
        particlesBuffer = .init(device: device, count: maxPoints, index: kParticleUniforms.rawValue)

        // Voxel occupancy grid (최대 크기인 Mobile-LIO 기준으로 할당)
        let maxVoxelGridSize = 1 << 24  // 16M (Mobile-LIO용, ARKit은 일부만 사용)
        voxelGridBuffer = device.makeBuffer(length: maxVoxelGridSize * MemoryLayout<UInt32>.size, options: .storageModeShared)!

        // rbg does not need to read/write depth
        let relaxedStateDescriptor = MTLDepthStencilDescriptor()
        relaxedStencilState = device.makeDepthStencilState(descriptor: relaxedStateDescriptor)!
        
        // setup depth test for point cloud
        let depthStateDescriptor = MTLDepthStencilDescriptor()
        depthStateDescriptor.depthCompareFunction = .lessEqual
        depthStateDescriptor.isDepthWriteEnabled = true
        depthStencilState = device.makeDepthStencilState(descriptor: depthStateDescriptor)!
        
        inFlightSemaphore = DispatchSemaphore(value: maxInFlightBuffers)
        
        print("✅ Renderer 초기화 완료")
    }
    
    /// 사용자 설정 적용 (ARSession이 시작된 후, 녹화 시작 전에 호출)
    func applySettings() {
        let settings = ScanSettings.shared
        // Confidence → 셰이더 uniform (particleVertex에서 visibility 판정용)
        confidenceThreshold = settings.confidenceLevel.shaderThreshold
        // Distance limit → 셰이더에서 초과 거리 즉시 거부
        // 설정 UI가 슬라이더(float)를 "ScanDistanceLimit"에 직접 저장하므로 UserDefaults에서 읽기
        let savedDist = Float(UserDefaults.standard.double(forKey: "ScanDistanceLimit"))
        pointCloudUniforms.maxDistance = savedDist > 0 ? savedDist : settings.distanceLimit.distanceValue
        // Depth edge rejection + temporal voting (depth bleeding / ghost point 방지)
        pointCloudUniforms.depthEdgeThreshold = settings.confidenceLevel.depthEdgeThreshold
        pointCloudUniforms.temporalThreshold = Int32(settings.confidenceLevel.temporalThreshold)
        // 알고리즘에 맞게 그리드 포인트 버퍼 재생성 (DV-SLAM: 8192, ARKit: 2048)
        rebuildGridPointsBuffer()
        let distDisplay = pointCloudUniforms.maxDistance >= 999 ? "∞" : String(format: "%.1fm", pointCloudUniforms.maxDistance)
        print("⚙️ 설정 적용: 알고리즘=\(settings.algorithm.badge), 신뢰도=\(settings.confidenceLevel.rawValue)(shader≥\(confidenceThreshold), edge=\(Int(pointCloudUniforms.depthEdgeThreshold*100))cm, temporal=\(pointCloudUniforms.temporalThreshold)회), 거리=\(distDisplay), 복셀=\(voxelSize*1000)mm")
    }
    
    /// DV-SLAM용 IMU 데이터 공급 시작 (100Hz — LIO 필수)
    func startIMUForSLAM() {
        guard motionManager.isDeviceMotionAvailable else {
            print("❌ [DV-SLAM] DeviceMotion 사용 불가 — IMU 없이 LIO 불가능!")
            return
        }
        // 진단 카운터 초기화
        slamPoseUpdateCount = 0
        slamFrameFeedCount = 0
        imuSampleCount = 0

        imuQueue.maxConcurrentOperationCount = 1
        imuQueue.qualityOfService = .userInteractive
        motionManager.deviceMotionUpdateInterval = 1.0 / 100.0
        recordingStartTime = Date()
        motionManager.startDeviceMotionUpdates(to: imuQueue) { [weak self] motion, error in
            guard let self = self, let motion = motion else { return }
            self.imuSampleCount += 1
            SLAMService.sharedInstance().processIMUData(motion)

            let att = motion.attitude
            let acc = motion.userAcceleration
            let rot = motion.rotationRate

            // IMU 로그 축적 (10Hz — CSV용)
            if self.imuSampleCount % 10 == 0 {
                let elapsed = Date().timeIntervalSince(self.recordingStartTime ?? Date())
                self.imuLog.append(IMULogEntry(
                    timestamp: elapsed,
                    roll: Float(att.roll * 180 / Double.pi),
                    pitch: Float(att.pitch * 180 / Double.pi),
                    yaw: Float(att.yaw * 180 / Double.pi),
                    ax: Float(acc.x), ay: Float(acc.y), az: Float(acc.z),
                    gx: Float(rot.x), gy: Float(rot.y), gz: Float(rot.z)
                ))

                // UI 업데이트
                DispatchQueue.main.async {
                    self.imuDisplayData = IMUDisplayData(
                        roll: Float(att.roll * 180 / Double.pi),
                        pitch: Float(att.pitch * 180 / Double.pi),
                        yaw: Float(att.yaw * 180 / Double.pi),
                        ax: Float(acc.x), ay: Float(acc.y), az: Float(acc.z),
                        gx: Float(rot.x), gy: Float(rot.y), gz: Float(rot.z)
                    )
                }
            }
        }

        // 2초마다 진단 로그 출력
        DispatchQueue.main.async { [weak self] in
            self?.slamDiagTimer?.invalidate()
            self?.slamDiagTimer = Timer.scheduledTimer(withTimeInterval: 2.0, repeats: true) { [weak self] _ in
                guard let self = self else { return }
                let poseOK = self.latestSLAMPose != nil
                let correction: String
                let rotDiff: String
                if let slam = self.latestSLAMPose, let arkit = self.lastCameraTransform {
                    let d = distance(slam.columns.3, arkit.columns.3)
                    correction = String(format: "%.3fm", d)
                    // 회전 차이
                    let rel = slam * arkit.inverse
                    let trace = rel.columns.0.x + rel.columns.1.y + rel.columns.2.z
                    let angle = acos(max(-1, min(1, (trace - 1) / 2))) * 180 / .pi
                    rotDiff = String(format: "%.1f°", angle)
                } else {
                    correction = "N/A"
                    rotDiff = "N/A"
                }
                print("📊 [DV-SLAM] IMU:\(self.imuSampleCount) | 프레임:\(self.slamFrameFeedCount) | 포즈:\(self.slamPoseUpdateCount) | 보정쌍:\(self.poseCorrectionLog.count) | 위치차:\(correction) 회전차:\(rotDiff) | 포인트:\(self.currentPointCount)")
            }
        }

        print("✅ [DV-SLAM] IMU 공급 시작 (100Hz)")
    }

    /// DV-SLAM용 IMU 데이터 공급 중지
    func stopIMUForSLAM() {
        motionManager.stopDeviceMotionUpdates()
        slamDiagTimer?.invalidate()
        slamDiagTimer = nil
        // 최종 통계
        print("📊 [DV-SLAM 최종] IMU:\(imuSampleCount)회 | AR프레임:\(slamFrameFeedCount)회 | 포즈업데이트:\(slamPoseUpdateCount)회")
    }

    func drawRectResized(size: CGSize) {
        self.viewportSize = size
    }
    
    /// 스캔 시작 시점의 카메라 위치 반환
    func getStartCameraTransform() -> simd_float4x4? {
        return startCameraTransform
    }
    // Texture update methods removed - now handled by MetalTextureManager
    
    private func update(frame: ARFrame) {
        // 첫 프레임에서 카메라 정보 초기화
        if cameraResolution == nil {
            let width = Float(frame.camera.imageResolution.width)
            let height = Float(frame.camera.imageResolution.height)
            cameraResolution = Float2(width, height)
            pointCloudUniforms.cameraResolution = cameraResolution!
            lastCameraTransform = frame.camera.transform
            print("✅ 카메라 해상도 초기화: \(width)x\(height)")
        }
        
        // frame dependent info
        let camera = frame.camera
        let cameraTransform = camera.transform
        let cameraIntrinsicsInversed = camera.intrinsics.inverse
        let viewMatrix = camera.viewMatrix(for: orientation)
        let viewMatrixInversed = viewMatrix.inverse
        let projectionMatrix = camera.projectionMatrix(for: orientation, viewportSize: viewportSize, zNear: 0.001, zFar: 0)
        pointCloudUniforms.viewProjectionMatrix = projectionMatrix * viewMatrix
        // 항상 ARKit 포즈 사용 (안정적)
        // DV-SLAM 보정은 후처리에서만 적용 — 실시간 적용 시 0.5~0.8m 드리프트 발생
        pointCloudUniforms.localToWorld = viewMatrixInversed * rotateToARCamera
        pointCloudUniforms.cameraIntrinsicsInversed = cameraIntrinsicsInversed
        
        let cameraTranslation = cameraTransform.columns.3

        // 트라젝토리 기록 (녹화 중일 때)
        if isRecording {
            let pos = SIMD3<Float>(cameraTranslation.x, cameraTranslation.y, cameraTranslation.z)
            trajectoryPoses.append(pos)
            let elapsed = Date().timeIntervalSince(recordingStartTime ?? Date())
            trajectoryLog.append(TrajectoryLogEntry(timestamp: elapsed, x: pos.x, y: pos.y, z: pos.z))
        }

        // RGB uniforms 업데이트 (카메라 피드 렌더링용)
        updateCameraUniforms(frame: frame)
    }
    
    private func updateCameraUniforms(frame: ARFrame) {
        // 카메라 배경 비활성화 — 포인트 클라우드만 검은 배경에 렌더링
        var uniforms = rgbUniformsBuffers[currentBufferIndex][0]
        uniforms.viewRatio = Float(viewportSize.width / viewportSize.height)
        uniforms.radius = 0  // radius 0 = 카메라 배경 숨김
        rgbUniformsBuffers[currentBufferIndex][0] = uniforms
    }
    
    func draw() {
        autoreleasepool {
            guard self.clearing == false else { return }
            
            guard let currentFrame = session.currentFrame,
                  let renderDescriptor = renderDestination.currentRenderPassDescriptor,
                  let drawable = renderDestination.currentDrawable,
                  let commandBuffer = commandQueue.makeCommandBuffer(),
                  let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderDescriptor) else {
                // 프레임이 nil인 경우 - 처음 몇 번만 로그
                if session.currentFrame == nil {
                    frameNilCount += 1
                    if frameNilCount <= 5 || frameNilCount % 30 == 0 {
                        print("⚠️ session.currentFrame is nil (\(frameNilCount)번째) - ARSession 초기화 중...")
                    }
                }
                return
            }
        
        // 첫 프레임 도착 로그
        if !firstFrameReceived {
            print("✅ 첫 프레임 도착! (\(frameNilCount)번의 nil 후)")
            firstFrameReceived = true
        }
        
        _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        commandBuffer.addCompletedHandler { [weak self] commandBuffer in
            if let self = self {
                self.inFlightSemaphore.signal()
            }
        }
        
        // handle buffer rotating FIRST so we write & read from the same buffer
        currentBufferIndex = (currentBufferIndex + 1) % maxInFlightBuffers

        // DV-SLAM: AR 프레임을 SLAM 엔진에 공급 (update 전에 호출하여 최신 포즈 확보)
        if isRecording && ScanSettings.shared.algorithm == .depthViz {
            slamFrameFeedCount += 1
            SLAMService.sharedInstance().processARFrame(currentFrame)
            // 첫 프레임 확인
            if slamFrameFeedCount == 1 {
                print("✅ [DV-SLAM] 첫 AR프레임 SLAM 엔진에 전달됨")
            }
        }

        // update frame data
        update(frame: currentFrame)
        textureManager.updateCapturedImageTextures(frame: currentFrame)
        pointCloudUniformsBuffers[currentBufferIndex][0] = pointCloudUniforms
        
        // 1. 카메라 피드를 배경으로 렌더링
        renderEncoder.setDepthStencilState(relaxedStencilState)
        renderEncoder.setRenderPipelineState(rgbPipelineState)
        renderEncoder.setVertexBuffer(rgbUniformsBuffers[currentBufferIndex])
        renderEncoder.setFragmentBuffer(rgbUniformsBuffers[currentBufferIndex])
        
        // RGB pipeline set (one-time log handled by cameraBackgroundRendered flag below)
        
        // 카메라 텍스처 상태 확인
        let hasTextureY = textureManager.capturedImageTextureY != nil
        let hasTextureCbCr = textureManager.capturedImageTextureCbCr != nil
        
        if hasTextureY && hasTextureCbCr,
           let textureY = textureManager.capturedImageTextureY,
           let textureCbCr = textureManager.capturedImageTextureCbCr,
           let metalTextureY = CVMetalTextureGetTexture(textureY),
           let metalTextureCbCr = CVMetalTextureGetTexture(textureCbCr) {
            
            renderEncoder.setFragmentTexture(metalTextureY, index: Int(kTextureY.rawValue))
            renderEncoder.setFragmentTexture(metalTextureCbCr, index: Int(kTextureCbCr.rawValue))
            renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
            
            if !cameraBackgroundRendered {
                print("✅ 카메라 배경 렌더링 시작!")
                print("   - Y 텍스처: \(metalTextureY.width)x\(metalTextureY.height)")
                print("   - CbCr 텍스처: \(metalTextureCbCr.width)x\(metalTextureCbCr.height)")
                print("   - drawPrimitives 호출: triangleStrip, vertexCount=4")
                cameraBackgroundRendered = true
            }
        } else {
            // 카메라 텍스처가 아직 준비 안됨 - 상세 로그
            if !cameraBackgroundRendered {
                print("⚠️ 카메라 텍스처 상태:")
                print("   - textureY: \(hasTextureY ? "있음" : "없음")")
                print("   - textureCbCr: \(hasTextureCbCr ? "있음" : "없음")")
                
                if hasTextureY, let textureY = textureManager.capturedImageTextureY {
                    let metalTexture = CVMetalTextureGetTexture(textureY)
                    print("   - Y MetalTexture: \(metalTexture != nil ? "성공" : "실패")")
                }
                if hasTextureCbCr, let textureCbCr = textureManager.capturedImageTextureCbCr {
                    let metalTexture = CVMetalTextureGetTexture(textureCbCr)
                    print("   - CbCr MetalTexture: \(metalTexture != nil ? "성공" : "실패")")
                }
            }
        }
        
        // 2. 포인트 클라우드 축적 (녹화 중일 때)
        if shouldAccumulate(frame: currentFrame), textureManager.updateDepthTextures(frame: currentFrame) {
            accumulatePoints(frame: currentFrame, commandBuffer: commandBuffer, renderEncoder: renderEncoder)
        }
       
        // 3. 포인트 클라우드를 카메라 피드 위에 렌더링
        renderEncoder.setDepthStencilState(depthStencilState)
        renderEncoder.setRenderPipelineState(particlePipelineState)
        renderEncoder.setVertexBuffer(pointCloudUniformsBuffers[currentBufferIndex])
        renderEncoder.setVertexBuffer(particlesBuffer)
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: currentPointCount)
        renderEncoder.endEncoding()
            
        commandBuffer.present(drawable)
        commandBuffer.commit()
        }
    }
    
    /// 연속 정지 프레임 수 (움직임이 없는 프레임 카운트)
    private var stationaryFrameCount = 0
    /// 정지 판정 후 누적을 중단하기 위한 연속 프레임 수 임계값
    private let stationaryFrameLimit = 5

    private func shouldAccumulate(frame: ARFrame) -> Bool {
        // 녹화 중일 때만 포인트 누적
        guard isRecording else { return false }
        guard frame.sceneDepth != nil else { return false }

        let isDVSLAM = ScanSettings.shared.algorithm == .depthViz

        // 첫 프레임은 무조건 누적
        guard let lastTransform = lastCameraTransform else {
            currentFrameIndex = 0
            stationaryFrameCount = 0
            return true
        }

        let cameraTransform = frame.camera.transform

        let hasRotation = dot(cameraTransform.columns.2, lastTransform.columns.2) <= cameraRotationThreshold
        let hasTranslation = distance_squared(cameraTransform.columns.3, lastTransform.columns.3) >= cameraTranslationThreshold

        let hasMoved = hasRotation || hasTranslation

        if hasMoved {
            stationaryFrameCount = 0
        } else {
            stationaryFrameCount += 1
        }

        if isDVSLAM {
            // DV-SLAM: 임계값은 낮지만 정지 시 누적은 중단 (8프레임 여유)
            if stationaryFrameCount >= 8 {
                return false
            }
            return currentPointCount == 0 || hasMoved
        } else {
            // ARKit: 기존 보수적 로직
            if stationaryFrameCount >= stationaryFrameLimit {
                return false
            }
            let shouldAccumulate = currentPointCount == 0 || hasMoved
            if shouldAccumulate { currentFrameIndex = 0 }
            return shouldAccumulate
        }
    }
    
    private func accumulatePoints(frame: ARFrame, commandBuffer: MTLCommandBuffer, renderEncoder: MTLRenderCommandEncoder) {
        // 주기적 복셀 그리드 초기화 — 5mm 격자 패턴 방지
        // 카메라가 미세하게 이동하므로 다음 사이클에선 다른 위치에 점이 찍힘
        voxelResetCounter += 1
        if voxelResetCounter >= voxelResetInterval {
            voxelResetCounter = 0
            memset(voxelGridBuffer.contents(), 0, voxelGridSize * MemoryLayout<UInt32>.size)
        }

        ensureParticleCapacity(additionalPoints: gridPointsBuffer.count)
        pointCloudUniforms.pointCloudCurrentIndex = Int32(currentPointIndex)
        // GPU 버퍼에 최신 pointCloudCurrentIndex 반영 (draw()의 초기 복사 이후 변경됨)
        pointCloudUniformsBuffers[currentBufferIndex][0] = pointCloudUniforms

        var retainingTextures = [textureManager.capturedImageTextureY, textureManager.capturedImageTextureCbCr,
                                 textureManager.depthTexture, textureManager.confidenceTexture]
        commandBuffer.addCompletedHandler { buffer in
            retainingTextures.removeAll()
        }

        renderEncoder.setDepthStencilState(relaxedStencilState)
        renderEncoder.setRenderPipelineState(unprojectPipelineState)
        renderEncoder.setVertexBuffer(pointCloudUniformsBuffers[currentBufferIndex])
        renderEncoder.setVertexBuffer(particlesBuffer)
        renderEncoder.setVertexBuffer(gridPointsBuffer)
        renderEncoder.setVertexBuffer(voxelGridBuffer, offset: 0, index: Int(kVoxelOccupancy.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.capturedImageTextureY!), index: Int(kTextureY.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.capturedImageTextureCbCr!), index: Int(kTextureCbCr.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.depthTexture!), index: Int(kTextureDepth.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.confidenceTexture!), index: Int(kTextureConfidence.rawValue))
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: gridPointsBuffer.count)
        
        currentPointIndex += gridPointsBuffer.count
        currentPointCount = min(currentPointCount + gridPointsBuffer.count, maxPoints)
        lastCameraTransform = frame.camera.transform
        // 라이브뷰에서도 바로 결과가 보이도록 최소 포인트가 생기면 로그
        if currentPointCount > 0 && currentPointCount % 10000 == 0 {
            print("📍 누적 포인트: \(currentPointCount)")
        }
    }
}

// MARK: - Metal Helpers

private extension Renderer {
    func makeUnprojectionPipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "unprojectVertex") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.isRasterizationEnabled = false
        descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    func makeRGBPipelineState() -> MTLRenderPipelineState? {
        print("🔧 RGB 파이프라인 생성 시작...")
        
        guard let vertexFunction = library.makeFunction(name: "rgbVertex"),
            let fragmentFunction = library.makeFunction(name: "rgbFragment") else {
                print("❌ RGB 셰이더 함수를 찾을 수 없음")
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        
        do {
            let pipelineState = try device.makeRenderPipelineState(descriptor: descriptor)
            print("✅ RGB 파이프라인 생성 성공")
            return pipelineState
        } catch {
            print("❌ RGB 파이프라인 생성 실패: \(error)")
            return nil
        }
    }
    
    func makeParticlePipelineState() -> MTLRenderPipelineState? {
        guard let vertexFunction = library.makeFunction(name: "particleVertex"),
            let fragmentFunction = library.makeFunction(name: "particleFragment") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = true
        descriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        descriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
    }
    
    /// Makes sample points on camera image, also precompute the anchor point for animation
    func makeGridPoints() -> [Float2] {
        // cameraResolution이 아직 설정되지 않은 경우 기본 해상도 사용
        let resolution = cameraResolution ?? Float2(1920, 1440)

        // FOV 제한: 화면 중앙 영역만 샘플링
        let scale = max(0.2, min(1.0, scanFOVScale))
        let marginX = resolution.x * (1.0 - scale) / 2.0
        let marginY = resolution.y * (1.0 - scale) / 2.0
        let scanWidth = resolution.x * scale
        let scanHeight = resolution.y * scale

        let gridArea = scanWidth * scanHeight
        let spacing = sqrt(gridArea / Float(numGridPoints))
        let deltaX = Int(round(scanWidth / spacing))
        let deltaY = Int(round(scanHeight / spacing))

        var points = [Float2]()
        for gridY in 0 ..< deltaY {
            let alternatingOffsetX = Float(gridY % 2) * spacing / 2
            for gridX in 0 ..< deltaX {
                let cameraPoint = Float2(
                    marginX + alternatingOffsetX + (Float(gridX) + 0.5) * spacing,
                    marginY + (Float(gridY) + 0.5) * spacing
                )
                points.append(cameraPoint)
            }
        }

        return points
    }
    
    // Texture management methods removed - now handled by MetalTextureManager
    
    static func cameraToDisplayRotation(orientation: UIInterfaceOrientation) -> Int {
        switch orientation {
        case .landscapeLeft:
            return 180
        case .portrait:
            return 90
        case .portraitUpsideDown:
            return -90
        default:
            return 0
        }
    }
    
    static func makeRotateToARCameraMatrix(orientation: UIInterfaceOrientation) -> matrix_float4x4 {
        // 카메라 intrinsics 좌표계: x-오른쪽, y-아래, z-앞(장면 방향)
        // ARKit view 좌표계: x-오른쪽, y-위, z-뒤(카메라 방향)
        // → Y와 Z 모두 뒤집어야 함 (Apple 공식 샘플 코드와 동일)
        let flipYZ = matrix_float4x4(
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1] )

        let rotationAngle = Float(cameraToDisplayRotation(orientation: orientation)) * .degreesToRadian
        return flipYZ * matrix_float4x4(simd_quaternion(rotationAngle, Float3(0, 0, 1)))
    }
}

extension Renderer {
    /// Save all particles using PointCloudExporter (format-aware, binary support)
    func savePointCloud() {
        delegate?.startMakingPlyFile()
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }

            let format = ScanSettings.shared.fileFormat
            let startTime = CFAbsoluteTimeGetCurrent()
            print("📊 포인트 클라우드 파일 생성 시작 (\(format.rawValue)) - 포인트 수: \(self.currentPointCount)")

            if let data = PointCloudExporter.export(renderer: self, format: format) {
                let elapsed = CFAbsoluteTimeGetCurrent() - startTime
                print("✅ 파일 생성 완료 (\(format.rawValue)) - 소요 시간: \(String(format: "%.2f", elapsed))초, 크기: \(data.count) bytes")
                DispatchQueue.main.async { [weak self] in
                    self?.lidarRawData = data
                    self?.delegate?.finishMakingPlyFile()
                }
            } else {
                print("❌ PointCloudExporter 실패, ASCII fallback")
                if let str = PointCloudExporter.exportAsString(renderer: self, format: .plyAscii) {
                    DispatchQueue.main.async { [weak self] in
                        self?.lidarRawStringData = str
                        self?.delegate?.finishMakingPlyFile()
                    }
                }
            }
        }
    }
}

extension Renderer {
    /// 재측정을 위한 Renderer 초기화 함수
    /// https://github.com/ryanphilly/IOS-PointCloud 코드 참고
    func clearParticles() {
        self.clearing = true

        // ⚠️ GPU에 제출된 command buffer가 완료될 때까지 대기
        // 완료 안 된 상태에서 commandQueue/버퍼를 교체하면 세마포어가 영원히 잠김
        for _ in 0..<maxInFlightBuffers {
            inFlightSemaphore.wait()
        }

        self.currentPointIndex = 0
        self.currentPointCount = 0

        // Export 세대 증가 (이전 비동기 export 결과 무효화)
        self.exportGeneration += 1
        // Release exported data
        self.lidarRawData = nil
        self.lidarRawStringData = nil
        self.imuDisplayData = nil
        self.trajectoryPoses.removeAll()
        self.imuLog.removeAll()
        self.trajectoryLog.removeAll()
        self.poseCorrectionLog.removeAll()
        self.recordingStartTime = nil

        // 카메라/SLAM 상태 완전 초기화 (4000포인트 잔류 방지)
        self.lastCameraTransform = nil
        self.startCameraTransform = nil
        self.latestSLAMPose = nil
        self.slamPoseUpdateCount = 0
        self.slamFrameFeedCount = 0
        self.imuSampleCount = 0
        self.voxelResetCounter = 0
        self.slamDiagTimer?.invalidate()
        self.slamDiagTimer = nil
        self.cameraResolution = nil
        self.stationaryFrameCount = 0
        self.currentFrameIndex = 0
        self.frameNilCount = 0
        self.firstFrameReceived = false
        self.cameraBackgroundRendered = false

        // 버퍼 인덱스 + commandQueue 리셋 (GPU 에러 복구 필수)
        self.currentBufferIndex = 0
        self.commandQueue = device.makeCommandQueue()!

        self.rgbUniformsBuffers.removeAll()
        self.pointCloudUniformsBuffers.removeAll()
        for _ in 0 ..< maxInFlightBuffers {
            self.rgbUniformsBuffers.append(.init(device: device, count: 1, index: 0))
            self.pointCloudUniformsBuffers.append(.init(device: device, count: 1, index: kPointCloudUniforms.rawValue))
        }
        self.particlesBuffer = .init(device: device, count: maxPoints, index: kParticleUniforms.rawValue)
        // Voxel grid 초기화 (0으로 채움 = 모든 복셀 비어있음, 최대 크기로 리셋)
        let maxVoxelGridSize = 1 << 24  // 16M
        memset(voxelGridBuffer.contents(), 0, maxVoxelGridSize * MemoryLayout<UInt32>.size)
        // 알고리즘에 맞게 그리드 포인트 재생성 (Mobile-LIO: 8192, ARKit: 2048)
        self.rebuildGridPointsBuffer()

        // 세마포어 복원 (초기 상태로 되돌림)
        for _ in 0..<maxInFlightBuffers {
            inFlightSemaphore.signal()
        }

        self.clearing = false
    }
    
    // MARK: - SLAMDelegate Methods
    //
    // DV-SLAM provides real-time pose refinement via didUpdatePose(), used for
    // localToWorld calculation in update(). After recording stops, the optimized
    // map is loaded into particlesBuffer via optimizeAndExport().

    func didUpdatePose(_ pose: simd_float4x4) {
        // DV-SLAM 보정 포즈 저장
        let wasNil = latestSLAMPose == nil
        latestSLAMPose = pose
        slamPoseUpdateCount += 1

        // SLAM 포즈 보정 쌍 기록 (후처리에서 GPU 포인트 보정에 사용)
        if isRecording, let arkit = lastCameraTransform {
            poseCorrectionLog.append(PoseCorrectionEntry(
                pointIndex: currentPointIndex,
                arkitPose: arkit,
                slamPose: pose
            ))
        }

        if wasNil {
            let pos = pose.columns.3
            print("🎯 [DV-SLAM] 첫 포즈 수신! 위치: (\(String(format: "%.3f, %.3f, %.3f", pos.x, pos.y, pos.z)))")
        }
    }

    func didUpdatePointCloud(_ pointCloudBuffer: MTLBuffer, count: UInt) {
        // Not used — GPU shader accumulates points directly from depth textures.
    }

    func didUpdateDisplayPoints(_ points: UnsafeRawPointer, count: UInt) {
        // Not used during real-time rendering.
    }

    // MARK: - Post-Processing Optimization Pipeline
    //
    // DV-SLAM: SLAM 포즈보정 → 표면씬닝 → 복셀 평균화 → 이상치 제거
    // ARKit:   최적화 없이 raw output
    //
    // SLAM 맵은 포즈 보정에만 사용 (직접 필터링은 confidence level이 충분히 처리)

    func optimizeAndExport(useSLAM: Bool) {
        self.clearing = true
        delegate?.startMakingPlyFile()

        // GPU에 제출된 command buffer 완료 대기 (particlesBuffer CPU 읽기 전 필수)
        for _ in 0..<maxInFlightBuffers {
            inFlightSemaphore.wait()
        }
        for _ in 0..<maxInFlightBuffers {
            inFlightSemaphore.signal()
        }

        let currentGen = self.exportGeneration

        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else { return }

            // 세대 불일치 시 이전 export → 무시
            guard self.exportGeneration == currentGen else {
                print("⚠️ Export 세대 불일치 (\(currentGen) vs \(self.exportGeneration)) — 무시")
                DispatchQueue.main.async { self.clearing = false }
                return
            }

            let startTime = CFAbsoluteTimeGetCurrent()
            let originalCount = self.currentPointCount

            // 포인트가 없으면 즉시 종료
            guard originalCount > 0 else {
                print("⚠️ 포인트 0개 — Export 스킵")
                DispatchQueue.main.async {
                    self.clearing = false
                    self.delegate?.finishMakingPlyFile()
                }
                return
            }

            // ── Phase 0: Confidence 필터링 ──
            // 셰이더는 confidence 0,1,2 모두 버퍼에 저장하지만 렌더링 시 threshold 이상만 표시.
            // 후처리/프리뷰/내보내기에서도 동일 기준 적용: 저품질 포인트 제거.
            let confThreshold = self.pointCloudUniforms.confidenceThreshold  // High=1.5
            var confRejected = 0
            for i in 0..<originalCount {
                var p = self.particlesBuffer[i]
                if p.confidence >= 0 && Float(p.confidence) < confThreshold {
                    p.confidence = -1
                    self.particlesBuffer[i] = p
                    confRejected += 1
                }
            }

            // 버퍼 압축: confidence < 0 제거
            if confRejected > 0 {
                var writeIdx = 0
                for i in 0..<originalCount {
                    let p = self.particlesBuffer[i]
                    if p.confidence >= 0 {
                        if writeIdx != i { self.particlesBuffer[writeIdx] = p }
                        writeIdx += 1
                    }
                }
                for i in writeIdx..<originalCount {
                    var p = self.particlesBuffer[i]
                    p.confidence = -1
                    self.particlesBuffer[i] = p
                }
                self.currentPointIndex = writeIdx
                self.currentPointCount = writeIdx
            }
            let afterConfCount = self.currentPointCount
            print("📊 [Confidence 필터] \(originalCount)개 → \(afterConfCount)개 (제거: \(confRejected)개, threshold≥\(confThreshold))")

            // ── Phase 0b: 원점 기준 거리 필터 ──
            // 셰이더의 maxDistance는 카메라↔점 depth만 체크 (카메라 이동 시 전체 범위 무제한)
            // 여기서 스캔 시작 위치 기준으로 월드 공간 거리도 필터
            let worldDistLimit = self.pointCloudUniforms.maxDistance
            if worldDistLimit < 999, let startTransform = self.startCameraTransform {
                let origin = SIMD3<Float>(startTransform.columns.3.x,
                                          startTransform.columns.3.y,
                                          startTransform.columns.3.z)
                var distRejected = 0
                let count = self.currentPointCount
                for i in 0..<count {
                    var p = self.particlesBuffer[i]
                    guard p.confidence >= 0 else { continue }
                    let d = distance(p.position, origin)
                    if d > worldDistLimit {
                        p.confidence = -1
                        self.particlesBuffer[i] = p
                        distRejected += 1
                    }
                }
                // 버퍼 압축
                if distRejected > 0 {
                    var writeIdx = 0
                    for i in 0..<count {
                        let p = self.particlesBuffer[i]
                        if p.confidence >= 0 {
                            if writeIdx != i { self.particlesBuffer[writeIdx] = p }
                            writeIdx += 1
                        }
                    }
                    for i in writeIdx..<count {
                        var p = self.particlesBuffer[i]
                        p.confidence = -1
                        self.particlesBuffer[i] = p
                    }
                    self.currentPointIndex = writeIdx
                    self.currentPointCount = writeIdx
                }
                print("📊 [거리 필터] \(afterConfCount)개 → \(self.currentPointCount)개 (제거: \(distRejected)개, 원점기준 >\(String(format: "%.1f", worldDistLimit))m)")
            }
            let afterDistCount = self.currentPointCount

            // GPU 포인트 바운딩 박스 분석
            var gpuMin = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
            var gpuMax = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
            var validCount = 0
            self.particlesBuffer.withUnsafeBufferPointer { buffer in
                let n = min(afterDistCount, buffer.count)
                for i in 0..<n {
                    let p = buffer[i]
                    guard p.confidence >= 0 else { continue }
                    gpuMin = min(gpuMin, p.position)
                    gpuMax = max(gpuMax, p.position)
                    validCount += 1
                }
            }
            let gpuSpan = gpuMax - gpuMin
            print("📊 [GPU 포인트] \(afterConfCount)개 (유효 \(validCount)개) | 범위: X=\(String(format: "%.2f", gpuSpan.x))m Y=\(String(format: "%.2f", gpuSpan.y))m Z=\(String(format: "%.2f", gpuSpan.z))m")

            if useSLAM {
                // ═══════════════════════════════════════════
                // Mobile-LIO 4단계 파이프라인
                // SLAM포즈보정 → 표면씬닝 → 8mm복셀 → 이상치제거
                // (SLAM맵은 포즈보정에만 사용, 직접 필터링 안함)
                // ═══════════════════════════════════════════

                // Phase 1: SLAM 포즈 보정 (드리프트 교정)
                let t1 = CFAbsoluteTimeGetCurrent()
                self.applySLAMPoseCorrection()
                let p1 = self.currentPointCount
                let t1e = CFAbsoluteTimeGetCurrent() - t1

                // Phase 2: 표면 씬닝 (드리프트로 겹친 중복 벽 제거)
                let t2 = CFAbsoluteTimeGetCurrent()
                self.performSurfaceThinning()
                let p2 = self.currentPointCount
                let t2e = CFAbsoluteTimeGetCurrent() - t2

                // Phase 3: 8mm 복셀 평균화 (세밀한 디테일 유지)
                let t3 = CFAbsoluteTimeGetCurrent()
                self.performVoxelDownsampling(voxelSize: 0.008)
                let p3 = self.currentPointCount
                let t3e = CFAbsoluteTimeGetCurrent() - t3

                // Phase 4: 통계적 이상치 제거 (고립 노이즈 포인트)
                let t4 = CFAbsoluteTimeGetCurrent()
                self.removeStatisticalOutliers()
                let p4 = self.currentPointCount
                let t4e = CFAbsoluteTimeGetCurrent() - t4

                let elapsed = CFAbsoluteTimeGetCurrent() - startTime
                let totalReduction = Float(originalCount - p4) / Float(max(originalCount, 1)) * 100
                print("═══════════════════════════════════════")
                print("🔬 Mobile-LIO 파이프라인 완료")
                print("   0a.신뢰도: \(originalCount) → \(afterConfCount) (conf≥\(confThreshold))")
                print("   0b.거리:  \(afterConfCount) → \(afterDistCount) (원점기준)")
                print("   1.포즈보정: \(afterDistCount) → \(p1) [\(String(format: "%.2f", t1e))초]")
                print("   2.씬닝:    \(p1) → \(p2) [\(String(format: "%.2f", t2e))초]")
                print("   3.복셀8mm: \(p2) → \(p3) [\(String(format: "%.2f", t3e))초]")
                print("   4.이상치:  \(p3) → \(p4) [\(String(format: "%.2f", t4e))초]")
                print("   총: \(originalCount) → \(p4) (제거 \(String(format: "%.1f%%", totalReduction))) [\(String(format: "%.1f", elapsed))초]")
                print("═══════════════════════════════════════")
            } else {
                // ═══════════════════════════════════════════
                // ARKit: confidence + 거리 필터만 적용
                // ═══════════════════════════════════════════
                let elapsed = CFAbsoluteTimeGetCurrent() - startTime
                print("📱 ARKit: \(originalCount) → \(afterDistCount) (conf≥\(confThreshold), dist) [\(String(format: "%.1f", elapsed))초]")
            }

            // GPU 렌더 재개
            DispatchQueue.main.async { self.clearing = false }

            // 파일 내보내기
            let format = ScanSettings.shared.fileFormat
            if let data = PointCloudExporter.export(renderer: self, format: format) {
                let total = CFAbsoluteTimeGetCurrent() - startTime
                print("✅ 내보내기 완료 (\(format.rawValue)) — \(String(format: "%.1f", total))초, \(data.count) bytes")
                DispatchQueue.main.async { [weak self] in
                    self?.lidarRawData = data
                    self?.delegate?.finishMakingPlyFile()
                }
            } else if let str = PointCloudExporter.exportAsString(renderer: self, format: .plyAscii) {
                DispatchQueue.main.async { [weak self] in
                    self?.lidarRawStringData = str
                    self?.delegate?.finishMakingPlyFile()
                }
            }
        }
    }

    // MARK: Phase 0 — SLAM 포즈 보정 (드리프트 교정)
    //
    // poseCorrectionLog의 (ARKit, SLAM) 포즈 쌍을 이용하여
    // GPU 포인트의 ARKit 드리프트를 SLAM 기준으로 보정.
    //
    // T_correction_i = SLAM_i × ARKit_i⁻¹
    // p_corrected = T_correction_0⁻¹ × T_correction_i × p_world
    //
    // T_correction_0⁻¹가 Z-up↔Y-up 변환을 상쇄 → 출력은 ARKit Y-up 유지

    /// 보정 쌍의 회전 차이(도)를 계산
    private func rotationDifferenceDeg(_ a: simd_float4x4, _ b: simd_float4x4) -> Float {
        let rel = b * a.inverse
        let trace = rel.columns.0.x + rel.columns.1.y + rel.columns.2.z
        return acos(max(-1, min(1, (trace - 1) / 2))) * 180 / .pi
    }

    private func applySLAMPoseCorrection() {
        guard poseCorrectionLog.count >= 2 else {
            print("⚠️ SLAM 보정 쌍 부족 (\(poseCorrectionLog.count)) — 보정 생략")
            return
        }

        let count = currentPointCount
        guard count > 0 else { return }

        // ── 수렴 감지: SLAM 초기화 단계의 불안정 쌍 제거 ──
        // SLAM 엔진은 초기 몇 초간 위치/회전이 크게 발산 (최대 90°+)
        // 수렴 기준: 회전차 < 10° AND 위치차 < 0.15m
        let convergenceRotThreshold: Float = 10.0  // 도
        let convergencePosThreshold: Float = 0.15  // m

        var convergedLog: [PoseCorrectionEntry] = []
        var skippedCount = 0

        for entry in poseCorrectionLog {
            let aPos = SIMD3<Float>(entry.arkitPose.columns.3.x, entry.arkitPose.columns.3.y, entry.arkitPose.columns.3.z)
            let sPos = SIMD3<Float>(entry.slamPose.columns.3.x, entry.slamPose.columns.3.y, entry.slamPose.columns.3.z)
            let posDiff = distance(aPos, sPos)
            let rotDiff = rotationDifferenceDeg(entry.arkitPose, entry.slamPose)

            if rotDiff < convergenceRotThreshold && posDiff < convergencePosThreshold {
                convergedLog.append(entry)
            } else {
                skippedCount += 1
            }
        }

        print("📊 [수렴 필터] 전체 \(poseCorrectionLog.count)쌍 → 수렴 \(convergedLog.count)쌍 (스킵 \(skippedCount)쌍, 기준: 회전<\(convergenceRotThreshold)° 위치<\(convergencePosThreshold)m)")

        guard convergedLog.count >= 2 else {
            print("⚠️ 수렴된 보정 쌍 부족 (\(convergedLog.count)) — 보정 생략")
            return
        }

        // ── 드리프트 분석 (수렴된 쌍만) ──
        var driftMin: Float = .greatestFiniteMagnitude
        var driftMax: Float = 0
        var driftSum: Float = 0
        var rotMin: Float = .greatestFiniteMagnitude
        var rotMax: Float = 0

        for entry in convergedLog {
            let aPos = SIMD3<Float>(entry.arkitPose.columns.3.x, entry.arkitPose.columns.3.y, entry.arkitPose.columns.3.z)
            let sPos = SIMD3<Float>(entry.slamPose.columns.3.x, entry.slamPose.columns.3.y, entry.slamPose.columns.3.z)
            let d = distance(aPos, sPos)
            driftMin = min(driftMin, d)
            driftMax = max(driftMax, d)
            driftSum += d

            let rotDiff = rotationDifferenceDeg(entry.arkitPose, entry.slamPose)
            rotMin = min(rotMin, rotDiff)
            rotMax = max(rotMax, rotDiff)
        }
        let driftAvg = driftSum / Float(convergedLog.count)

        print("📊 [포즈 분석] \(convergedLog.count)쌍 | ARKit↔SLAM 위치차: min=\(String(format: "%.3f", driftMin))m avg=\(String(format: "%.3f", driftAvg))m max=\(String(format: "%.3f", driftMax))m | 회전차: \(String(format: "%.1f", rotMin))°~\(String(format: "%.1f", rotMax))°")

        // 기준 프레임: 첫 번째 수렴된 쌍
        let T0 = convergedLog[0].slamPose * convergedLog[0].arkitPose.inverse
        let T0_inv = T0.inverse

        // 보정량 분석 + 200mm 안전 캡
        let maxCorrectionCap: Float = 0.200  // 200mm — 이 이상 보정은 잘못된 것
        var correctionMin: Float = .greatestFiniteMagnitude
        var correctionMax: Float = 0
        var correctionSum: Float = 0
        var cappedCount = 0

        var correctedCount = 0
        var logIdx = 0

        for i in 0..<count {
            var p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }

            // 수렴 전 포인트 (pointIndex < 첫 수렴 쌍) → 보정 안함
            if i < convergedLog[0].pointIndex {
                continue
            }

            // 수렴 후 포인트: 해당 시점의 보정 쌍으로 드리프트 교정
            while logIdx < convergedLog.count - 1 &&
                  convergedLog[logIdx + 1].pointIndex <= i {
                logIdx += 1
            }

            let entry = convergedLog[logIdx]
            let Ti = entry.slamPose * entry.arkitPose.inverse
            let correction = T0_inv * Ti  // 드리프트만 추출

            let p4 = SIMD4<Float>(p.position.x, p.position.y, p.position.z, 1.0)
            let corrected = correction * p4
            let correctedPos = SIMD3<Float>(corrected.x, corrected.y, corrected.z)
            let displacement = distance(p.position, correctedPos)

            // 보정량이 200mm 초과 시 스킵 (비정상 보정 방지)
            if displacement > maxCorrectionCap {
                cappedCount += 1
                continue
            }

            correctionMin = min(correctionMin, displacement)
            correctionMax = max(correctionMax, displacement)
            correctionSum += displacement

            p.position = correctedPos
            particlesBuffer[i] = p
            correctedCount += 1
        }

        let correctionAvg = correctedCount > 0 ? correctionSum / Float(correctedCount) : 0
        print("🔬 SLAM 포즈 보정: \(correctedCount)/\(count) 포인트 | 보정량: min=\(String(format: "%.1f", correctionMin*1000))mm avg=\(String(format: "%.1f", correctionAvg*1000))mm max=\(String(format: "%.1f", correctionMax*1000))mm | 캡초과스킵: \(cappedCount)개")
    }

    // MARK: Phase 1 — SLAM 맵 기반 공간 필터링
    //
    // SLAM full_map (키프레임 LIO 포즈로 배치된 검증된 점)을
    // GPU 포인트의 공간 필터로 사용.
    //
    // Step 0에서 GPU 포인트가 T0_inv * Ti * p로 보정됨
    // SLAM 맵도 동일하게 T0_inv * p_slam으로 변환해야 같은 좌표계

    private func filterBySLAMProximity() {
        let count = currentPointCount
        guard count > 0 else { return }

        // ── 1. SLAM full_map 가져오기 ──
        var slamPoints: [SLAMPoint] = []
        SLAMService.sharedInstance().getMapForExport { points, cnt in
            let n = Int(cnt)
            guard n > 0 else { return }
            let ptr = points.bindMemory(to: SLAMPoint.self, capacity: n)
            slamPoints = Array(UnsafeBufferPointer(start: ptr, count: n))
        }

        guard slamPoints.count >= 50 else {
            print("⚠️ SLAM 맵 부족 (\(slamPoints.count)개) — 필터링 생략")
            return
        }

        // SLAM 맵 바운딩 박스 분석
        var slamMin = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var slamMax = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
        for sp in slamPoints {
            let p = SIMD3<Float>(sp.x, sp.y, sp.z)
            slamMin = min(slamMin, p)
            slamMax = max(slamMax, p)
        }
        let slamSpan = slamMax - slamMin
        print("📊 [SLAM 맵] \(slamPoints.count)개 | 범위: X=\(String(format: "%.2f", slamSpan.x))m Y=\(String(format: "%.2f", slamSpan.y))m Z=\(String(format: "%.2f", slamSpan.z))m")

        // ── 2. SLAM→보정된 ARKit 공간 정렬 행렬 ──
        // Step 0(applySLAMPoseCorrection)에서 GPU 포인트가 T0_inv * Ti * p로 보정됨
        // SLAM 맵도 동일한 기준 프레임으로 변환: T0_inv * p_slam
        let alignTransform: simd_float4x4
        if poseCorrectionLog.count >= 1 {
            let T0 = poseCorrectionLog[0].slamPose * poseCorrectionLog[0].arkitPose.inverse
            alignTransform = T0.inverse  // SLAM→보정된 ARKit 공간
            let t = alignTransform.columns.3
            let transDist = sqrt(t.x*t.x + t.y*t.y + t.z*t.z)
            print("🔬 SLAM→ARKit 정렬: T0⁻¹ 사용 (translation=\(String(format: "%.3f", transDist))m) [\(poseCorrectionLog.count)쌍]")
        } else {
            alignTransform = matrix_identity_float4x4
            print("⚠️ poseCorrectionLog 비어있음 — identity 사용")
        }

        // ── 3. SLAM 점을 ARKit 프레임으로 변환 + 복셀 점유 그리드 구축 ──
        // SLAM full_map은 키프레임 점만 포함 (공간적으로 희소)
        // → 10cm 복셀 + 2-ring 확장 = ~50cm 유효 반경으로 충분한 커버리지 확보
        let voxelSize: Float = 0.10  // 10cm 복셀
        let invVoxel: Float = 1.0 / voxelSize

        struct VKey: Hashable {
            let x: Int32, y: Int32, z: Int32
        }

        var occupiedVoxels = Set<VKey>()
        occupiedVoxels.reserveCapacity(slamPoints.count * 125)

        for sp in slamPoints {
            let p4 = SIMD4<Float>(sp.x, sp.y, sp.z, 1.0)
            let aligned = alignTransform * p4

            let vx = Int32(floor(aligned.x * invVoxel))
            let vy = Int32(floor(aligned.y * invVoxel))
            let vz = Int32(floor(aligned.z * invVoxel))

            // 2-ring 확장: 각 SLAM 점 주변 125개 복셀 점유 표시
            // → 약 50cm 유효 반경 (10cm × 2 + 반복셀)
            for dx: Int32 in -2...2 {
                for dy: Int32 in -2...2 {
                    for dz: Int32 in -2...2 {
                        occupiedVoxels.insert(VKey(x: vx + dx, y: vy + dy, z: vz + dz))
                    }
                }
            }
        }

        // ── 4. Dry-run: 제거 비율 사전 검사 ──
        var wouldRemove = 0
        for i in 0..<count {
            let p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }
            let vx = Int32(floor(p.position.x * invVoxel))
            let vy = Int32(floor(p.position.y * invVoxel))
            let vz = Int32(floor(p.position.z * invVoxel))
            if !occupiedVoxels.contains(VKey(x: vx, y: vy, z: vz)) {
                wouldRemove += 1
            }
        }

        let removeRatio = Float(wouldRemove) / Float(max(count, 1))
        if removeRatio > 0.85 {
            // 85% 이상 제거 = 정렬 실패 의심 → 필터링 중단
            print("⚠️ SLAM 필터 중단: \(String(format: "%.0f%%", removeRatio * 100)) 제거 예상 — 정렬 실패 의심 [SLAM \(slamPoints.count)개, 복셀 \(occupiedVoxels.count)개]")
            return
        }

        // ── 5. 실제 필터링 적용 ──
        var removeCount = 0
        for i in 0..<count {
            var p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }
            let vx = Int32(floor(p.position.x * invVoxel))
            let vy = Int32(floor(p.position.y * invVoxel))
            let vz = Int32(floor(p.position.z * invVoxel))
            if !occupiedVoxels.contains(VKey(x: vx, y: vy, z: vz)) {
                p.confidence = -1
                particlesBuffer[i] = p
                removeCount += 1
            }
        }

        // ── 6. 버퍼 압축 ──
        if removeCount > 0 {
            var writeIdx = 0
            for i in 0..<count {
                let p = particlesBuffer[i]
                if p.confidence >= 0 {
                    if writeIdx != i {
                        particlesBuffer[writeIdx] = p
                    }
                    writeIdx += 1
                }
            }
            for i in writeIdx..<count {
                var p = particlesBuffer[i]
                p.confidence = -1
                particlesBuffer[i] = p
            }
            currentPointIndex = writeIdx
            currentPointCount = writeIdx
        }

        let pct = Float(removeCount) / Float(max(count, 1)) * 100
        print("🔬 SLAM 근접 필터: \(count) → \(currentPointCount) (제거: \(removeCount), \(String(format: "%.1f%%", pct))) [SLAM 맵: \(slamPoints.count)개, 점유 복셀: \(occupiedVoxels.count)개]")
    }

    // MARK: Phase 2 — 중복 표면 제거 (Surface Thinning)
    //
    // 드리프트/multipath로 인해 벽 뒤로 길쭉하게 늘어나는 고스트 제거
    // 30mm 영역마다 포인트 분포를 분석하여 가장 밀집된 단일 표면층만 유지
    // Gap(빈 공간)이 감지된 경우에만 적용 → 모서리/코너 보존

    private func performSurfaceThinning() {
        let count = currentPointCount
        guard count > 1000 else { return }

        let coarseSize: Float = 0.030   // 30mm 분석 영역
        let binSize: Float = 0.003      // 3mm 히스토그램 빈
        let minSpread: Float = 0.015    // 15mm 미만 두께 = 정상 표면, 스킵
        let gapThreshold: Float = 0.008 // 8mm 이상 빈 공간 = 중복/고스트 감지
        let keepWindow: Float = 0.015   // 15mm 윈도우 = 유지할 표면 두께
        let maxRemoveRatio: Float = 0.50

        struct CKey: Hashable { let x: Int32, y: Int32, z: Int32 }

        var groups: [CKey: [Int]] = [:]
        groups.reserveCapacity(count / 5)

        for i in 0..<count {
            let p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }
            let key = CKey(x: Int32(floor(p.position.x / coarseSize)),
                           y: Int32(floor(p.position.y / coarseSize)),
                           z: Int32(floor(p.position.z / coarseSize)))
            groups[key, default: []].append(i)
        }

        var removeFlags = [Bool](repeating: false, count: count)
        var totalRemoved = 0

        for (_, indices) in groups {
            guard indices.count > 10 else { continue }

            // 바운딩 박스 계산
            var mn = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
            var mx = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
            for idx in indices {
                let pos = particlesBuffer[idx].position
                mn = min(mn, pos)
                mx = max(mx, pos)
            }
            let spread = mx - mn

            // 최대 확산 축 결정
            var axis = 0
            var maxSprd = spread.x
            if spread.y > maxSprd { axis = 1; maxSprd = spread.y }
            if spread.z > maxSprd { axis = 2; maxSprd = spread.z }

            // 15mm 미만이면 단일 표면 → 스킵
            guard maxSprd > minSpread else { continue }

            let minVal: Float
            switch axis {
            case 0: minVal = mn.x
            case 1: minVal = mn.y
            default: minVal = mn.z
            }

            // 히스토그램 구축 (3mm 빈)
            let numBins = min(Int(ceil(maxSprd / binSize)), 200)
            guard numBins > 3 else { continue }
            var histogram = [Int](repeating: 0, count: numBins)

            for idx in indices {
                let pos = particlesBuffer[idx].position
                let val: Float
                switch axis {
                case 0: val = pos.x
                case 1: val = pos.y
                default: val = pos.z
                }
                let bin = max(0, min(numBins - 1, Int(floor((val - minVal) / binSize))))
                histogram[bin] += 1
            }

            // Gap 감지: 8mm 이상 빈 공간이 있는지 확인
            // Gap이 없으면 = 연속 표면(모서리 등) → 건드리지 않음
            var hasGap = false
            var gapRun = 0
            let gapBins = max(1, Int(ceil(gapThreshold / binSize)))
            for bin in 0..<numBins {
                if histogram[bin] == 0 {
                    gapRun += 1
                    if gapRun >= gapBins { hasGap = true; break }
                } else {
                    gapRun = 0
                }
            }

            guard hasGap else { continue }

            // 슬라이딩 윈도우: 가장 밀집된 15mm 구간 탐색
            let windowBins = max(1, Int(ceil(keepWindow / binSize)))
            var bestStart = 0
            var bestCount = 0

            var winCount = 0
            for b in 0..<min(windowBins, numBins) { winCount += histogram[b] }
            bestCount = winCount

            for start in 1..<max(1, numBins - windowBins + 1) {
                winCount -= histogram[start - 1]
                if start + windowBins - 1 < numBins {
                    winCount += histogram[start + windowBins - 1]
                }
                if winCount > bestCount {
                    bestCount = winCount
                    bestStart = start
                }
            }

            let keepMin = minVal + Float(bestStart) * binSize
            let keepMax = keepMin + Float(windowBins) * binSize

            // 가장 밀집된 표면층 밖의 포인트를 제거 대상으로 표시
            for idx in indices {
                let pos = particlesBuffer[idx].position
                let val: Float
                switch axis {
                case 0: val = pos.x
                case 1: val = pos.y
                default: val = pos.z
                }
                if val < keepMin || val > keepMax {
                    removeFlags[idx] = true
                    totalRemoved += 1
                }
            }
        }

        guard totalRemoved > 0 else {
            print("🔬 표면 씬닝: \(count)개 중 중복 레이어 없음 — 스킵")
            return
        }

        // 안전장치: 최대 50% 제거 — 너무 많이 제거하려 하면 제한
        let maxAllowed = Int(Float(count) * maxRemoveRatio)
        if totalRemoved > maxAllowed {
            // 제거 대상 중 일부만 실제 제거
            var removed = 0
            for i in 0..<count where removeFlags[i] {
                if removed >= maxAllowed { removeFlags[i] = false }
                else { removed += 1 }
            }
            totalRemoved = maxAllowed
            print("⚠️ 표면 씬닝 안전장치 발동: 제거 제한 \(maxAllowed)개")
        }

        // 버퍼 압축 (제거된 포인트 건너뛰기)
        var newIndex = 0
        for i in 0..<count {
            if !removeFlags[i] {
                if newIndex != i {
                    particlesBuffer[newIndex] = particlesBuffer[i]
                }
                newIndex += 1
            }
        }

        for i in newIndex..<count {
            var p = particlesBuffer[i]
            p.confidence = -1
            particlesBuffer[i] = p
        }

        currentPointIndex = newIndex
        currentPointCount = newIndex
        let pct = Float(totalRemoved) / Float(max(count, 1)) * 100
        print("🔬 표면 씬닝: \(count) → \(newIndex) (제거: \(totalRemoved)개, \(String(format: "%.1f%%", pct))) [분석 영역: \(groups.count)개, bin=\(binSize*1000)mm, gap≥\(gapThreshold*1000)mm]")
    }

    // MARK: Phase 3 — 복셀 다운샘플링
    //
    // 작은 복셀(5mm) 내 관측 횟수가 minCount 미만이면 버림
    // 노이즈/고스트는 1~2회만 관측 → 제거, 실제 표면은 다수 관측 → 유지
    // 통과한 복셀은 위치/색상 평균화로 정밀도 향상

    private func performVoxelDownsampling(voxelSize vs: Float) {
        let count = currentPointCount
        guard count > 100 else { return }

        struct VoxelKey: Hashable {
            let x: Int32, y: Int32, z: Int32
        }

        var voxels: [VoxelKey: (px: Float, py: Float, pz: Float,
                                cr: Float, cg: Float, cb: Float,
                                conf: Float, n: Int)] = [:]
        voxels.reserveCapacity(count)

        particlesBuffer.withUnsafeBufferPointer { buffer in
            let clampedCount = min(count, buffer.count)
            for i in 0..<clampedCount {
                let p = buffer[i]
                guard p.confidence >= 0 else { continue }

                let key = VoxelKey(
                    x: Int32(floor(p.position.x / vs)),
                    y: Int32(floor(p.position.y / vs)),
                    z: Int32(floor(p.position.z / vs))
                )

                if var v = voxels[key] {
                    v.px += p.position.x; v.py += p.position.y; v.pz += p.position.z
                    v.cr += p.color.x; v.cg += p.color.y; v.cb += p.color.z
                    v.conf = max(v.conf, p.confidence)
                    v.n += 1
                    voxels[key] = v
                } else {
                    voxels[key] = (p.position.x, p.position.y, p.position.z,
                                   p.color.x, p.color.y, p.color.z,
                                   p.confidence, 1)
                }
            }
        }

        var newIndex = 0
        for (_, v) in voxels {
            guard newIndex < particlesBuffer.count else { break }
            let n = Float(v.n)
            var particle = ParticleUniforms()
            particle.position = SIMD3<Float>(v.px / n, v.py / n, v.pz / n)
            particle.color = SIMD3<Float>(v.cr / n, v.cg / n, v.cb / n)
            particle.confidence = v.conf
            particlesBuffer[newIndex] = particle
            newIndex += 1
        }

        for i in newIndex..<count {
            var p = particlesBuffer[i]
            p.confidence = -1
            particlesBuffer[i] = p
        }

        currentPointIndex = newIndex
        currentPointCount = newIndex
        // 복셀당 포인트 수 통계
        let counts = voxels.values.map { $0.n }
        let avgPerVoxel = counts.isEmpty ? 0 : Float(counts.reduce(0, +)) / Float(counts.count)
        let maxPerVoxel = counts.max() ?? 0
        let singleVoxels = counts.filter { $0 == 1 }.count
        let pct = Float(count - newIndex) / Float(max(count, 1)) * 100
        print("🔬 복셀 평균화: \(count) → \(newIndex) (제거: \(String(format: "%.1f%%", pct))) [vs=\(vs*1000)mm, \(voxels.count)복셀, avg=\(String(format: "%.1f", avgPerVoxel))pts/복셀, max=\(maxPerVoxel), 단독=\(singleVoxels)]")
    }

    // MARK: Phase 4 — 이상치 제거
    //
    // 안전한 필터링: 절대 기준만 사용, 최대 제거 비율 30% 제한
    //  (A) 밀도: 30mm 셀, 27-이웃 합계 < 3 → 완전 고립된 포인트만
    //  (B) 거리: IQR × 3.0 (매우 극단적인 이상치만)

    private func removeStatisticalOutliers() {
        let count = currentPointCount
        guard count > 500 else { return }
        let maxRemove = Int(Float(count) * 0.30)  // 안전장치: 최대 30% 제거

        struct GKey: Hashable { let x: Int32, y: Int32, z: Int32 }

        let cellSize: Float = 0.030  // 30mm 셀

        var cellCounts: [GKey: Int] = [:]
        cellCounts.reserveCapacity(count)
        var pointCells = [GKey]()
        pointCells.reserveCapacity(count)

        var sumPos = SIMD3<Float>(0, 0, 0)
        var validCount = 0

        particlesBuffer.withUnsafeBufferPointer { buffer in
            let n = min(count, buffer.count)
            for i in 0..<n {
                let p = buffer[i]
                guard p.confidence >= 0 else {
                    pointCells.append(GKey(x: 0, y: 0, z: 0))
                    continue
                }
                let key = GKey(x: Int32(floor(p.position.x / cellSize)),
                               y: Int32(floor(p.position.y / cellSize)),
                               z: Int32(floor(p.position.z / cellSize)))
                cellCounts[key, default: 0] += 1
                pointCells.append(key)
                sumPos += p.position
                validCount += 1
            }
        }

        guard validCount > 100 else { return }

        // 각 포인트의 이웃 밀도 계산
        var densities = [Int](repeating: 0, count: count)
        for i in 0..<count {
            let p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }
            let key = pointCells[i]
            var neighborCount = 0
            for dx: Int32 in -1...1 {
                for dy: Int32 in -1...1 {
                    for dz: Int32 in -1...1 {
                        let nk = GKey(x: key.x + dx, y: key.y + dy, z: key.z + dz)
                        neighborCount += cellCounts[nk] ?? 0
                    }
                }
            }
            densities[i] = neighborCount - 1
        }

        // 거리 기반 IQR (극단 이상치만: 3.0배)
        let centroid = sumPos / Float(validCount)
        var distances = [Float]()
        distances.reserveCapacity(validCount)

        for i in 0..<count {
            let p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }
            distances.append(distance(p.position, centroid))
        }
        distances.sort()

        let q1 = distances[distances.count / 4]
        let q3 = distances[(distances.count * 3) / 4]
        let iqr = q3 - q1
        let distanceCutoff = q3 + 3.0 * iqr  // 극단 이상치만 (3.0배)

        // 필터 적용
        var removeFlags = [Bool](repeating: false, count: count)
        var removedByDensity = 0
        var removedByDistance = 0
        var totalRemoved = 0

        var distIdx = 0
        for i in 0..<count {
            let p = particlesBuffer[i]
            guard p.confidence >= 0 else { continue }

            // 안전장치: 이미 30% 제거했으면 중단
            if totalRemoved >= maxRemove { distIdx += 1; continue }

            let dist = distances[distIdx]
            distIdx += 1

            // (B) 거리 극단 이상치
            if dist > distanceCutoff {
                removeFlags[i] = true
                removedByDistance += 1
                totalRemoved += 1
                continue
            }

            // (A) 완전 고립 (이웃 < 3)
            if densities[i] < 3 {
                removeFlags[i] = true
                removedByDensity += 1
                totalRemoved += 1
                continue
            }
        }

        guard totalRemoved > 0 else { return }

        // 버퍼 압축
        var newIndex = 0
        for i in 0..<count {
            if !removeFlags[i] && particlesBuffer[i].confidence >= 0 {
                if newIndex != i {
                    particlesBuffer[newIndex] = particlesBuffer[i]
                }
                newIndex += 1
            }
        }

        for i in newIndex..<count {
            var p = particlesBuffer[i]
            p.confidence = -1
            particlesBuffer[i] = p
        }

        currentPointIndex = newIndex
        currentPointCount = newIndex
        let outlierPct = Float(totalRemoved) / Float(max(count, 1)) * 100
        let medianDist = distances[distances.count / 2]
        print("🔬 이상치 제거: \(count) → \(newIndex) (\(String(format: "%.1f%%", outlierPct))) [밀도<3:\(removedByDensity)개, 거리극단:\(removedByDistance)개] 거리분포: Q1=\(String(format: "%.2f", q1))m 중앙=\(String(format: "%.2f", medianDist))m Q3=\(String(format: "%.2f", q3))m IQR=\(String(format: "%.2f", iqr))m cutoff=\(String(format: "%.2f", distanceCutoff))m")
    }
}

// MARK: - MetalTextureManager (Integrated for compatibility)

/// Metal 텍스처 관리를 담당하는 클래스
final class MetalTextureManager {
    private let device: MTLDevice
    private lazy var textureCache = makeTextureCache()
    
    // Captured image textures
    var capturedImageTextureY: CVMetalTexture?
    var capturedImageTextureCbCr: CVMetalTexture?
    
    // Depth and confidence textures
    var depthTexture: CVMetalTexture?
    var confidenceTexture: CVMetalTexture?
    
    init(device: MTLDevice) {
        self.device = device
    }
    
    /// Update captured image textures from AR frame
    func updateCapturedImageTextures(frame: ARFrame) {
        let pixelBuffer = frame.capturedImage
        let planeCount = CVPixelBufferGetPlaneCount(pixelBuffer)
        
        guard planeCount >= 2 else {
            print("⚠️ PixelBuffer plane count 부족: \(planeCount)")
            return
        }
        
        capturedImageTextureY = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .r8Unorm, planeIndex: 0)
        capturedImageTextureCbCr = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .rg8Unorm, planeIndex: 1)
        
        #if DEBUG
        if capturedImageTextureY == nil || capturedImageTextureCbCr == nil {
            print("❌ 카메라 텍스처 생성 실패: Y=\(capturedImageTextureY != nil), CbCr=\(capturedImageTextureCbCr != nil)")
        }
        #endif
    }
    
    /// Update depth and confidence textures from AR frame
    func updateDepthTextures(frame: ARFrame) -> Bool {
        guard let depthMap = frame.sceneDepth?.depthMap,
              let confidenceMap = frame.sceneDepth?.confidenceMap else {
            return false
        }
        
        depthTexture = makeTexture(fromPixelBuffer: depthMap, pixelFormat: .r32Float, planeIndex: 0)
        confidenceTexture = makeTexture(fromPixelBuffer: confidenceMap, pixelFormat: .r8Uint, planeIndex: 0)
        
        return true
    }
    
    // MARK: - Private Methods
    
    private func makeTextureCache() -> CVMetalTextureCache {
        var cache: CVMetalTextureCache!
        CVMetalTextureCacheCreate(nil, nil, device, nil, &cache)
        return cache
    }
    
    private func makeTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        
        var texture: CVMetalTexture?
        let status = CVMetalTextureCacheCreateTextureFromImage(nil, textureCache, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &texture)
        
        if status != kCVReturnSuccess {
            texture = nil
        }
        
        return texture
    }
}
