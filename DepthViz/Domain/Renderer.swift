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

final class Renderer {
    // Maximum number of points we store in the point cloud
    private let maxPoints = 8000000
    // Number of sample points on the grid (increased for better quality)
    private let numGridPoints = 1000
    // Particle's size in pixels (optimized for better visual quality)
    private let particleSize: Float = 3
    // We only use portrait orientation in this app
    private let orientation = UIInterfaceOrientation.portrait
    // Camera's threshold values for detecting when the camera moves so that we can accumulate the points
    // Reduced thresholds for more frequent sampling and higher quality
    private let cameraRotationThreshold = cos(1 * .degreesToRadian)
    private let cameraTranslationThreshold: Float = pow(0.01, 2)   // (meter-squared)
    // The max number of command buffers in flight
    private let maxInFlightBuffers = 3
    
    private lazy var rotateToARCamera = Self.makeRotateToARCameraMatrix(orientation: orientation)
    private let session: ARSession
    
    // Metal objects and textures
    private let device: MTLDevice
    private let library: MTLLibrary
    private let renderDestination: RenderDestinationProvider
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
    // The grid of sample points
    private lazy var gridPointsBuffer = MetalBuffer<Float2>(device: device,
                                                            array: makeGridPoints(),
                                                            index: kGridPoints.rawValue, options: [])
    
    private var rgbUniformsBuffers = [MetalBuffer<RGBUniforms>]()
    // Point Cloud buffer
    // This is not the point cloud data, but some parameters
    private lazy var pointCloudUniforms: PointCloudUniforms = {
        var uniforms = PointCloudUniforms()
        uniforms.maxPoints = Int32(maxPoints)
        uniforms.confidenceThreshold = Int32(confidenceThreshold)
        uniforms.particleSize = particleSize
        uniforms.cameraResolution = cameraResolution
        return uniforms
    }()
    private var pointCloudUniformsBuffers = [MetalBuffer<PointCloudUniforms>]()
    // Particles buffer
    // Saves the point cloud data, filled by unprojectVertex func in Shaders.metal
    private var particlesBuffer: MetalBuffer<ParticleUniforms>
    private var currentPointIndex = 0
    @Published private(set) var currentPointCount = 0
    
    // Camera data
    private var sampleFrame: ARFrame { session.currentFrame! }
    private lazy var cameraResolution = Float2(Float(sampleFrame.camera.imageResolution.width), Float(sampleFrame.camera.imageResolution.height))
    private lazy var lastCameraTransform = sampleFrame.camera.transform
    
    // threshHold 값을 low로 설정 (반사율이 낮은 물질도 잘 표시되도록 설정)
    var confidenceThreshold = 0 {
        didSet {
            // apply the change for the shader
            pointCloudUniforms.confidenceThreshold = Int32(confidenceThreshold)
        }
    }
    
    // Whether recording is on
    var isRecording = false
    // Pick every n frames (~1/sampling frequency)
    public var pickFrames = 5 // default to save 1/5 of the new frames
    public var currentFrameIndex = 0;
    // Task delegate for informing ViewController of tasks
    public weak var delegate: TaskDelegate?
    
    @Published private(set) var lidarRawStringData: String?
    private var clearing: Bool = false
    
    init(session: ARSession, metalDevice device: MTLDevice, renderDestination: RenderDestinationProvider) {
        self.session = session
        self.device = device
        self.renderDestination = renderDestination
        // MTLibrary를 생성
        self.library = device.makeDefaultLibrary()!
        self.commandQueue = device.makeCommandQueue()!
        
        // initialize our buffers
        for _ in 0 ..< maxInFlightBuffers {
            rgbUniformsBuffers.append(.init(device: device, count: 1, index: 0))
            pointCloudUniformsBuffers.append(.init(device: device, count: 1, index: kPointCloudUniforms.rawValue))
        }
        particlesBuffer = .init(device: device, count: maxPoints, index: kParticleUniforms.rawValue)
        
        // rbg does not need to read/write depth
        let relaxedStateDescriptor = MTLDepthStencilDescriptor()
        relaxedStencilState = device.makeDepthStencilState(descriptor: relaxedStateDescriptor)!
        
        // setup depth test for point cloud
        let depthStateDescriptor = MTLDepthStencilDescriptor()
        depthStateDescriptor.depthCompareFunction = .lessEqual
        depthStateDescriptor.isDepthWriteEnabled = true
        depthStencilState = device.makeDepthStencilState(descriptor: depthStateDescriptor)!
        
        inFlightSemaphore = DispatchSemaphore(value: maxInFlightBuffers)
    }
    
    func drawRectResized(size: CGSize) {
        self.viewportSize = size
    }
    // Texture update methods removed - now handled by MetalTextureManager
    
    private func update(frame: ARFrame) {
        // frame dependent info
        let camera = frame.camera
        let cameraIntrinsicsInversed = camera.intrinsics.inverse
        let viewMatrix = camera.viewMatrix(for: orientation)
        let viewMatrixInversed = viewMatrix.inverse
        let projectionMatrix = camera.projectionMatrix(for: orientation, viewportSize: viewportSize, zNear: 0.001, zFar: 0)
        pointCloudUniforms.viewProjectionMatrix = projectionMatrix * viewMatrix
        pointCloudUniforms.localToWorld = viewMatrixInversed * rotateToARCamera
        pointCloudUniforms.cameraIntrinsicsInversed = cameraIntrinsicsInversed
    }
    
    func draw() {
        guard self.clearing == false else { return }
        
        guard let currentFrame = session.currentFrame,
              let renderDescriptor = renderDestination.currentRenderPassDescriptor,
              let commandBuffer = commandQueue.makeCommandBuffer(),
              let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderDescriptor) else {
            return
        }
        
        _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        commandBuffer.addCompletedHandler { [weak self] commandBuffer in
            if let self = self {
                self.inFlightSemaphore.signal()
            }
        }
        
        // update frame data
        update(frame: currentFrame)
        textureManager.updateCapturedImageTextures(frame: currentFrame)
        
        // handle buffer rotating
        currentBufferIndex = (currentBufferIndex + 1) % maxInFlightBuffers
        pointCloudUniformsBuffers[currentBufferIndex][0] = pointCloudUniforms
        
        if shouldAccumulate(frame: currentFrame), textureManager.updateDepthTextures(frame: currentFrame) {
            accumulatePoints(frame: currentFrame, commandBuffer: commandBuffer, renderEncoder: renderEncoder)
        }
       
        // render particles
        renderEncoder.setDepthStencilState(depthStencilState)
        renderEncoder.setRenderPipelineState(particlePipelineState)
        renderEncoder.setVertexBuffer(pointCloudUniformsBuffers[currentBufferIndex])
        renderEncoder.setVertexBuffer(particlesBuffer)
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: currentPointCount)
        renderEncoder.endEncoding()
            
        commandBuffer.present(renderDestination.currentDrawable!)
        commandBuffer.commit()
    }
    
    private func shouldAccumulate(frame: ARFrame) -> Bool {
        // MARK: SAVE PLY
        if (!isRecording) {
            return false
        }
        let cameraTransform = frame.camera.transform
        return currentPointCount == 0
            || dot(cameraTransform.columns.2, lastCameraTransform.columns.2) <= cameraRotationThreshold
            || distance_squared(cameraTransform.columns.3, lastCameraTransform.columns.3) >= cameraTranslationThreshold
    }
    
    private func accumulatePoints(frame: ARFrame, commandBuffer: MTLCommandBuffer, renderEncoder: MTLRenderCommandEncoder) {
        pointCloudUniforms.pointCloudCurrentIndex = Int32(currentPointIndex)
        
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
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.capturedImageTextureY!), index: Int(kTextureY.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.capturedImageTextureCbCr!), index: Int(kTextureCbCr.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.depthTexture!), index: Int(kTextureDepth.rawValue))
        renderEncoder.setVertexTexture(CVMetalTextureGetTexture(textureManager.confidenceTexture!), index: Int(kTextureConfidence.rawValue))
        renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: gridPointsBuffer.count)
        
        currentPointIndex = (currentPointIndex + gridPointsBuffer.count) % maxPoints
        currentPointCount = min(currentPointCount + gridPointsBuffer.count, maxPoints)
        lastCameraTransform = frame.camera.transform
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
        guard let vertexFunction = library.makeFunction(name: "rgbVertex"),
            let fragmentFunction = library.makeFunction(name: "rgbFragment") else {
                return nil
        }
        
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertexFunction
        descriptor.fragmentFunction = fragmentFunction
        descriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        descriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        
        return try? device.makeRenderPipelineState(descriptor: descriptor)
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
        let gridArea = cameraResolution.x * cameraResolution.y
        let spacing = sqrt(gridArea / Float(numGridPoints))
        let deltaX = Int(round(cameraResolution.x / spacing))
        let deltaY = Int(round(cameraResolution.y / spacing))
        
        var points = [Float2]()
        for gridY in 0 ..< deltaY {
            let alternatingOffsetX = Float(gridY % 2) * spacing / 2
            for gridX in 0 ..< deltaX {
                let cameraPoint = Float2(alternatingOffsetX + (Float(gridX) + 0.5) * spacing, (Float(gridY) + 0.5) * spacing)
                
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
        // flip to ARKit Camera's coordinate
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
    /// Save all particles to a point cloud file in ply format using PLYExporter
    func savePointCloud() {
        delegate?.startMakingPlyFile()
        DispatchQueue.global().async { [weak self] in
            guard let self = self else { return }
            
            Task.init(priority: .utility) {
                // Use PLYExporter for cleaner code separation
                let plyString = PLYExporter.exportToPLYString(
                    particlesBuffer: self.particlesBuffer,
                    pointCount: self.currentPointCount
                )
                
                self.lidarRawStringData = plyString
            }
        }
    }
}

extension Renderer {
    /// 재측정을 위한 Renderer 초기화 함수
    /// https://github.com/ryanphilly/IOS-PointCloud 코드 참고
    func clearParticles() {
        self.clearing = true
        self.currentPointIndex = 0
        self.currentPointCount = 0
        
        self.rgbUniformsBuffers = [MetalBuffer<RGBUniforms>]()
        self.pointCloudUniformsBuffers = [MetalBuffer<PointCloudUniforms>]()
        
        self.commandQueue = device.makeCommandQueue()!
        for _ in 0 ..< maxInFlightBuffers {
            self.rgbUniformsBuffers.append(.init(device: device, count: 1, index: 0))
            self.pointCloudUniformsBuffers.append(.init(device: device, count: 1, index: kPointCloudUniforms.rawValue))
        }
        self.particlesBuffer = .init(device: device, count: maxPoints, index: kParticleUniforms.rawValue)
        self.clearing = false
    }
}
