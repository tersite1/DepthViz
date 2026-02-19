//
//  MetalTextureManager.swift
//  DepthViz
//
//  Manages Metal textures for AR frame processing
//

import Foundation
import Metal
import ARKit
import CoreVideo

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
        guard CVPixelBufferGetPlaneCount(pixelBuffer) >= 2 else {
            return
        }
        
        capturedImageTextureY = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .r8Unorm, planeIndex: 0)
        capturedImageTextureCbCr = makeTexture(fromPixelBuffer: pixelBuffer, pixelFormat: .rg8Unorm, planeIndex: 1)
    }
    
    /// Update depth and confidence textures from AR frame
    /// smoothedSceneDepth 우선 사용 — Apple temporal filtering으로 multipath ghost 억제
    func updateDepthTextures(frame: ARFrame) -> Bool {
        // smoothedSceneDepth: 여러 프레임 temporal 평균 → multipath 노이즈 감소
        // fallback: sceneDepth (raw, 첫 몇 프레임에서 smoothed 아직 미준비 시)
        let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth
        guard let depthMap = depthData?.depthMap,
              let confidenceMap = depthData?.confidenceMap else {
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

