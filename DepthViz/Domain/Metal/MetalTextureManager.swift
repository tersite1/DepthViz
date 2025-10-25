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

