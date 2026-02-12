//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


#include <metal_stdlib>
#include <simd/simd.h>
#import "ShaderTypes.h"

using namespace metal;

// Camera's RGB vertex shader outputs
struct RGBVertexOut {
    float4 position [[position]];
    float2 texCoord;
};

// Particle vertex shader outputs and fragment shader inputs
struct ParticleVertexOut {
    float4 position [[position]];
    float pointSize [[point_size]];
    float4 color;
};

constexpr sampler colorSampler(mip_filter::linear, mag_filter::linear, min_filter::linear);
constant auto yCbCrToRGB = float4x4(float4(+1.0000f, +1.0000f, +1.0000f, +0.0000f),
                                    float4(+0.0000f, -0.3441f, +1.7720f, +0.0000f),
                                    float4(+1.4020f, -0.7141f, +0.0000f, +0.0000f),
                                    float4(-0.7010f, +0.5291f, -0.8860f, +1.0000f));
constant float2 viewVertices[] = { float2(-1, 1), float2(-1, -1), float2(1, 1), float2(1, -1) };
constant float2 viewTexCoords[] = { float2(0, 0), float2(0, 1), float2(1, 0), float2(1, 1) };

/// Retrieves the world position of a specified camera point with depth
static simd_float4 worldPoint(simd_float2 cameraPoint, float depth, matrix_float3x3 cameraIntrinsicsInversed, matrix_float4x4 localToWorld) {
    const auto localPoint = cameraIntrinsicsInversed * simd_float3(cameraPoint, 1) * depth;
    const auto worldPoint = localToWorld * simd_float4(localPoint, 1);
    
    return worldPoint / worldPoint.w;
}

///  Vertex shader that takes in a 2D grid-point and infers its 3D position in world-space, along with RGB and confidence
///  Voxel occupancy check: skip points that fall in already-occupied voxels to prevent duplicate accumulation
vertex void unprojectVertex(uint vertexID [[vertex_id]],
                            constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                            device ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]],
                            constant float2 *gridPoints [[buffer(kGridPoints)]],
                            device atomic_uint *voxelGrid [[buffer(kVoxelOccupancy)]],
                            texture2d<float, access::sample> capturedImageTextureY [[texture(kTextureY)]],
                            texture2d<float, access::sample> capturedImageTextureCbCr [[texture(kTextureCbCr)]],
                            texture2d<float, access::sample> depthTexture [[texture(kTextureDepth)]],
                            texture2d<unsigned int, access::sample> confidenceTexture [[texture(kTextureConfidence)]]) {

    const auto gridPoint = gridPoints[vertexID];
    const auto currentPointIndex = (uniforms.pointCloudCurrentIndex + vertexID) % uniforms.maxPoints;
    const auto texCoord = gridPoint / uniforms.cameraResolution;
    // Sample the depth map to get the depth value
    const auto depth = depthTexture.sample(colorSampler, texCoord).r;
    // With a 2D point plus depth, we can now get its 3D position
    const auto position = worldPoint(gridPoint, depth, uniforms.cameraIntrinsicsInversed, uniforms.localToWorld);

    // Voxel occupancy check — hash world position to a voxel and skip if already occupied
    const float vs = uniforms.voxelSize;
    const int ix = int(floor(position.x / vs));
    const int iy = int(floor(position.y / vs));
    const int iz = int(floor(position.z / vs));
    const uint hash = (uint(ix) * 73856093u ^ uint(iy) * 19349663u ^ uint(iz) * 83492791u) % uint(uniforms.voxelGridSize);
    const uint prev = atomic_exchange_explicit(&voxelGrid[hash], 1u, memory_order_relaxed);
    if (prev != 0u) {
        // Voxel already occupied — mark point as invalid
        particleUniforms[currentPointIndex].confidence = -1.0;
        return;
    }

    // Sample Y and CbCr textures to get the YCbCr color at the given texture coordinate
    const auto ycbcr = float4(capturedImageTextureY.sample(colorSampler, texCoord).r, capturedImageTextureCbCr.sample(colorSampler, texCoord.xy).rg, 1);
    const auto sampledColor = (yCbCrToRGB * ycbcr).rgb;
    // Sample the confidence map to get the confidence value
    const auto confidence = confidenceTexture.sample(colorSampler, texCoord).r;

    // Write the data to the buffer
    particleUniforms[currentPointIndex].position = position.xyz;
    particleUniforms[currentPointIndex].color = sampledColor;
    particleUniforms[currentPointIndex].confidence = confidence;
}

vertex RGBVertexOut rgbVertex(uint vertexID [[vertex_id]],
                              constant RGBUniforms &uniforms [[buffer(0)]]) {
    const float3 texCoord = float3(viewTexCoords[vertexID], 1) * uniforms.viewToCamera;
    
    RGBVertexOut out;
    out.position = float4(viewVertices[vertexID], 0, 1);
    out.texCoord = texCoord.xy;
    
    return out;
}

fragment float4 rgbFragment(RGBVertexOut in [[stage_in]],
                            constant RGBUniforms &uniforms [[buffer(0)]],
                            texture2d<float, access::sample> capturedImageTextureY [[texture(kTextureY)]],
                            texture2d<float, access::sample> capturedImageTextureCbCr [[texture(kTextureCbCr)]]) {
    
    const float2 offset = (in.texCoord - 0.5) * float2(1, 1 / uniforms.viewRatio) * 2;
    const float visibility = saturate(uniforms.radius * uniforms.radius - length_squared(offset));
    const float4 ycbcr = float4(capturedImageTextureY.sample(colorSampler, in.texCoord.xy).r, capturedImageTextureCbCr.sample(colorSampler, in.texCoord.xy).rg, 1);
    
    // convert and save the color back to the buffer
    const float3 sampledColor = (yCbCrToRGB * ycbcr).rgb;
    return float4(sampledColor, 1) * visibility;
}

vertex ParticleVertexOut particleVertex(uint vertexID [[vertex_id]],
                                        constant PointCloudUniforms &uniforms [[buffer(kPointCloudUniforms)]],
                                        constant ParticleUniforms *particleUniforms [[buffer(kParticleUniforms)]]) {
    
    // get point data
    const auto particleData = particleUniforms[vertexID];
    const auto position = particleData.position;
    const auto confidence = particleData.confidence;
    const auto sampledColor = particleData.color;
    const auto visibility = confidence >= uniforms.confidenceThreshold;
    
    // animate and project the point
    float4 projectedPosition = uniforms.viewProjectionMatrix * float4(position, 1.0);

    // Improved point size calculation with better depth scaling
    const float pointSize = max(6.0 * uniforms.particleSize / max(1.0, projectedPosition.z), 1.5);

    projectedPosition /= projectedPosition.w;
    
    // prepare for output
    ParticleVertexOut out;
    out.position = projectedPosition;
    out.pointSize = pointSize;
    out.color = float4(sampledColor, visibility);
    
    return out;
}

fragment float4 particleFragment(ParticleVertexOut in [[stage_in]],
                                 const float2 coords [[point_coord]]) {
    // we draw within a circle with smooth edges for better visual quality
    const float2 centeredCoord = coords - float2(0.5);
    const float distSquared = length_squared(centeredCoord);
    
    if (in.color.a == 0 || distSquared > 0.25) {
        discard_fragment();
    }
    
    // Apply smooth alpha falloff for softer edges
    const float smoothRadius = 0.5;
    const float alpha = 1.0 - smoothstep(0.15, 0.25, distSquared);
    
    return float4(in.color.rgb, in.color.a * alpha);
}
