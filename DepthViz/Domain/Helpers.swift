//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import ARKit

typealias Float2 = SIMD2<Float>
typealias Float3 = SIMD3<Float>

extension Float {
    static let degreesToRadian = Float.pi / 180
}

extension matrix_float3x3 {
    mutating func copy(from affine: CGAffineTransform) {
        columns.0 = Float3(Float(affine.a), Float(affine.c), Float(affine.tx))
        columns.1 = Float3(Float(affine.b), Float(affine.d), Float(affine.ty))
        columns.2 = Float3(0, 0, 1)
    }
}
