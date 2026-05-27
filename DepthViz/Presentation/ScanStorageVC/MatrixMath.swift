import simd

extension matrix_float4x4 {
    init(scale: Float) {
        self = matrix_float4x4(diagonal: vector_float4(scale, scale, scale, 1.0))
    }
    
    init(rotationAbout axis: vector_float3, by angleRadians: Float) {
        let c = cos(angleRadians)
        let s = sin(angleRadians)
        
        let axis = normalize(axis)
        let x = axis.x, y = axis.y, z = axis.z
        self.init(columns: (
            vector_float4(c + (1-c)*x*x,     (1-c)*x*y + s*z, (1-c)*x*z - s*y, 0),
            vector_float4((1-c)*x*y - s*z,   c + (1-c)*y*y,   (1-c)*y*z + s*x, 0),
            vector_float4((1-c)*x*z + s*y,   (1-c)*y*z - s*x, c + (1-c)*z*z,   0),
            vector_float4(0,                 0,               0,               1)
        ))
    }
}
