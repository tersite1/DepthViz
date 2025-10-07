import Foundation
import simd

struct AccumulatedFrameData: Codable {
    struct Metadata: Codable {
        let index: Int
        let timestamp: TimeInterval
        let cameraPose: simd_float4x4
        let cameraIntrinsics: simd_float3x3
        let viewMatrix: simd_float4x4
        let projectionMatrix: simd_float4x4
        let imageFileName: String
    }

    let index: Int
    let timestamp: TimeInterval
    let cameraPose: simd_float4x4
    let cameraIntrinsics: simd_float3x3
    let viewMatrix: simd_float4x4
    let projectionMatrix: simd_float4x4
    let capturedImagePNGData: Data

    func metadataRecord() -> Metadata {
        Metadata(
            index: index,
            timestamp: timestamp,
            cameraPose: cameraPose,
            cameraIntrinsics: cameraIntrinsics,
            viewMatrix: viewMatrix,
            projectionMatrix: projectionMatrix,
            imageFileName: Self.imageFileName(for: index)
        )
    }

    static func imageFileName(for index: Int) -> String {
        "frame_\(index).png"
    }

    static func metadataFileName(for index: Int) -> String {
        "frame_\(index).json"
    }

    /// Saves the image and metadata to the specified directory.
    /// - Parameter directory: The directory URL where files will be saved.
    /// - Throws: An error if writing fails.
    func save(to directory: URL) throws {
        // Save image
        let imageURL = directory.appendingPathComponent(Self.imageFileName(for: index))
        try capturedImagePNGData.write(to: imageURL)

        // Save metadata (excluding image data)
        let metadata = metadataRecord()
        let metadataURL = directory.appendingPathComponent(Self.metadataFileName(for: index))
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        let metadataData = try encoder.encode(metadata)
        try metadataData.write(to: metadataURL)
    }
}
