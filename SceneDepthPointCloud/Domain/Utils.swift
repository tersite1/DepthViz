//
//  Utils.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/09.
//  Copyright © 2023 Apple. All rights reserved.
//

// MARK: Created by Waley Zheng on 10/2/22.
import Foundation
import UIKit
import VideoToolbox

/// Get current time in string.
func getTimeStr() -> String {
    let df = DateFormatter()
    df.dateFormat = "yyyy-MM-dd-HHmm"
    return df.string(from: Date())
}

/// Save file to a directory.
func saveFile(content: String, filename: String, folder: String) async throws -> () {
    print("Save file to \(folder)/\(filename)")
    let url = getDocumentsDirectory().appendingPathComponent(folder, isDirectory: true).appendingPathComponent(filename)
    try content.write(to: url, atomically: true, encoding: .utf8)
}

func shareFile(content: String, filename: String, folder: String) -> URL? {
    print("SHARE file to \(folder)/\(filename)")
    let url = getDocumentsDirectory().appendingPathComponent(folder, isDirectory: true).appendingPathComponent(filename)
    do {
        try content.write(to: url, atomically: true, encoding: .ascii)
        return url
    } catch {
        print(error.localizedDescription)
        return nil
    }
}

/// Save jpeg to a directory.
func savePic(pic: UIImage, filename: String, folder: String) async throws -> () {
    print("Save picture to \(folder)/\(filename)")
    let url = getDocumentsDirectory().appendingPathComponent(folder, isDirectory: true).appendingPathComponent(filename)
    try pic.jpegData(compressionQuality: 0)?.write(to: url)
}

/// Transform cvPixelBuffer of datatype <T> to a 2D array map.
func cvPixelBuffer2Map<T : Numeric>(rawDepth: CVPixelBuffer) async -> [[T]] {
    CVPixelBufferLockBaseAddress(rawDepth, CVPixelBufferLockFlags(rawValue: 0))
    let addr = CVPixelBufferGetBaseAddress(rawDepth)
    let height = CVPixelBufferGetHeight(rawDepth)
    let width = CVPixelBufferGetWidth(rawDepth)
    
    let TBuffer = unsafeBitCast(addr, to: UnsafeMutablePointer<T>.self)
    
    var TMap : [[T]] = Array(repeating: Array(repeating: T(exactly: 0)!, count: width), count: height)
    
    for row in 0...(height - 1){
        for col in 0...(width - 1){
            TMap[row][col] = TBuffer[row * width + col]
        }
    }
    CVPixelBufferUnlockBaseAddress(rawDepth, CVPixelBufferLockFlags(rawValue: 0))
    return TMap
}

/// Transform cvPixelBuffer to a UIImage.
func cvPixelBuffer2UIImage(pixelBuffer: CVPixelBuffer) -> UIImage {
    var cgImage: CGImage?
    VTCreateCGImageFromCVPixelBuffer(pixelBuffer, options: nil, imageOut: &cgImage)
    guard let cgImage = cgImage else {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        print("error transforming CVPixelBuffer to UIImage using VTCreateCGImageFromCVPixelBuffer")
        return UIImage(ciImage: ciImage)
    }
    return UIImage(cgImage: cgImage)
}

func getDocumentsDirectory() -> URL {
    // find all possible documents directories for this user
    let paths = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)
    
    // just send back the first one, which ought to be the only one
    return paths[0]
}

// MARK: 임시 로직
func getDocuments() {
    let fm = FileManager.default
    let documentPath = fm.urls(for: .documentDirectory, in: .userDomainMask)[0].path
    do {
        let documentContents = try FileManager.default.contentsOfDirectory(atPath: documentPath)
        for content in documentContents {
            print("Content: \(content)")
        }
    } catch {
        print(error.localizedDescription)
    }
//
//    // List all contents of directory and return as [String] OR nil if failed
//    do {
//        for path in try fileMngr.contentsOfDirectory(atPath:docs) {
//            print("file: \(path)")
//        }
//    } catch {
//        print(error.localizedDescription)
//    }
}

func createDirectory(folder: String) {
    let path = getDocumentsDirectory().appendingPathComponent(folder)
    do
    {
        try FileManager.default.createDirectory(at: path, withIntermediateDirectories: true)
    }
    catch let error as NSError
    {
        print("Unable to create directory \(error.debugDescription)")
    }
    
}

/// https://stackoverflow.com/questions/63661474/how-can-i-encode-an-array-of-simd-float4x4-elements-in-swift-convert-simd-float
extension simd_float4x4: Codable {
    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        try self.init(container.decode([SIMD4<Float>].self))
    }
    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        try container.encode([columns.0, columns.1, columns.2, columns.3])
    }
}

extension simd_float3x3: Codable {
    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        try self.init(container.decode([SIMD3<Float>].self))
    }
    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        try container.encode([columns.0, columns.1, columns.2])
    }
}

/// Send task start/finish messages.
// TODO: 구조수정 필요 부분
protocol TaskDelegate: AnyObject {
    func didStartTask()
    func didFinishTask()
    func sharePLY(file: Any)
    func showUploadResult(result: NetworkResult)
}

/// Deep copy CVPixelBuffer for depth data
/// https://stackoverflow.com/questions/65868215/deep-copy-cvpixelbuffer-for-depth-data-in-swift
func duplicatePixelBuffer(input: CVPixelBuffer) -> CVPixelBuffer {
    var copyOut: CVPixelBuffer?
    let bufferWidth = CVPixelBufferGetWidth(input)
    let bufferHeight = CVPixelBufferGetHeight(input)
    let bytesPerRow = CVPixelBufferGetBytesPerRow(input)
    let bufferFormat = CVPixelBufferGetPixelFormatType(input)
    
    _ = CVPixelBufferCreate(kCFAllocatorDefault, bufferWidth, bufferHeight, bufferFormat, CVBufferGetAttachments(input, CVAttachmentMode.shouldPropagate), &copyOut)
    let output = copyOut!
    // Lock the depth map base address before accessing it
    CVPixelBufferLockBaseAddress(input, CVPixelBufferLockFlags.readOnly)
    CVPixelBufferLockBaseAddress(output, CVPixelBufferLockFlags(rawValue: 0))
    let baseAddress = CVPixelBufferGetBaseAddress(input)
    let baseAddressCopy = CVPixelBufferGetBaseAddress(output)
    memcpy(baseAddressCopy, baseAddress, bufferHeight * bytesPerRow)
    
    // Unlock the base address when finished accessing the buffer
    CVPixelBufferUnlockBaseAddress(input, CVPixelBufferLockFlags.readOnly)
    CVPixelBufferUnlockBaseAddress(output, CVPixelBufferLockFlags(rawValue: 0))
    return output
}
