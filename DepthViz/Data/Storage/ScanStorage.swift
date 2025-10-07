//
//  LidarStorage.swift
//  DepthViz
//
//  Created by Construction Management on 2023/05/29.
//  Copyright © 2023 Apple. All rights reserved.
//
//

import Foundation

/// Storage for scanned Point Cloud Data
final class ScanStorage: ObservableObject {
    static let shared = ScanStorage()
    
    let infosRoot = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("scanInfos", isDirectory: true)
    let filesRoot = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("scanFiles", isDirectory: true)
    
    @Published var infos: [ScanInfo] = [] {
        didSet {
            print(infos.count)
        }
    }
    
    private init() {
        self.updateInfos()
    }
    
    private func updateInfos() {
        do {
            if FileManager.default.fileExists(atPath: self.infosRoot.path) == false {
                self.createDirectory()
            }
            
            let urls = try FileManager.default.contentsOfDirectory(at: infosRoot, includingPropertiesForKeys: nil)
            var infos: [ScanInfo] = []
            
            for url in urls {
                if let info = getInfo(url: url) {
                    infos.append(info)
                }
            }
            self.infos = infos
        } catch {
            print(error.localizedDescription)
        }
    }
    
    func createDirectory() {
        do {
            try FileManager.default.createDirectory(at: infosRoot, withIntermediateDirectories: true)
            try FileManager.default.createDirectory(at: filesRoot, withIntermediateDirectories: true)
        } catch {
            print(error.localizedDescription)
        }
    }
    
    func save(_ obj: ScanData) -> Bool {
        let infoUrl = infosRoot.appendingPathComponent(obj.fileName)
        let fileUrl = filesRoot.appendingPathComponent(obj.fileName)
        print("---> save to here: \(infoUrl.path)")
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        encoder.dateEncodingStrategy = .iso8601

        do {
            if FileManager.default.fileExists(atPath: infosRoot.path) == false {
                createDirectory()
            }

            let infoData = try encoder.encode(obj.info)
            // If a file with the same name exists, delete it
            if FileManager.default.fileExists(atPath: infoUrl.path) {
                try FileManager.default.removeItem(at: infoUrl)
            }
            // Save the file
            FileManager.default.createFile(atPath: infoUrl.path, contents: infoData, attributes: nil)

            if FileManager.default.fileExists(atPath: fileUrl.path) {
                try FileManager.default.removeItem(at: fileUrl)
            }

            FileManager.default.createFile(atPath: fileUrl.path, contents: obj.lidarData)

            // Save frame info if present
            if let frames = obj.frames {
                let frameDir = filesRoot.appendingPathComponent("\(obj.fileName)_frames", isDirectory: true)
                if !FileManager.default.fileExists(atPath: frameDir.path) {
                    try FileManager.default.createDirectory(at: frameDir, withIntermediateDirectories: true)
                }
                for frame in frames {
                    // Save metadata JSON
                    let metaFile = frameDir.appendingPathComponent(AccumulatedFrameData.metadataFileName(for: frame.index))
                    let metaData = try encoder.encode(frame.metadataRecord())
                    FileManager.default.createFile(atPath: metaFile.path, contents: metaData, attributes: nil)
                    // Save image PNG
                    let imageFile = frameDir.appendingPathComponent(AccumulatedFrameData.imageFileName(for: frame.index))
                    FileManager.default.createFile(atPath: imageFile.path, contents: frame.capturedImagePNGData, attributes: nil)
                }
            }

            self.updateInfos()
            return true
        } catch let error {
            print("---> Failed to store msg: \(error.localizedDescription)")
            return false
        }
    }
    
    func getInfo(url: URL) -> ScanInfo? {
        guard FileManager.default.fileExists(atPath: url.path) else { return nil }
        guard let data = FileManager.default.contents(atPath: url.path) else { return nil }
        
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        
        do {
            let scanInfo = try decoder.decode(ScanInfo.self, from: data)
            return scanInfo
        } catch let error {
            print("---> Failed to decode msg: \(error.localizedDescription)")
            return nil
        }
    }
    
    func remove(fileName: String) {
        do {
            let infoUrl = infosRoot.appendingPathComponent(fileName)
            try FileManager.default.removeItem(at: infoUrl)
            let fileUrl = filesRoot.appendingPathComponent(fileName)
            try FileManager.default.removeItem(at: fileUrl)
            
            self.updateInfos()
        } catch let error {
            print("---> Failed to remove msg: \(error.localizedDescription)")
        }
    }
    
    func fileUrl(fileName: String) -> URL {
        return filesRoot.appendingPathComponent(fileName)
    }
}
