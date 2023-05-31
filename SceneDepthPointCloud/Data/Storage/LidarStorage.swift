//
//  LidarStorage.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/29.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class LidarStorage {
    private init() { }
    
    static let url = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("lidarData", isDirectory: false)
    
    static func save(_ obj: LiDARData) -> Bool {
        print("---> save to here: \(url)")
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        encoder.dateEncodingStrategy = .iso8601
        
        do {
            let data = try encoder.encode(obj)
            //동일이름의 파일이 있는 경우 삭제
            if FileManager.default.fileExists(atPath: url.path) {
                try FileManager.default.removeItem(at: url)
            }
            //파일을 저장
            FileManager.default.createFile(atPath: url.path, contents: data, attributes: nil)
            return true
        } catch let error {
            print("---> Failed to store msg: \(error.localizedDescription)")
            return false
        }
    }
    
    static func get() -> LiDARData? {
        guard FileManager.default.fileExists(atPath: url.path) else { return nil }
        guard let data = FileManager.default.contents(atPath: url.path) else { return nil }
        
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        
        do {
            let lidarData = try decoder.decode(LiDARData.self, from: data)
            return lidarData
        } catch let error {
            print("---> Failed to decode msg: \(error.localizedDescription)")
            return nil
        }
    }
    
    static func remove() {
        do {
            try FileManager.default.removeItem(at: url)
        } catch let error {
            print("---> Failed to remove msg: \(error.localizedDescription)")
        }
    }
}
