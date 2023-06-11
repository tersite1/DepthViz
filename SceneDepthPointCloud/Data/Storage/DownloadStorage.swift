//
//  DownloadStorage.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/06/11.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class DownloadStorage {
    private init() { }
    
    static let url = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("download", isDirectory: false)
    
    static func getURL(fileName: String) -> URL? {
        let url = url.appendingPathComponent(fileName)
        if FileManager.default.fileExists(atPath: url.path) {
            return url
        } else {
            return nil
        }
    }
    
    static func remove(fileName: String) -> Bool {
        do {
            let url = url.appendingPathComponent(fileName)
            try FileManager.default.removeItem(at: url)
            return true
        } catch let error {
            print("---> Failed to remove msg: \(error.localizedDescription)")
            return false
        }
    }
}
