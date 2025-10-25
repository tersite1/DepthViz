//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import Foundation

/// ScanStorage에 저장되는 스캔된 Point Cloud Data 정보
struct ScanData: Codable {
    let date: Date
    var fileName: String
    let lidarData: Data
    let fileSize: String
    let points: Int
    
    
    init(rawStringData: String, pointCount: Int) {
        let now = Date()
        self.date = now
        self.fileName = "\(now.yyyyMMddHHmmss).ply"
        guard let plyData = rawStringData.data(using: .utf8) else {
            self.lidarData = Data()
            self.fileSize = "0 MB"
            self.points = 0
            return
        }
        
        self.lidarData = plyData
        self.fileSize = plyData.fileSize
        self.points = pointCount
    }
    
    // MARK: Filename 변경
    mutating func rename(to name: String) {
        self.fileName = "\(name).ply"
    }
    
    var info: ScanInfo {
        return ScanInfo(id: self.fileName, date: self.date, fileName: self.fileName, fileSize: self.fileSize, points: self.points)
    }
}
