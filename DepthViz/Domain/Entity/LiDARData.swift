//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import Foundation

/// LiDAR 데이터 구조체
struct LiDARData: Codable {
    // LiDAR 데이터 파일명
    var lidarFileName: String
    // LiDAR 데이터
    let lidarData: Data
    // LiDAR 데이터 파일크기
    let lidarFileSize: String
    // LiDAR 데이터 point 개수
    let pointCount: Int
    
    init(rawStringData: String, pointCount: Int) {
        self.lidarFileName = "\(getTimeStr()).ply"
        
        if let data = rawStringData.data(using: .utf8) {
            self.lidarData = data
            let bcf = ByteCountFormatter()
            bcf.allowedUnits = [.useMB]
            bcf.countStyle = .file
            self.lidarFileSize = bcf.string(fromByteCount: Int64(data.count))
        } else {
            print("/// lidarData is nil")
            self.lidarData = Data()
            self.lidarFileSize = "0 MB"
        }
        
        self.pointCount = pointCount
    }
    
    // MARK: Filename 변경
    mutating func rename(to name: String) {
        self.lidarFileName = "\(name).ply"
    }
}
