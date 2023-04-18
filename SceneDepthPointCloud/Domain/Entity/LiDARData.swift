//
//  LiDARData.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/18.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// LiDAR 데이터 구조체
struct LiDARData {
    // LiDAR 데이터 파일명
    let lidarFileName: String
    // LiDAR 데이터
    let lidarData: Data?
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
            self.lidarData = nil
            self.lidarFileSize = "0 MB"
        }
        
        self.pointCount = pointCount
    }
}
