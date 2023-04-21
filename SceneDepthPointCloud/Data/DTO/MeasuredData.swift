//
//  MeasuredData.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/28.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct MeasuredData: Codable {
    let collectID: UUID
    let lidar: Data
    let lidarFileName: String
    
    init(lidar: Data, fileName: String) {
        self.collectID = UUID.init()
        self.lidar = lidar
        self.lidarFileName = fileName
    }
}
