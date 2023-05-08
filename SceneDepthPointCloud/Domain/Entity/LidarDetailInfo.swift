//
//  LidarDetailInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct LidarDetailInfo {
    let collectId: String
    let generalFileId: String
    let fileSize: String
    let originFileName: String
    let floor: Int
    let createdDate: String
    let totalPoints: Int
    let latitude: Double
    let longitude: Double
    let altitude: Double?
}
