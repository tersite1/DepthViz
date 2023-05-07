//
//  LidarInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct LidarInfo: Hashable {
    let collectId: String
    let buildingName: String
    let floor: Int
    let roadAddres: String
    let createdAt: String
    let fileSize: String
    
    static func == (lhs: LidarInfo, rhs: LidarInfo) -> Bool {
        return lhs.collectId == rhs.collectId
    }
}
