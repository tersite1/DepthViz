//
//  LidarDetailInfoDTO.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct LidarDetailInfoDTO: Decodable {
    let resultObject: LidarDetailInfoResultDTO
}

struct LidarDetailInfoResultDTO: Decodable {
    let collectId: String
    let generalFileId: String
    let fileSize: Int
    let originalFileName: String
    let floor: Int
    let createdDate: String
    let totalPoints: Int?
}
