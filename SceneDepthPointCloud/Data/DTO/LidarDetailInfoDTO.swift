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
    
    func toDomain() -> LidarDetailInfo {
        let fileSize = String(format: "%.1f MB", fileSize)
        let totalPoints = self.totalPoints != nil ? self.totalPoints! : 0
        
        return LidarDetailInfo(collectId: collectId, generalFileId: generalFileId, fileSize: fileSize, originalFileName: originalFileName, floor: floor, createdDate: createdDate, totalPoints: totalPoints)
    }
}
