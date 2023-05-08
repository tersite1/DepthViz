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
    let originFileName: String
    let floor: Int
    let createdDate: String
    let totalPoints: Int?
    let latitude: String
    let longitude: String
    let altitude: String?
    
    func toDomain() -> LidarDetailInfo {
        let fileSize = String(format: "%.1f MB", Double(fileSize)/Double(1000000))
        let totalPoints = self.totalPoints != nil ? self.totalPoints! : 0
        let latitude = Double(latitude)!
        let longitude = Double(longitude)!
        let altitude = self.altitude != nil ? Double(altitude!) : nil
        
        return LidarDetailInfo(collectId: collectId, generalFileId: generalFileId, fileSize: fileSize, originFileName: originFileName, floor: floor, createdDate: createdDate, totalPoints: totalPoints, latitude: latitude, longitude: longitude, altitude: altitude)
    }
}
