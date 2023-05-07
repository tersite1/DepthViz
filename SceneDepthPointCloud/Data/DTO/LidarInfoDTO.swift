//
//  LidarInfoDTO.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/06.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct LidarInfoDTO: Decodable {
    let resultList: [LidarInfoResultDTO]
    let totalMap: PaginationInfoDTO
}

struct LidarInfoResultDTO: Decodable {
    let collectId: String
    let fileSize: Int
    let gpsId: String
    let addressId: String
    let roadAddressName: String
    let placeName: String
    let floor: Int
    let createdDate: String
}

struct PaginationInfoDTO: Decodable {
    let totalCount: Int
    let totalPage: Int
}

extension LidarInfoResultDTO {
    func toDomain() -> LidarInfo {
        let fileSize = Double(fileSize)/Double(1000000)
        return LidarInfo(collectId: collectId, buildingName: placeName, floor: floor, roadAddres: roadAddressName, createdAt: createdDate, fileSize: String(format: "%.1f MB", fileSize), gpsId: gpsId, addressId: addressId)
    }
}
