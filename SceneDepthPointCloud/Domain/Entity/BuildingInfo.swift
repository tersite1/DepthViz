//
//  BuildingInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Building 정보 데이터 구조체
struct BuildingInfo {
    let roadAddress: String
    let placeName: String
    let distance: Int
    let addressName: String
    let categoryGroupCode: String
    let categoryGroupName: String
    let categoryName: String
    let id: String
    let phone: String
    let placeUrl: String
    let longitude: String
    let latitude: String
    
    init(dto: BuildingNearByGpsDocumentDTO) {
        self.roadAddress = dto.roadAddressName
        self.placeName = dto.placeName
        self.distance = Int(dto.distance) ?? 0
        self.addressName = dto.addressName
        self.categoryGroupCode = dto.categoryGroupCode
        self.categoryGroupName = dto.categoryGroupName
        self.categoryName = dto.categoryName
        self.id = dto.id
        self.phone = dto.phone
        self.placeUrl = dto.placeUrl
        self.longitude = dto.longitude
        self.latitude = dto.latitude
    }
}
