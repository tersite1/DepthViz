//
//  BuildingNearByGpsDTO.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// BuildingApiService로부터 수신받은 Building 데이터 구조체
struct BuildingNearByGpsDTO: Decodable {
    let meta: BuildingNearByGpsMetaDTO
    let documents: [BuildingNearByGpsDocumentDTO]
}

struct BuildingNearByGpsMetaDTO: Decodable {
    let isEnd: Bool
    let pageableCount: Int
    let totalCount: Int
    
    enum CodingKeys: String, CodingKey {
        case isEnd = "is_end"
        case pageableCount = "pageable_count"
        case totalCount = "total_count"
    }
}

struct BuildingNearByGpsDocumentDTO: Decodable {
    let roadAddressName: String // 도로명주소
    let placeName: String // 건물명
    let distance: String // 거리(미터)
    let addressName: String
    let categoryGroupCode: String
    let categoryGroupName: String
    let categoryName: String
    let id: String
    let phone: String
    let placeUrl: String
    let longitude: String
    let latitude: String
    
    enum CodingKeys: String, CodingKey {
        case roadAddressName = "road_address_name"
        case placeName = "place_name"
        case distance = "distance"
        case addressName = "address_name"
        case categoryGroupCode = "category_group_code"
        case categoryGroupName = "category_group_name"
        case categoryName = "category_name"
        case id = "id"
        case phone = "phone"
        case placeUrl = "place_url"
        case longitude = "x"
        case latitude = "y"
    }
    
}
