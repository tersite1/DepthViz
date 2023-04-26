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

struct BuildingNearByGpsDocumentDTO: Codable, Hashable {
    let road_address_name: String // 도로명주소
    let place_name: String // 건물명
    let distance: String // 거리(미터)
    let address_name: String
    let category_group_code: String
    let category_group_name: String
    let category_name: String
    let id: String
    let phone: String
    let place_url: String
    let x: String
    let y: String
    
    enum CodingKeys: String, CodingKey {
        case road_address_name = "road_address_name"
        case place_name = "place_name"
        case distance = "distance"
        case address_name = "address_name"
        case category_group_code = "category_group_code"
        case category_group_name = "category_group_name"
        case category_name = "category_name"
        case id = "id"
        case phone = "phone"
        case place_url = "place_url"
        case x = "x"
        case y = "y"
    }
    
    func toDomain() -> BuildingInfo {
        return .init(roadAddress: self.road_address_name,
                     placeName: self.place_name,
                     distance: Int(self.distance) ?? 0,
                     dto: self)
    }
}
