//
//  BuildingInfoDTO.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct BuildingInfoDTO: Decodable {
    let resultObject: BuildingInfoResultDTO
}

struct BuildingInfoResultDTO: Decodable {
    let id: String
    let placeName: String
    let categoryName: String
    let addressName: String
    let roadAddressName: String
    let placeURL: String
    
    enum CodingKeys: String, CodingKey {
        case id
        case placeName = "place_name"
        case categoryName = "category_name"
        case addressName = "address_name"
        case roadAddressName = "road_address_name"
        case placeURL = "place_url"
    }
    
    func toDomain() -> BuildingInfo {
        return BuildingInfo(id: id, placeName: placeName, categoryName: categoryName, addressName: addressName, roadAddressName: roadAddressName, placeURL: placeURL)
    }
}
