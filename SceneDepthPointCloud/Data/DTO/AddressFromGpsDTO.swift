//
//  AddressFromGpsDTO.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// AddressApiService로부터 수신받은 Address 데이터 구조체
struct AddressFromGpsDTO: Decodable {
    let documents: [AddressFromGpsDocumentDTO]
}

struct AddressFromGpsDocumentDTO: Decodable {
    let roadAddress: RoadAddressDTO
    let address: AddressDTO
    
    enum CodingKeys: String, CodingKey {
        case roadAddress = "road_address"
        case address = "address"
    }
}

struct RoadAddressDTO: Decodable {
    let addressName: String
    let region1DepthName: String
    let region2DepthName: String
    let region3DepthName: String
    let roadName: String
    let mainBuildingNo: String
    let buildingName: String
    let zoneNo: String
    
    enum CodingKeys: String, CodingKey {
        case addressName = "address_name"
        case region1DepthName = "region_1depth_name"
        case region2DepthName = "region_2depth_name"
        case region3DepthName = "region_3depth_name"
        case roadName = "road_name"
        case mainBuildingNo = "main_building_no"
        case buildingName = "building_name"
        case zoneNo = "zone_no"
    }
}

struct AddressDTO: Decodable {
    let addressName: String
    let region1DepthName: String
    let region2DepthName: String
    let region3DepthName: String
    let mainAddressNo: String
    let subAddressNo: String
    let zipCode: String
    
    enum CodingKeys: String, CodingKey {
        case addressName = "address_name"
        case region1DepthName = "region_1depth_name"
        case region2DepthName = "region_2depth_name"
        case region3DepthName = "region_3depth_name"
        case mainAddressNo = "main_address_no"
        case subAddressNo = "sub_address_no"
        case zipCode = "zip_code"
    }
}
