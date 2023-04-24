//
//  BuildingInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Building 정보 데이터 구조체
struct BuildingInfo: Hashable {
    let roadAddress: String // 도로명주소
    let placeName: String // 장소명(건물명)
    let distance: Int // 거리(m단위)
    let addressName: String
    let categoryGroupCode: String
    let categoryGroupName: String
    let categoryName: String
    let id: String
    let phone: String
    let placeUrl: String
    let longitude: String
    let latitude: String
    
    static func == (lhs: BuildingInfo, rhs: BuildingInfo) -> Bool {
        return lhs.id == rhs.id
    }
}
