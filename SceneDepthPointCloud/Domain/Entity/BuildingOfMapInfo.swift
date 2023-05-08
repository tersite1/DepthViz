//
//  BuildingOfMapInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/24.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

/// Building 정보 데이터 구조체
struct BuildingOfMapInfo: Hashable {
    var uuid = UUID()
    let roadAddress: String // 도로명주소
    let placeName: String // 장소명(건물명)
    let distance: Int // 거리(m단위)
    let dto: BuildingNearByGpsDocumentDTO
    
    static func == (lhs: BuildingOfMapInfo, rhs: BuildingOfMapInfo) -> Bool {
        return lhs.uuid == rhs.uuid
    }
    
    mutating func updateUUID() {
        self.uuid = UUID()
    }
}
