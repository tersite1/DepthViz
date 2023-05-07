//
//  LidarDetailVM.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/07.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import Combine

final class LidarDetailVM {
    private let collectId: String
    private let gpsId: String
    private let addressId: String
    
    init(lidarInfo: LidarInfo) {
        self.collectId = lidarInfo.collectId
        self.gpsId = lidarInfo.gpsId
        self.addressId = lidarInfo.addressId
        
        print(collectId)
        print(gpsId)
        print(addressId)
    }
}
