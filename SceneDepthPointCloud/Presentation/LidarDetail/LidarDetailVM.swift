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
    @Published private(set) var networkError: (title: String, text: String)?
    private let collectId: String
    private let gpsId: String
    private let addressId: String
    private(set) var buildingInfo: BuildingInfo?
    private let buildingRepository: BuildingRepositoryInterface
    
    init(lidarInfo: LidarInfo, buildingRepository: BuildingRepositoryInterface) {
        self.collectId = lidarInfo.collectId
        self.gpsId = lidarInfo.gpsId
        self.addressId = lidarInfo.addressId
        
        self.buildingRepository = buildingRepository
    }
}

// MARK: INPUT
extension LidarDetailVM {
    func fetchDetailInfos() {
        self.getAddressInfo()
    }
}

extension LidarDetailVM {
    private func getLidarDetailInfo() {
        
    }
    
    private func getGpsInfo() {
        
    }
    
    private func getAddressInfo() {
        self.buildingRepository.fetchBuildingInfo(addressId: self.addressId) { [weak self] result in
            switch result {
            case .success(let buildingInfo):
                self?.buildingInfo = buildingInfo
            case.failure(let fetchError):
                self?.networkError = (title: "Fetch BuildingInfo Error", text: fetchError.message)
            }
        }
    }
}
