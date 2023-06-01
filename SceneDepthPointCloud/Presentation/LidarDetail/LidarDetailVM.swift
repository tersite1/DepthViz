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
    @Published private(set) var infosDownloaded: Bool = false
    private let collectId: String
    let addressId: String
    private(set) var buildingInfo: BuildingInfo?
    private(set) var lidarDetailInfo: LidarDetailInfo?
    private let buildingRepository: BuildingRepositoryInterface
    private let lidarRepository: LidarRepositoryInterface
    
    init(lidarInfo: LidarInfo, buildingRepository: BuildingRepositoryInterface, lidarRepository: LidarRepositoryInterface) {
        self.collectId = lidarInfo.collectId
        self.addressId = lidarInfo.addressId
        
        self.buildingRepository = buildingRepository
        self.lidarRepository = lidarRepository
    }
}

// MARK: INPUT
extension LidarDetailVM {
    func fetchDetailInfos() {
        let group = DispatchGroup()
        group.enter()
        DispatchQueue.global().async(group: group) { [weak self] in
            self?.getAddressInfo(group: group)
        }
        group.enter()
        DispatchQueue.global().async(group: group) { [weak self] in
            self?.getLidarDetailInfo(group: group)
        }
        group.notify(queue: DispatchQueue.global()) { [weak self] in
            self?.infosDownloaded = true
        }
    }
    
    func deleteLidar() {
        print("deleteLidar")
    }
}

extension LidarDetailVM {
    private func getLidarDetailInfo(group: DispatchGroup) {
        self.lidarRepository.fetchLidarDetailInfo(collectId: self.collectId) { [weak self] result in
            switch result {
            case .success(let lidarDetailInfo):
                self?.lidarDetailInfo = lidarDetailInfo
            case .failure(let fetchError):
                self?.networkError = (title: "Fetch LidarDetailInfo Error", text: fetchError.message)
            }
            group.leave()
        }
    }
    
    private func getAddressInfo(group: DispatchGroup) {
        self.buildingRepository.fetchBuildingInfo(addressId: self.addressId) { [weak self] result in
            switch result {
            case .success(let buildingInfo):
                self?.buildingInfo = buildingInfo
            case.failure(let fetchError):
                self?.networkError = (title: "Fetch BuildingInfo Error", text: fetchError.message)
            }
            group.leave()
        }
    }
}
