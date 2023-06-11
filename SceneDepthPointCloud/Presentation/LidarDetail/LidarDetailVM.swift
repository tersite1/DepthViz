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
    @Published private(set) var deleteCompleted: Bool?
    @Published private(set) var downloadProgress: Double = 0
    @Published private(set) var downloadedURL: URL?
    private(set) var fileName: String = ""
    private(set) var serverError: String = ""
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
        self.lidarRepository.deleteLidar(collectId: self.collectId) { [weak self] result in
            switch result {
            case .success(let success):
                self?.deleteCompleted = success
            case .failure(let fetchError):
                self?.serverError = fetchError.message
                self?.deleteCompleted = false
            }
        }
    }
    
    func downloadPCD() {
        guard let fileName = self.lidarDetailInfo?.originFileName,
              let fileId = self.lidarDetailInfo?.generalFileId,
              fileName.hasSuffix(".pcd") else { return }
        
        self.lidarRepository.downloadLidarFile(fileName: fileName, fileId: fileId, isPLY: false) { [weak self] progress in
            self?.downloadProgress = progress
        } completion: { [weak self] result in
            switch result {
            case .success(_):
                self?.fileName = fileName
                self?.downloadedURL = DownloadStorage.url.appendingPathComponent(fileName)
            case .failure(let fetchError):
                self?.networkError = (title: "다운로드 실패", text: fetchError.message)
            }
        }
    }
    
    func downloadPLY() {
        guard var fileName = self.lidarDetailInfo?.originFileName,
              let fileId = self.lidarDetailInfo?.generalFileId,
              fileName.hasSuffix(".pcd") else { return }
        
        guard let name = fileName.split(separator: ".").first else { return }
        fileName = "\(name).ply"
        
        self.lidarRepository.downloadLidarFile(fileName: fileName, fileId: fileId, isPLY: true) { [weak self] progress in
            self?.downloadProgress = progress
        } completion: { [weak self] result in
            switch result {
            case .success(_):
                self?.fileName = fileName
                self?.downloadedURL = DownloadStorage.url.appendingPathComponent(fileName)
            case .failure(let fetchError):
                self?.networkError = (title: "다운로드 실패", text: fetchError.message)
            }
        }
    }
    
    func removeDownloaded() {
        if DownloadStorage.remove(fileName: self.fileName) == false {
            self.networkError = (title: "파일삭제 실패", text: "")
        }
        self.downloadProgress = 0
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
