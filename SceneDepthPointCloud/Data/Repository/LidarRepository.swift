//
//  LidarRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class LidarRepository: LidarRepositoryInterface {
    func fetchLidarList(page: Int, completion: @escaping (Result<(infos: [LidarInfo], isLastPage: Bool), FetchError>) -> Void) {
        let endpoint = LidarApiService()
        
        endpoint.getLidarList(page: page) { result in
            switch result {
            case.success(let lidarInfoDTO):
                let infos = lidarInfoDTO.resultList.map { $0.toDomain() }
                let isLastPage = page == lidarInfoDTO.totalMap.totalPage
                completion(.success((infos: infos, isLastPage: isLastPage)))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
    }
    
    func fetchLidarDetailInfo(collectId: String, completion: @escaping (Result<LidarDetailInfo, FetchError>) -> Void) {
        let endpoint = LidarApiService()
        
        endpoint.getLidarDetailInfo(collectId: collectId) { result in
            switch result {
            case .success(let lidarDetailDTO):
                completion(.success(lidarDetailDTO.resultObject.toDomain()))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
    }
    
    func deleteLidar(collectId: String, completion: @escaping (Result<Bool, FetchError>) -> Void) {
        let endpoint = LidarApiService()
        
        endpoint.deleteLidar(collectId: collectId) { result in
            switch result {
            case .success(let success):
                completion(.success(success))
                
            case .failure(let error):
                completion(.failure(error))
            }
        }
    }
    
    // MARK: 메모리 문제 및 업로드 문제로 인해 앱 내부에 저장하는 기능 추가
    func saveToStorage(lidarData: LiDARData) -> Bool {
        return LidarStorage.save(lidarData)
    }
    
    func getFromStorage() -> LiDARData? {
        return LidarStorage.get()
    }
    
    func clearStorage() {
        LidarStorage.remove()
    }
}
