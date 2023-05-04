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
        // MARK: Fake 로직
        var fakeInfos: [LidarInfo] = []
        fakeInfos.append(LidarInfo(collectId: "1", buildingName: "인하대학교 하이테크센터", floor: 1, roadAddres: "인천 미추홀구 인하로 100", createdAt: Date(), fileSize: "19.3 MB"))
        fakeInfos.append(LidarInfo(collectId: "2", buildingName: "인하대학교 하이테크센터", floor: 2, roadAddres: "인천 미추홀구 인하로 100", createdAt: Date(), fileSize: "20.3 MB"))
        
        completion(.success((infos: fakeInfos, isLastPage: true)))
    }
}
