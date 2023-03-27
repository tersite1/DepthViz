//
//  MainApiService.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class MainApiService {
    /// ply 데이터를 서버로 전송
    func uploadPlyData(data: Data, completion: @escaping (NetworkResult) -> Void) {
        completion(NetworkResult(data: nil, status: .SUCCESS))
    }
}
