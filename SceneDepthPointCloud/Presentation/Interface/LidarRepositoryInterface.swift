//
//  LidarRepositoryInterface.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/04.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

protocol LidarRepositoryInterface: AnyObject {
    func fetchLidarList(page: Int, completion: @escaping(Result<(infos: [LidarInfo], isLastPage: Bool), FetchError>) -> Void)
}
