//
//  BuildingRepositoryInterface.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/08.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

protocol BuildingRepositoryInterface: AnyObject {
    func fetchBuildingInfo(addressId: String, completion: @escaping (Result<BuildingInfo, FetchError>) -> Void)
}
