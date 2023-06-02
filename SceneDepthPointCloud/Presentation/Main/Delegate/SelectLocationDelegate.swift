//
//  SelectLocationDelegate.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

protocol SelectLocationDelegate: AnyObject {
    func uploadCancel()
    func uploadMeasuredData(location: LocationData, buildingInfo: BuildingOfMapInfo, floor: Int, lidarName: String)
}
