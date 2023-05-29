//
//  LocationRepositoryInterface.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/29.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

protocol LocationRepositoryInterface: AnyObject {
    func saveToStorage(locationData: LocationData) -> Bool
    func getFromStorage() -> LocationData?
    func clearStorage()
}
