//
//  LocationRepository.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/29.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

final class LocationRepository: LocationRepositoryInterface {
    // MARK: 메모리 문제 및 업로드 문제로 인해 앱 내부에 저장하는 기능 추가
    func saveToStorage(locationData: LocationData) -> Bool {
        return LocationStorage.save(locationData)
    }
    
    func getFromStorage() -> LocationData? {
        return LocationStorage.get()
    }
    
    func clearStorage() {
        LocationStorage.remove()
    }
}
