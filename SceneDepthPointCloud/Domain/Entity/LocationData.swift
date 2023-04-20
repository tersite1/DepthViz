//
//  LocationData.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/20.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import CoreLocation

/// 위치정보 데이터 구조체
struct LocationData {
    let latitude: CLLocationDegrees
    let longitude: CLLocationDegrees
    let altitude: CLLocationDistance
    let floor: Int?
    var roadAddressName: String?
    
    init(location: CLLocation) {
        self.latitude = location.coordinate.latitude
        self.longitude = location.coordinate.longitude
        self.altitude = location.altitude
        self.floor = location.floor?.level
    }
    
    mutating func updateRoadAddress(to address: String) {
        self.roadAddressName = address
    }
}
