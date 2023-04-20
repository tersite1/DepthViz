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
    
    init(cllocation: CLLocation) {
        self.latitude = cllocation.coordinate.latitude
        self.longitude = cllocation.coordinate.longitude
        self.altitude = cllocation.altitude
        self.floor = cllocation.floor?.level
    }
    
    init(_ latitude: CLLocationDegrees, _ longitude: CLLocationDegrees, _ altitude: CLLocationDistance, _ floor: Int?) {
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.floor = floor
    }
    
    mutating func updateRoadAddress(to address: String) {
        self.roadAddressName = address
    }
}
