//
//  LocationUsecase.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/20.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import CoreLocation

/// Location 관련 핵심비즈니스 로직 담당
final class LocationUsecase {
    /// 전달받은 gps 값들로 대표값(평균 등)의 location 객체를 생성 후 반환하는 함수
    func getSuitableLocation(from locations: [LocationData]) -> LocationData {
        var latitude: CLLocationDegrees = 0
        var longitude: CLLocationDegrees = 0
        var altitude: CLLocationDistance = 0
        var floor: Int = 0
        var floorCount = 0
        
        for location in locations {
            latitude += location.latitude
            longitude += location.longitude
            altitude += location.altitude
            if let f = location.floor {
                floor += f
                floorCount += 1
            }
        }
        
        let calculatedFloor = floorCount == 0 ? nil : Int(round(Double(floor)/Double(floorCount)))
        
        return LocationData(latitude, longitude, altitude, calculatedFloor)
    }
}
