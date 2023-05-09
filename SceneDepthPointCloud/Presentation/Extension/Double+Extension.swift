//
//  Double+Extension.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/10.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

// https://stackoverflow.com/questions/27996351/swift-convert-decimal-coordinate-into-degrees-minutes-seconds-direction/50765689#50765689
extension Double {
    var latitudeToDMS: String {
        var latitudeSeconds = self * 3600
        let latitudeDegrees = latitudeSeconds / 3600
        latitudeSeconds = latitudeSeconds.truncatingRemainder(dividingBy: 3600)
        let latitudeMinutes = latitudeSeconds / 60
        latitudeSeconds = latitudeSeconds.truncatingRemainder(dividingBy: 60)
        let latitudeCardinalDirection = latitudeDegrees >= 0 ? "N" : "S"
        let latitudeDescription = String(format: "%02.0f°%02.0f'%02.0f\" %@",
                                         abs(latitudeDegrees), abs(latitudeMinutes),
                                         abs(latitudeSeconds), latitudeCardinalDirection)
        return latitudeDescription
    }
    
    var longitudeToDMS: String {
        var longitudeSeconds = self * 3600
        let longitudeDegrees = longitudeSeconds / 3600
        longitudeSeconds = longitudeSeconds.truncatingRemainder(dividingBy: 3600)
        let longitudeMinutes = longitudeSeconds / 60
        longitudeSeconds = longitudeSeconds.truncatingRemainder(dividingBy: 60)
        let longitudeCardinalDirection = longitudeDegrees >= 0 ? "E" : "W"
        let longitudeDescription = String(format: "%02.0f°%02.0f'%02.0f\" %@",
                                          abs(longitudeDegrees), abs(longitudeMinutes),
                                          abs(longitudeSeconds), longitudeCardinalDirection)
        return longitudeDescription
    }
}
