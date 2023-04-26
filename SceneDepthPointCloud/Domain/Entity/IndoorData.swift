//
//  IndoorData.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/26.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct IndoorData: Encodable {
    let latitude: Double
    let longitude: Double
    let altitude: Double?
    let floor: String
}
