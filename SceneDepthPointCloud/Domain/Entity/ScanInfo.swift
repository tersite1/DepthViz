//
//  ScanInfo.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/07/14.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct ScanInfo: Codable, Identifiable, Hashable {
    let id: String
    let date: Date
    let fileName: String
    let fileSize: String
    let points: Int
}
