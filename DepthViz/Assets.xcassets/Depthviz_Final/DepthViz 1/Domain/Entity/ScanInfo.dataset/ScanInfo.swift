//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import Foundation

struct ScanInfo: Codable, Identifiable, Hashable {
    let id: String
    let date: Date
    let fileName: String
    let fileSize: String
    let points: Int
}
