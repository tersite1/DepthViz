//
//  Collection+Extension.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/25.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

extension Collection {
    /// Returns the element at the specified index if it is within bounds, otherwise nil.
    subscript (safe index: Index) -> Element? {
        return indices.contains(index) ? self[index] : nil
    }
}
