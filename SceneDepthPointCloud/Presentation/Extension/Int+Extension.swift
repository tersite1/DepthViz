//
//  Int+Extension.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/07/14.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

extension Int {
    var decimalString: String {
        let numberFormatter = NumberFormatter()
        numberFormatter.numberStyle = .decimal
        return numberFormatter.string(for: self)!
    }
}
