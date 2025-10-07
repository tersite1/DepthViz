//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//



import UIKit

final class StatusIndicatorLabel: UILabel {
    enum TextType: String {
        case readyForRecording = "Tap to start recording\n↓"
        case recording = "Recording..."
        case loading = "Loading..."
        case uploading = "Uploading..."
        case removed = ""
        case needGPS = "Inactivate GPS"
        case cantRecord = "Unsupported Device"
    }
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.textColor = .white
        self.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        self.textAlignment = .center
        self.numberOfLines = 0
        self.changeText(to: .readyForRecording)
    }
    
    func changeText(to type: TextType) {
        self.text = type.rawValue
    }
    
    func uploadProgress(to progress: Double) {
        self.text = "Uploading\n\(String(format: "%.1f", progress*100))%"
    }
}
