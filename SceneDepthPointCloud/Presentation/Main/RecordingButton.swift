//
//  RecordingButton.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/28.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit

final class RecordingButton: UIButton {
    let width: CGFloat = 70
    
    convenience init() {
        self.init(frame: CGRect())
        self.configureLayout()
        self.isSelected = false
    }
    
    private func configureLayout() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.clipsToBounds = true
        self.layer.borderWidth = 4
        self.layer.borderColor = UIColor.white.cgColor
        self.layer.cornerRadius = width/2
        
        NSLayoutConstraint.activate([
            self.widthAnchor.constraint(equalToConstant: width),
            self.heightAnchor.constraint(equalToConstant: width)
        ])
    }
    
    override var isSelected: Bool {
        didSet {
            if isSelected {
                self.stopRecording()
            } else {
                self.startRecording()
            }
        }
    }
    
    private func stopRecording() {
        self.setImage(UIImage(named: "stopRecording"), for: .normal)
    }
    
    private func startRecording() {
        self.setImage(UIImage(named: "startRecording"), for: .normal)
    }
}
