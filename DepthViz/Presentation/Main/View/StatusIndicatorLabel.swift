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
        case readyForRecording = "녹화를 시작하려면 탭하세요\n↓"
        case recording = "녹화 중..."
        case loading = "처리 중..."
        case uploading = "업로드 중..."
        case removed = ""
        case needGPS = "위치 권한이 필요합니다\n설정에서 위치 권한을 허용해주세요"
        case cantRecord = "LiDAR를 지원하지 않는 기기입니다\niPhone 12 Pro 이상이 필요합니다"
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
        self.text = "업로드 중\n\(String(format: "%.1f", progress*100))%"
    }
}
