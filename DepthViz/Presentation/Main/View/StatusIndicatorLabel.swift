//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//



import UIKit

final class StatusIndicatorLabel: UILabel {
    enum TextType {
        case readyForRecording
        case recording
        case loading
        case optimizing
        case uploading
        case removed
        case needGPS
        case cantRecord

        var localizedText: String {
            switch self {
            case .readyForRecording:
                return NSLocalizedString("status_ready", comment: "")
            case .recording:
                return NSLocalizedString("status_recording", comment: "")
            case .loading:
                return NSLocalizedString("status_processing", comment: "")
            case .optimizing:
                return NSLocalizedString("status_optimizing", comment: "")
            case .uploading:
                return NSLocalizedString("status_uploading", comment: "")
            case .removed:
                return ""
            case .needGPS:
                return NSLocalizedString("status_need_gps", comment: "")
            case .cantRecord:
                return NSLocalizedString("status_cant_record", comment: "")
            }
        }
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
        self.text = type.localizedText
    }

    func uploadProgress(to progress: Double) {
        self.text = String(format: NSLocalizedString("status_upload_progress", comment: ""), progress * 100)
    }
}
