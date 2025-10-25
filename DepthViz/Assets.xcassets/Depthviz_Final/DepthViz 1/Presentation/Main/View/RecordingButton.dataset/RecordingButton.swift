//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import UIKit

final class RecordingButton: UIButton {
    private let width: CGFloat = 70
    private(set) var status: Status = .ready {
        didSet {
            switch status {
            case .ready:
                self.hideLoadingIndicator()
                self.setImage(UIImage(named: "startRecording"), for: .normal)
                self.isUserInteractionEnabled = true
            case .recording:
                self.hideLoadingIndicator()
                self.setImage(UIImage(named: "stopRecording"), for: .normal)
                self.isUserInteractionEnabled = true
            case .loading:
                self.setImage(nil, for: .normal)
                self.showLoadingIndicator()
                self.isUserInteractionEnabled = false
            case .cantRecording:
                self.hideLoadingIndicator()
                self.setImage(UIImage(named: "cantRecording"), for: .normal)
                self.isUserInteractionEnabled = false
            }
        }
    }
    enum Status {
        case ready
        case recording
        case loading
        case cantRecording
    }
    private var loadingIndicator: UIActivityIndicatorView = {
        let indicator = UIActivityIndicatorView()
        indicator.translatesAutoresizingMaskIntoConstraints = false
        indicator.stopAnimating()
        indicator.tintColor = .white
        indicator.style = .large
        indicator.hidesWhenStopped = true
        return indicator
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configureLayout()
        self.setImage(UIImage(named: "startRecording"), for: .normal)
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
        
        self.addSubview(self.loadingIndicator)
        NSLayoutConstraint.activate([
            self.loadingIndicator.centerXAnchor.constraint(equalTo: self.centerXAnchor),
            self.loadingIndicator.centerYAnchor.constraint(equalTo: self.centerYAnchor)
        ])
    }
    
    func changeStatus(to status: Status) {
        self.status = status
    }
    
    private func showLoadingIndicator() {
        self.loadingIndicator.startAnimating()
    }
    
    private func hideLoadingIndicator() {
        self.loadingIndicator.stopAnimating()
    }
}
