//
//  WebViewVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/10.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import WebKit

final class WebViewVC: UIViewController {
    static let identifier = "WebViewVC"
    private let webView = WKWebView()
    private var url: String!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureWebView()
        self.loadWeb()
    }
    
    private func configureWebView() {
        self.webView.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(self.webView)
        NSLayoutConstraint.activate([
            self.webView.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor),
            self.webView.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            self.webView.trailingAnchor.constraint(equalTo: self.view.trailingAnchor),
            self.webView.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor)
        ])
    }
    
    func setUrl(to url: String) {
        self.url = url
    }
    
    private func loadWeb() {
        print(self.url)
        if let url = URL(string: self.url) {
            let request = URLRequest(url: url)
            self.webView.load(request)
        }
    }
}
