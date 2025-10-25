//
//  ActivityViewController.swift
//  DepthViz
//
//  Created by 장민석 on 8/27/24.
//  Copyright © 2024 Apple. All rights reserved.
//


import SwiftUI
import UIKit

struct ActivityViewController: UIViewControllerRepresentable {
    let activityItems: [Any]
    let applicationActivities: [UIActivity]? = nil

    func makeUIViewController(context: Context) -> UIActivityViewController {
        let controller = UIActivityViewController(activityItems: activityItems, applicationActivities: applicationActivities)

        // 화면의 반만 점유하도록 설정
        if let presentationController = controller.sheetPresentationController {
            presentationController.detents = [.medium()] // medium 크기는 화면의 반을 차지합니다.
            presentationController.prefersGrabberVisible = true // 위쪽에 손잡이 표시
        }

        return controller
    }

    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
