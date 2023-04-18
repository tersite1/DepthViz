//
//  MainVM.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/18.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import ARKit
import MetalKit

final class MainVM {
    enum Mode {
        case cantRecord // LiDAR 스캔이 불가능한 경우
        case ready // LiDAR 측정 전 상태
        case recording // LiDAR 측정중 상태
        case loading // LiDAR 측정종료 및 데이터생성중 상태
        case uploading // 데이터 업로드중 상태
    }
    
    // 측정전, 측정중, 로딩중, 업로드중 상태값
    @Published private(set) var mode: Mode = .ready
    // 전송 후 네트워킹 결과값
    @Published private(set) var networkStatus: NetworkStatus?
    // 실시간 LiDAR 측정 및 Point Cloud 표시관련 핵심로직 담당 객체
    let renderer: Renderer
    // 메인화면 관련 네트워킹 로직 담당 객체
    let apiService: MainApiService
    
    init(session: ARSession, device: MTLDevice, view: MTKView) {
        self.mode = .ready
        self.networkStatus = nil
        self.renderer = Renderer(session: session, metalDevice: device, renderDestination: view)
        self.renderer.drawRectResized(size: view.bounds.size)
        self.apiService = MainApiService()
    }
}
