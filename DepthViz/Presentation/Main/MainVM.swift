//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import Foundation
import ARKit
import MetalKit
import Combine

/// 메인화면의 View 를 나타내기 위한 데이터 처리 담당
final class MainVM {
    enum Mode {
        case cantRecord // LiDAR 스캔이 불가능한 경우
        case cantGetGPS // 위치정보를 받을 수 없는 경우
        case ready // LiDAR 측정 전 상태
        case recording // LiDAR 측정중 상태
        case loading // LiDAR 측정종료 및 데이터생성중 상태
        case uploading // 데이터 업로드중 상태
        case recordingTerminate
        case uploadingTerminate
    }
    // MARK: OUTPUT
    /// 측정전, 측정중, 로딩중, 업로드중 상태값
    @Published private(set) var mode: Mode = .ready
    /// 측정중인 Point Cloud Data 개수
    @Published private(set) var pointCount: Int = 0
    /// 측정종료 후 반환된 LiDAR data 값
    @Published private(set) var lidarData: LiDARData?
    /// 전송 후 네트워킹 성공
    @Published private(set) var uploadSuccess: Bool = false {
        didSet {
            if uploadSuccess {
                UserDefaults.standard.setValue(false, forKey: "lidarData")
            }
        }
    }
    /// 업로드 진행률
    @Published private(set) var uploadProgress: Double = 0
    /// 앱 내부 데이터 로딩여부
    @Published private(set) var processWithStorageData: Bool = false
    /// 앱 내부 저장 성공여부
    @Published private(set) var saveToStorageSuccess: Bool?
    private(set) var networkErrorMessage: String = ""
    /// ScanData 임시저장
    @Published private(set) var scanData: ScanData?
    /// ScanData 저장 성공여부
    @Published private(set) var saveScanDataSuccess: Bool?
    
    /// 실시간 LiDAR 측정 및 Point Cloud 표시관련 핵심로직 담당 객체
    private let renderer: Renderer
    /// Renderer 로부터 수신받기 위한 property
    private var cancellables: Set<AnyCancellable> = []
    
    init(session: ARSession, device: MTLDevice, view: MTKView) {
        self.renderer = Renderer(session: session, metalDevice: device, renderDestination: view)
        self.renderer.drawRectResized(size: view.bounds.size)
        self.bindRenderer()
        
        // storage에 저장된 lidar가 있는지 확인
        if UserDefaults.standard.bool(forKey: "lidarData") == true {
            self.mode = .loading
            self.processWithStorageData = true
        } else {
            self.mode = .ready
        }
    }
}

// MARK: INPUT
extension MainVM {
    /// 현재 mode 값에 따라 다음 mode 값으로 변경하는 함수
    func changeMode() {
        switch self.mode {
        // 측정상태로 변경
        case .ready:
            self.startRecording()
            self.mode = .recording
        // 측정종료 및 데이터 생성상태로 변경
        case .recording:
            self.stopRecording()
            self.mode = .loading
        // 그 외의 경우는 무시된다
        case.loading:
            self.mode = .uploading
        case.uploading:
            self.resetRenderer()
            self.mode = .ready
        default:
            return
        }
    }
    
    /// MTKViewDelegate 관련 함수로 resize 반영해주는 함수
    func rendererResize(to size: CGSize) {
        self.renderer.drawRectResized(size: size)
    }

    /// Renderer를 draw 시키기 위한 함수
    func rendererDraw() {
        self.renderer.draw()
    }
    
    /// 메모리 부족으로 인한 LiDAR 측정 종료 함수
    func terminateRecording() {
        self.mode = .recordingTerminate
        self.stopRecording()
    }
    
    func readyForRecording() {
        self.mode = .ready
    }
    
    /// LiDAR 측정불가 상태로 인한 측정불가한 경우 동작 비활성화를 위한 함수
    func cantRecording() {
        self.mode = .cantRecord
    }
    
    /// GPS 측정불가 상태로 인한 측정불가한 경우 동작 비활성화를 위한 함수
    func cantGetGPS() {
        self.mode = .cantGetGPS
    }
    
    func appendLocation(_ location: CLLocation) {
    }
    
    func uploadCancel() {
        self.resetRenderer()
        self.mode = .uploading
    }
    
    func saveScanData(fileName: String? = nil) {
        if var scanData = self.scanData {
            if let fileName = fileName {
                scanData.rename(to: fileName)
            }
            
            // ScanStorage 저장
            let success = ScanStorage.shared.save(scanData)
            self.saveScanDataSuccess = success
            // mode 변경
            self.resetRenderer()
            self.mode = .ready
            self.scanData = nil
        }
    }
}

extension MainVM {
    /// Renderer로부터 값 변화를 수신하여 View로 표시하기 위한 함수
    private func bindRenderer() {
        self.renderer.$currentPointCount
            .receive(on: DispatchQueue.global())
            .sink { [weak self] count in
                self?.pointCount = count
            }
            .store(in: &self.cancellables)
        
//        self.renderer.$lidarRawStringData
//            .receive(on: DispatchQueue.global())
//            .sink { [weak self] rawStringData in
//                guard let rawStringData = rawStringData,
//                      let pointCount = self?.renderer.currentPointCount else { return }
//
//                self?.lidarData = LiDARData(rawStringData: rawStringData, pointCount: pointCount)
//
//                // MARK: location, lidar 파일 저장로직
//                if self?.mode == .recordingTerminate {
//                    self?.saveCurrentFilesToStorage()
//                }
//            }
//            .store(in: &self.cancellables)
        
        self.renderer.$lidarRawStringData
            .receive(on: DispatchQueue.global())
            .sink { [weak self] rawStringData in
                guard let rawStringData = rawStringData,
                      let pointCount = self?.renderer.currentPointCount else { return }
                
                // ScanData 생성
                self?.scanData = ScanData(rawStringData: rawStringData, pointCount: pointCount)
            }
            .store(in: &self.cancellables)
    }
    
    /// LiDAR 측정 활성화 함수 (renderer 활성화)
    private func startRecording() {
        self.renderer.isRecording = true
    }
    
    /// LiDAR 측정 종료 함수
    private func stopRecording() {
        self.renderer.isRecording = false
        self.renderer.savePointCloud()
    }
    
    /// Renderer 초기화 및 재측정 설정 함수
    private func resetRenderer() {
        self.renderer.clearParticles()
    }
}
