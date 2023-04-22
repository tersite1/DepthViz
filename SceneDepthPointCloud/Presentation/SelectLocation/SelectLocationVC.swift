//
//  SelectLocationVC.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/20.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import Combine
import MapKit

/// 측정위치 선택화면의 UI 및 UX 담당
final class SelectLocationVC: UIViewController {
    static let identifier = "SelectLocationVC"
    /// MainVC 의 함수를 사용하기 위한 delegate
    private weak var delegate: SelectLocationDelegate?
    /// 화면 상단 타이틀 텍스트
    private let titleLabel = SelectLocationTitleLabel()
    /// 측정위치 선택 취소 및 창닫기 버튼
    private let cancelButton = CancelButton()
    /// 현재위치 기준 주소표시 텍스트
    private let currentLocationLabel = RoadAddressLabel()
    /// 2D 지도 view
    private let mapView = MKMapView()
    /// 선택 및 데이터 업로드 버튼
    private let bottomButton = SelectLocationLargeButton()
    /// 측정위치 선택화면 관련된 로직담당 객체
    private var viewModel: SelectLocationVM?
    private var cancellables: Set<AnyCancellable> = []
    
    /// SelectLocationVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.configureMapView()
        self.bindViewModel()
    }
}

// MARK: Configure
extension SelectLocationVC {
    /// SelectLocationVC 표시할 UI 설정
    private func configureUI() {
        // titleLabel
        self.view.addSubview(self.titleLabel)
        NSLayoutConstraint.activate([
            self.titleLabel.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor, constant: 20),
            self.titleLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor)
        ])
        
        // cancelButton
        self.cancelButton.addAction(UIAction(handler: { [weak self] _ in
            self?.dismiss(animated: true)
            self?.delegate?.uploadCancel()
        }), for: .touchUpInside)
        self.view.addSubview(self.cancelButton)
        NSLayoutConstraint.activate([
            self.cancelButton.centerYAnchor.constraint(equalTo: self.titleLabel.centerYAnchor),
            self.cancelButton.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -20)
        ])
        
        // currentLocationLabel
        self.view.addSubview(self.currentLocationLabel)
        NSLayoutConstraint.activate([
            self.currentLocationLabel.topAnchor.constraint(equalTo: self.titleLabel.bottomAnchor, constant: 14),
            self.currentLocationLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor)
        ])
        
        // mapView
        self.mapView.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(self.mapView)
        NSLayoutConstraint.activate([
            self.mapView.topAnchor.constraint(equalTo: self.currentLocationLabel.bottomAnchor, constant: 8),
            self.mapView.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            self.mapView.trailingAnchor.constraint(equalTo: self.view.trailingAnchor),
            self.mapView.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -80)
        ])
        
        // mapView center pin
        let centerPin = UIImageView(image: UIImage(named: "largePin"))
        let pinShadow = UIImageView(image: UIImage(named: "pihShadow"))
        centerPin.translatesAutoresizingMaskIntoConstraints = false
        pinShadow.translatesAutoresizingMaskIntoConstraints = false
        
        self.mapView.addSubview(pinShadow)
        self.mapView.addSubview(centerPin)
        
        NSLayoutConstraint.activate([
            centerPin.widthAnchor.constraint(equalToConstant: 42),
            centerPin.heightAnchor.constraint(equalToConstant: 42),
            pinShadow.widthAnchor.constraint(equalToConstant: 12),
            pinShadow.heightAnchor.constraint(equalToConstant: 7)
        ])
        
        NSLayoutConstraint.activate([
            centerPin.centerXAnchor.constraint(equalTo: self.mapView.centerXAnchor),
            centerPin.centerYAnchor.constraint(equalTo: self.mapView.centerYAnchor, constant: -21),
            pinShadow.centerXAnchor.constraint(equalTo: centerPin.centerXAnchor),
            pinShadow.centerYAnchor.constraint(equalTo: centerPin.bottomAnchor)
        ])
        
        // bottomButton
        self.bottomButton.addAction(UIAction(handler: { [weak self] _ in
            self?.viewModel?.changeMode()
        }), for: .touchUpInside)
        self.view.addSubview(self.bottomButton)
        NSLayoutConstraint.activate([
            self.bottomButton.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -8),
            self.bottomButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 16),
            self.bottomButton.trailingAnchor.constraint(equalTo: self.view.trailingAnchor, constant: -16)
        ])
    }
    
    /// mapView 화면을 표시할 초기화 함수
    private func configureMapView() {
        guard let locationData = self.viewModel?.locationData else { return }
        print("측정 위치: \(locationData.latitude), \(locationData.longitude)")
        let latitude = locationData.latitude
        let longitude = locationData.longitude
        let region = MKCoordinateRegion(center: CLLocationCoordinate2D(latitude: latitude, longitude: longitude), span: .init(latitudeDelta: 0.002, longitudeDelta: 0.002))
        
        self.mapView.setRegion(region, animated: true)
        self.mapView.showsUserLocation = true
        self.mapView.isPitchEnabled = false
        self.mapView.showsCompass = true
        
        self.mapView.delegate = self
    }
}

// MARK: INPUT from MainVC
extension SelectLocationVC {
    func configureDelegate(_ delegate: SelectLocationDelegate) {
        self.delegate = delegate
    }
    
    func configureViewModel(_ viewModel: SelectLocationVM) {
        self.viewModel = viewModel
    }
}

// MARK: INPUT from ViewModel
extension SelectLocationVC {
    private func bindViewModel() {
        self.bindLocationData()
        self.bindNetworkError()
    }
    
    private func bindLocationData() {
        self.viewModel?.$locationData
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] locationData in
                print("새 위치 업데이트: \(locationData.latitude), \(locationData.longitude)")
                self?.currentLocationLabel.updateAddress(to: locationData.roadAddressName)
            })
            .store(in: &self.cancellables)
    }
    
    private func bindNetworkError() {
        self.viewModel?.$networkError
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] error in
                guard let error = error else { return }
                self?.showAlert(title: error.title, text: error.text)
            })
            .store(in: &self.cancellables)
    }
}

extension SelectLocationVC: MKMapViewDelegate {
    /// mapView 위치 이동으로 인해 지역이 변경됨을 수신하는 함수
    func mapView(_ mapView: MKMapView, regionDidChangeAnimated animated: Bool) {
        // mapView 의 중심좌표로 locationData 를 업데이트 한다
        let center = mapView.centerCoordinate
        self.viewModel?.updateLocation(to: center)
    }
}
