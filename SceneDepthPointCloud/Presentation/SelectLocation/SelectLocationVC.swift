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
    /// 뒤로가기 버튼
    private let backButton = BackButton()
    /// 측정위치 선택 취소 및 창닫기 버튼
    private let cancelButton = CancelButton()
    /// 현재위치 기준 주소표시 텍스트
    private let currentLocationLabel = RoadAddressLabel()
    /// 2D 지도 view
    private let mapView = MKMapView()
    /// 주변 건물리스트표시 view
    private var buildingListView: UICollectionView = {
        let layout = UICollectionViewFlowLayout()
        layout.minimumLineSpacing = 10
        layout.scrollDirection = .vertical
        layout.sectionInset = UIEdgeInsets(top: 0, left: 16, bottom: 0, right: 16)
        return UICollectionView(frame: .zero, collectionViewLayout: layout)
    }()
    /// 선택 및 데이터 업로드 버튼
    private let bottomButton = SelectLocationLargeButton()
    /// 측정위치 선택화면 관련된 로직담당 객체
    private var viewModel: SelectLocationVM?
    private var cancellables: Set<AnyCancellable> = []
    private var buildingListDataSource: UICollectionViewDiffableDataSource<BuildingListCollectionViewCell.Section, BuildingInfo>!
    
    /// SelectLocationVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.configureMapView()
        self.configureBuildingListView()
        self.configureBuildingListDataSource()
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
        
        // backButton
        self.backButton.addAction(UIAction(handler: { [weak self] _ in
            self?.viewModel?.prevMode()
        }), for: .touchUpInside)
        self.view.addSubview(self.backButton)
        NSLayoutConstraint.activate([
            self.backButton.centerYAnchor.constraint(equalTo: self.titleLabel.centerYAnchor),
            self.backButton.leadingAnchor.constraint(equalTo: self.view.leadingAnchor, constant: 20)
        ])
        self.backButton.isHidden = true
        
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
        
        // buildingListView
        self.buildingListView.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(self.buildingListView)
        NSLayoutConstraint.activate([
            self.buildingListView.topAnchor.constraint(equalTo: self.currentLocationLabel.bottomAnchor, constant: 8),
            self.buildingListView.leadingAnchor.constraint(equalTo: self.view.leadingAnchor),
            self.buildingListView.trailingAnchor.constraint(equalTo: self.view.trailingAnchor),
            self.buildingListView.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -80)
        ])
    }
    
    /// mapView 화면을 표시할 초기화 함수
    private func configureMapView() {
        guard let locationData = self.viewModel?.locationData else { return }
        
        let latitude = locationData.latitude
        let longitude = locationData.longitude
        let region = MKCoordinateRegion(center: CLLocationCoordinate2D(latitude: latitude, longitude: longitude), span: .init(latitudeDelta: 0.002, longitudeDelta: 0.002))
        
        self.mapView.setRegion(region, animated: true)
        self.mapView.showsUserLocation = true
        self.mapView.isPitchEnabled = false
        self.mapView.showsCompass = true
        
        self.mapView.delegate = self
        self.mapView.isHidden = true
    }
    
    private func configureBuildingListView() {
        self.buildingListView.backgroundColor = .clear
        self.buildingListView.register(BuildingListCollectionViewCell.self, forCellWithReuseIdentifier: BuildingListCollectionViewCell.identifier)
        self.buildingListView.delegate = self
        self.buildingListView.isHidden = true
    }
    
    private func configureBuildingListDataSource() {
        let cellRegistration = UICollectionView.CellRegistration<BuildingListCollectionViewCell, BuildingInfo> { (cell, indexPath, info) in
            let isSelected = self.viewModel?.mode == .setIndoorInfo
            cell.updateCell(info: info, isSelected: isSelected)
        }
        
        self.buildingListDataSource = UICollectionViewDiffableDataSource<BuildingListCollectionViewCell.Section, BuildingInfo>(collectionView: self.buildingListView) { (collectionView: UICollectionView, indexPath: IndexPath, identifier: BuildingInfo) -> UICollectionViewCell? in
            return collectionView.dequeueConfiguredReusableCell(using: cellRegistration, for: indexPath, item: identifier)
        }
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
        self.bindMode()
        self.bindBuildingList()
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
    
    private func bindMode() {
        self.viewModel?.$mode
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] mode in
                switch mode {
                case .selectLocation:
                    self?.backButton.isHidden = true
                    self?.titleLabel.changeText(to: .selectLocation)
                    self?.mapView.isHidden = false
                    self?.mapView.isScrollEnabled = true
                    self?.buildingListView.isHidden = true
                    self?.bottomButton.changeStatus(to: .selectable)
                case .selectBuilding:
                    self?.backButton.isHidden = false
                    self?.titleLabel.changeText(to: .selectBuilding)
                    self?.mapView.isHidden = true
                    self?.mapView.isScrollEnabled = false
                    self?.buildingListView.isHidden = false
                    self?.bottomButton.changeStatus(to: .beforeSetting)
                default:
                    return
                }
            })
            .store(in: &self.cancellables)
    }
    
    private func bindBuildingList() {
        self.viewModel?.$buildingList
            .receive(on: DispatchQueue.main)
            .sink(receiveValue: { [weak self] buildingList in
                var snapshot = NSDiffableDataSourceSnapshot<BuildingListCollectionViewCell.Section, BuildingInfo>()
                snapshot.appendSections([.main])
                snapshot.appendItems(buildingList)
                self?.buildingListDataSource.apply(snapshot, animatingDifferences: true)
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

extension SelectLocationVC: UICollectionViewDelegateFlowLayout {
    func collectionView(_ collectionView: UICollectionView, layout collectionViewLayout: UICollectionViewLayout, sizeForItemAt indexPath: IndexPath) -> CGSize {
        return CGSize(width: collectionView.frame.width - (16*2), height: 60)
    }
}

extension SelectLocationVC: UICollectionViewDelegate {
    func collectionView(_ collectionView: UICollectionView, didSelectItemAt indexPath: IndexPath) {
        print("select")
    }
}
