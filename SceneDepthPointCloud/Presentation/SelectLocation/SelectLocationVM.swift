//
//  SelectLocationVM.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/20.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import Combine
import CoreLocation

/// 측정위치 선택화면의 View 를 나타내기 위한 데이터 처리 담당
final class SelectLocationVM {
    enum Mode {
        case selectLocation
        case selectBuilding
        case setIndoorInfo
        case done
    }
    /// MainVM 에서 생성된 lidarData 값
    let lidarData: LiDARData
    /// MainVM 에서 생성된 locationData 값 및 사용자설정 위치값
    @Published private(set) var locationData: LocationData
    /// 주변 건물리스트
    @Published private(set) var buildingList: [BuildingOfMapInfo] = []
    /// 위치선택, 건물선택, 건물선택 완료 상태값
    @Published private(set) var mode: Mode = .selectLocation
    /// 네트워크 통신으로 인한 Error 발생값
    @Published private(set) var networkError: (title: String, text: String)?
    /// 건물리스트 api 사용시 pagenation 을 위한 현재 page 값
    @Published private(set) var indoorFloor: Int?
    private var page: Int = 1
    /// pagenation 불가능 여부값
    private(set) var isLastPage: Bool = false
    private(set) var fetching: Bool = false
    /// Address 데이터를 담당하는 객체
    private let addressRepository: AddressRepositoryInterface
    /// BuildingInfo 데이터를 담당하는 객체
    private let buildingRepository: BuildingInMapRepositoryInterface
    
    init(lidarData: LiDARData, locationData: LocationData, addressRepository: AddressRepositoryInterface, buildingRepository: BuildingInMapRepositoryInterface) {
        self.lidarData = lidarData
        self.locationData = locationData
        self.addressRepository = addressRepository
        self.buildingRepository = buildingRepository
        
        self.updateLocation()
    }
}

// MARK: INPUT
extension SelectLocationVM {
    /// locationData 값을 토대로 도로명주소를locationData 값으로 반영하는 함수
    func updateLocation(to location: CLLocationCoordinate2D? = nil) {
        if let location = location {
            // 전달받은 location 값으로 LocationData 생성
            self.locationData = LocationData(location.latitude, location.longitude, locationData.altitude, locationData.floor)
        }
        let locationData: LocationData = self.locationData
        
        DispatchQueue.global().async { [weak self] in
            self?.addressRepository.fetchAddress(from: locationData) { [weak self] result in
                switch result {
                case .success(let address):
                    self?.locationData.updateRoadAddress(to: address)
                case .failure(let fetchError):
                    if case.decode = fetchError {
                        print("Decode Error")
                    } else {
                        self?.networkError = (title: "Fetch Address Error", text: fetchError.message)
                    }
                }
            }
        }
    }
    
    /// page 값 증가 후 fetchBuildingList() 함수 호출하는 함수
    func nextPageBuildingListFetch() {
        guard self.isLastPage == false else { return }
        
        self.page += 1
        self.fetchBuildingList()
    }
    
    /// 현재 mode 값에 따라 다음 mode 값으로 변경하는 함수
    func changeMode() {
        switch self.mode {
        case .selectLocation:
            self.mode = .selectBuilding
            self.page = 1
            self.isLastPage = false
            self.fetchBuildingList()
        case .selectBuilding:
            self.mode = .setIndoorInfo
        case .setIndoorInfo:
            self.mode = .done
        case .done:
            print("upload")
        }
    }
    
    /// 뒤로가기 버튼으로 인한 mode값 변경
    func prevMode() {
        switch self.mode {
        case .selectLocation:
            return
        case .selectBuilding:
            self.buildingList = []
            self.mode = .selectLocation
        case .setIndoorInfo:
            self.page = 1
            self.isLastPage = false
            self.buildingList = []
            self.fetchBuildingList()
            self.mode = .selectBuilding
        case .done:
            self.mode = .setIndoorInfo
        }
    }
    
    func selectBuilding(to index: Int) {
        guard var selected = self.buildingList[safe: index] else {
            print("/// out of index: \(index)")
            return
        }
        
        selected.updateUUID()
        self.changeMode()
        self.buildingList = [selected]
    }
    
    func setIndoorValue(to floor: Int?) {
        self.indoorFloor = floor
    }
}

extension SelectLocationVM {
    /// locationData 값을 토대로 getBuildingList() 함수호출을 통해 buildingList 배열에 추가하는 함수
    private func fetchBuildingList() {
        guard self.fetching == false,
              self.isLastPage == false else { return }
        
        self.fetching = true
        let locationData = self.locationData
        let page = self.page
        
        DispatchQueue.global().async { [weak self] in
            self?.buildingRepository.fetchBuildingInfo(from: locationData, page: page, completion: { [weak self] result in
                switch result {
                case .success((let infos, let isLastPage)):
                    self?.buildingList += infos
                    self?.isLastPage = isLastPage
                    
                case .failure(let fetchError):
                    self?.networkError = (title: "Fetch BuildingList Error", text: fetchError.message)
                }
                self?.fetching = false
            })
        }
    }
}
