//
//  MarkerStorage.swift
//  DepthViz
//
//  사용자 정의 마커 저장/로드 관리
//

import Foundation

class MarkerStorage {
    static let shared = MarkerStorage()
    
    private let userDefaultsKey = "UserDefinedMarkers"
    
    private init() {}
    
    /// 모든 마커 가져오기 (기본 샘플 + 사용자 추가)
    func getAllMarkers() -> [LocationMarker] {
        var markers = LocationMarker.samples
        markers.append(contentsOf: getUserMarkers())
        return markers
    }
    
    /// 사용자가 추가한 마커만 가져오기
    func getUserMarkers() -> [LocationMarker] {
        guard let data = UserDefaults.standard.data(forKey: userDefaultsKey) else {
            return []
        }
        
        do {
            let markers = try JSONDecoder().decode([LocationMarker].self, from: data)
            return markers
        } catch {
            print("❌ 마커 로드 실패: \(error)")
            return []
        }
    }
    
    /// 새 마커 추가
    func addMarker(_ marker: LocationMarker) {
        var markers = getUserMarkers()
        markers.append(marker)
        saveUserMarkers(markers)
    }
    
    /// 마커 삭제
    func deleteMarker(_ marker: LocationMarker) {
        var markers = getUserMarkers()
        markers.removeAll { $0.id == marker.id }
        saveUserMarkers(markers)
    }
    
    /// 사용자 마커 저장
    private func saveUserMarkers(_ markers: [LocationMarker]) {
        do {
            let data = try JSONEncoder().encode(markers)
            UserDefaults.standard.set(data, forKey: userDefaultsKey)
            print("✅ 마커 저장 완료: \(markers.count)개")
        } catch {
            print("❌ 마커 저장 실패: \(error)")
        }
    }
}

