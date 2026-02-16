//
//  MapViewController.swift
//  DepthViz
//
//  Google Maps를 표시하는 UIViewController
//

import UIKit
import MapKit
import SwiftUI
import CoreLocation

/// Google Maps 대신 Apple Maps를 사용한 지도 화면
class MapViewController: UIViewController, UIGestureRecognizerDelegate {
    
    private var mapView: MKMapView!
    private var markers: [LocationMarker] = []
    private var projects: [Project] = []
    private var selectedMarker: LocationMarker?
    private var temporaryAnnotation: MKPointAnnotation?
    private var poiAnnotations: [POIAnnotation] = []
    private var poiFetchDebounceWorkItem: DispatchWorkItem?
    private var lastPOIFetchRegion: MKCoordinateRegion?
    private var isFetchingPOIList = false
    private var poiSearch: MKLocalSearch?
    private let geocoder = CLGeocoder()
    private var lastSuggestedName: String?
    private var lastSuggestedDescription: String?
    private var lastSuggestedAddress: String?

    // 검색 UI
    private var searchBar: UISearchBar!
    private var searchResultsTable: UITableView!
    private var searchCompleter = MKLocalSearchCompleter()
    private var searchResults: [MKLocalSearchCompletion] = []
    private var searchTableHeightConstraint: NSLayoutConstraint!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        navigationController?.setNavigationBarHidden(true, animated: false)
        
        setupMapView()
        setupSearchBar()
        loadMarkers()
        addMarkers()
        setupGestures()
        schedulePOIFetch()
    }
    
    private func schedulePOIFetch() {
        poiFetchDebounceWorkItem?.cancel()
        let workItem = DispatchWorkItem { [weak self] in
            self?.refreshPOIs()
        }
        poiFetchDebounceWorkItem = workItem
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35, execute: workItem)
    }
    
    private func refreshPOIs() {
        let region = mapView.region
        let maxDelta = max(region.span.latitudeDelta, region.span.longitudeDelta)
        if maxDelta > 0.2 {
            clearPOIAnnotations()
            lastPOIFetchRegion = nil
            return
        }
        if let lastRegion = lastPOIFetchRegion,
           abs(lastRegion.center.latitude - region.center.latitude) < lastRegion.span.latitudeDelta * 0.25,
           abs(lastRegion.center.longitude - region.center.longitude) < lastRegion.span.longitudeDelta * 0.25,
           abs(lastRegion.span.latitudeDelta - region.span.latitudeDelta) / max(lastRegion.span.latitudeDelta, 0.0001) < 0.25,
           abs(lastRegion.span.longitudeDelta - region.span.longitudeDelta) / max(lastRegion.span.longitudeDelta, 0.0001) < 0.25 {
            return
        }
        lastPOIFetchRegion = region
        fetchPOIs(in: region)
    }
    
    private func fetchPOIs(in region: MKCoordinateRegion) {
        guard #available(iOS 13.0, *), isFetchingPOIList == false else { return }
        let radius = min(max(approximateRadius(for: region), 150), 1000)
        guard radius >= 150 else { return }
        isFetchingPOIList = true
        let request = MKLocalPointsOfInterestRequest(center: region.center, radius: radius)
        let search = MKLocalSearch(request: request)
        poiSearch?.cancel()
        poiSearch = search
        search.start { [weak self] response, _ in
            guard let self = self else { return }
            self.poiSearch = nil
            self.isFetchingPOIList = false
            guard let response = response else {
                self.clearPOIAnnotations()
                return
            }
            self.updatePOIAnnotations(with: response.mapItems)
        }
    }
    
    private func approximateRadius(for region: MKCoordinateRegion) -> CLLocationDistance {
        let center = CLLocation(latitude: region.center.latitude, longitude: region.center.longitude)
        let edgeLatitude = CLLocation(latitude: region.center.latitude + region.span.latitudeDelta / 2, longitude: region.center.longitude)
        let edgeLongitude = CLLocation(latitude: region.center.latitude, longitude: region.center.longitude + region.span.longitudeDelta / 2)
        return max(center.distance(from: edgeLatitude), center.distance(from: edgeLongitude))
    }
    
    private func updatePOIAnnotations(with items: [MKMapItem]) {
        mapView.removeAnnotations(poiAnnotations)
        poiAnnotations.removeAll()
        guard !items.isEmpty else { return }
        let threshold: CLLocationDistance = 15
        let filteredItems = items.filter { item in
            let coordinate = item.placemark.coordinate
            return !markerExists(near: coordinate, threshold: threshold)
        }
        let annotations = filteredItems.map { POIAnnotation(mapItem: $0) }
        poiAnnotations = annotations
        mapView.addAnnotations(annotations)
    }
    
    private func clearPOIAnnotations() {
        if !poiAnnotations.isEmpty {
            mapView.removeAnnotations(poiAnnotations)
            poiAnnotations.removeAll()
        }
    }
    
    private func loadMarkers() {
        markers = MarkerStorage.shared.getAllMarkers()
        projects = ProjectManager.shared.projects
        print("✅ 마커 로드 완료: \(markers.count)개, 프로젝트: \(projects.count)개")
    }
    
    @objc private func addMarkerButtonTapped() {
        let addMarkerView = AddMarkerView { [weak self] marker in
            self?.addNewMarker(marker)
        }
        let hostingController = UIHostingController(rootView: addMarkerView)
        present(hostingController, animated: true)
    }
    
    private func addNewMarker(_ marker: LocationMarker) {
        // 저장
        MarkerStorage.shared.addMarker(marker)
        
        // 배열에 추가
        markers.append(marker)
        
        // 지도에 추가
        let annotation = MarkerAnnotation(marker: marker)
        mapView.addAnnotation(annotation)
        
        // 추가된 위치로 이동
        let region = MKCoordinateRegion(
            center: marker.coordinate,
            span: MKCoordinateSpan(latitudeDelta: 0.05, longitudeDelta: 0.05)
        )
        mapView.setRegion(region, animated: true)
        
        print("✅ 새 마커 추가: \(marker.name)")
        clearPOIAnnotations()
        schedulePOIFetch()
    }
    
    private func markerExists(near coordinate: CLLocationCoordinate2D, threshold: CLLocationDistance) -> Bool {
        let target = CLLocation(latitude: coordinate.latitude, longitude: coordinate.longitude)
        if markers.contains(where: { target.distance(from: CLLocation(latitude: $0.latitude, longitude: $0.longitude)) < threshold }) {
            return true
        }
        if poiAnnotations.contains(where: { target.distance(from: CLLocation(latitude: $0.coordinate.latitude, longitude: $0.coordinate.longitude)) < threshold }) {
            return true
        }
        if let temp = temporaryAnnotation {
            let tempLocation = CLLocation(latitude: temp.coordinate.latitude, longitude: temp.coordinate.longitude)
            if target.distance(from: tempLocation) < threshold {
                return true
            }
        }
        return false
    }
    
    private func setupMapView() {
        mapView = MKMapView()
        mapView.delegate = self
        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.showsUserLocation = true
        mapView.showsCompass = true
        if #available(iOS 13.0, *) {
            mapView.pointOfInterestFilter = .includingAll
        }
        
        // Clustering Registration
        mapView.register(MKMarkerAnnotationView.self, forAnnotationViewWithReuseIdentifier: "MarkerAnnotation")
        mapView.register(MKMarkerAnnotationView.self, forAnnotationViewWithReuseIdentifier: "POIAnnotation")
        
        view.addSubview(mapView)
        
        NSLayoutConstraint.activate([
            mapView.topAnchor.constraint(equalTo: view.topAnchor), // Full screen
            mapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.bottomAnchor.constraint(equalTo: view.bottomAnchor)
        ])
        
        // 서울 중심으로 초기 위치 설정
        let seoulCenter = CLLocationCoordinate2D(latitude: 37.5665, longitude: 126.9780)
        let region = MKCoordinateRegion(
            center: seoulCenter,
            span: MKCoordinateSpan(latitudeDelta: 0.1, longitudeDelta: 0.1)
        )
        mapView.setRegion(region, animated: false)
        
        setupFloatingControls()
    }
    
    private func setupSearchBar() {
        // 검색바
        searchBar = UISearchBar()
        searchBar.placeholder = "장소 검색..."
        searchBar.delegate = self
        searchBar.searchBarStyle = .minimal
        searchBar.backgroundImage = UIImage()
        searchBar.translatesAutoresizingMaskIntoConstraints = false

        let searchContainer = UIView()
        searchContainer.backgroundColor = .systemBackground.withAlphaComponent(0.9)
        searchContainer.layer.cornerRadius = 12
        searchContainer.layer.shadowColor = UIColor.black.cgColor
        searchContainer.layer.shadowOpacity = 0.15
        searchContainer.layer.shadowOffset = CGSize(width: 0, height: 2)
        searchContainer.layer.shadowRadius = 6
        searchContainer.clipsToBounds = false
        searchContainer.translatesAutoresizingMaskIntoConstraints = false
        searchContainer.addSubview(searchBar)
        view.addSubview(searchContainer)

        // 검색 결과 테이블
        searchResultsTable = UITableView()
        searchResultsTable.delegate = self
        searchResultsTable.dataSource = self
        searchResultsTable.register(UITableViewCell.self, forCellReuseIdentifier: "SearchCell")
        searchResultsTable.backgroundColor = .systemBackground.withAlphaComponent(0.95)
        searchResultsTable.layer.cornerRadius = 12
        searchResultsTable.layer.shadowColor = UIColor.black.cgColor
        searchResultsTable.layer.shadowOpacity = 0.1
        searchResultsTable.layer.shadowOffset = CGSize(width: 0, height: 2)
        searchResultsTable.layer.shadowRadius = 4
        searchResultsTable.clipsToBounds = true
        searchResultsTable.isHidden = true
        searchResultsTable.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(searchResultsTable)

        searchTableHeightConstraint = searchResultsTable.heightAnchor.constraint(equalToConstant: 0)

        NSLayoutConstraint.activate([
            searchContainer.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            searchContainer.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 60),
            searchContainer.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            searchContainer.heightAnchor.constraint(equalToConstant: 44),

            searchBar.topAnchor.constraint(equalTo: searchContainer.topAnchor),
            searchBar.leadingAnchor.constraint(equalTo: searchContainer.leadingAnchor, constant: 4),
            searchBar.trailingAnchor.constraint(equalTo: searchContainer.trailingAnchor, constant: -4),
            searchBar.bottomAnchor.constraint(equalTo: searchContainer.bottomAnchor),

            searchResultsTable.topAnchor.constraint(equalTo: searchContainer.bottomAnchor, constant: 4),
            searchResultsTable.leadingAnchor.constraint(equalTo: searchContainer.leadingAnchor),
            searchResultsTable.trailingAnchor.constraint(equalTo: searchContainer.trailingAnchor),
            searchTableHeightConstraint
        ])

        // 자동완성
        searchCompleter.delegate = self
        searchCompleter.resultTypes = [.address, .pointOfInterest]
    }

    private func setupFloatingControls() {
        // Close Button
        let closeButton = UIButton(type: .custom)
        let closeConfig = UIImage.SymbolConfiguration(pointSize: 14, weight: .semibold)
        closeButton.setImage(UIImage(systemName: "xmark", withConfiguration: closeConfig), for: .normal)
        closeButton.tintColor = .label
        closeButton.backgroundColor = .systemBackground.withAlphaComponent(0.85)
        closeButton.layer.cornerRadius = 20
        closeButton.layer.shadowColor = UIColor.black.cgColor
        closeButton.layer.shadowOpacity = 0.15
        closeButton.layer.shadowOffset = CGSize(width: 0, height: 2)
        closeButton.layer.shadowRadius = 4
        closeButton.clipsToBounds = false
        closeButton.translatesAutoresizingMaskIntoConstraints = false
        closeButton.addTarget(self, action: #selector(closeButtonTapped), for: .touchUpInside)
        view.addSubview(closeButton)

        // Add Button
        let addButton = UIButton(type: .custom)
        let addConfig = UIImage.SymbolConfiguration(pointSize: 20, weight: .semibold)
        addButton.setImage(UIImage(systemName: "plus", withConfiguration: addConfig), for: .normal)
        addButton.tintColor = .white
        addButton.backgroundColor = .systemBlue
        addButton.layer.cornerRadius = 25
        addButton.layer.shadowColor = UIColor.black.cgColor
        addButton.layer.shadowOpacity = 0.2
        addButton.layer.shadowOffset = CGSize(width: 0, height: 2)
        addButton.layer.shadowRadius = 6
        addButton.clipsToBounds = false
        addButton.translatesAutoresizingMaskIntoConstraints = false
        addButton.addTarget(self, action: #selector(addMarkerButtonTapped), for: .touchUpInside)
        view.addSubview(addButton)
        
        NSLayoutConstraint.activate([
            closeButton.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 16),
            closeButton.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            closeButton.widthAnchor.constraint(equalToConstant: 40),
            closeButton.heightAnchor.constraint(equalToConstant: 40),
            
            addButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -20),
            addButton.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -20),
            addButton.widthAnchor.constraint(equalToConstant: 50),
            addButton.heightAnchor.constraint(equalToConstant: 50)
        ])
    }

    
    private func setupGestures() {
        // Long Press Gesture로 마커 추가
        let longPress = UILongPressGestureRecognizer(target: self, action: #selector(handleLongPress(_:)))
        longPress.minimumPressDuration = 0.5
        mapView.addGestureRecognizer(longPress)
        
        // Tap Gesture로도 마커 추가
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(handleMapTap(_:)))
        tapGesture.numberOfTouchesRequired = 1
        tapGesture.numberOfTapsRequired = 1
        tapGesture.cancelsTouchesInView = false
        tapGesture.require(toFail: longPress)
        tapGesture.delegate = self
        mapView.addGestureRecognizer(tapGesture)
    }
    
    @objc private func handleLongPress(_ gesture: UILongPressGestureRecognizer) {
        guard gesture.state == .began else { return }
        
        // 터치 위치를 좌표로 변환
        let touchPoint = gesture.location(in: mapView)
        let coordinate = mapView.convert(touchPoint, toCoordinateFrom: mapView)
        placeTemporaryAnnotation(at: coordinate, feedbackStyle: .medium)
    }
    
    @objc private func handleMapTap(_ gesture: UITapGestureRecognizer) {
        guard gesture.state == .ended else { return }
        let touchPoint = gesture.location(in: mapView)
        
        // 기존 annotation을 터치한 경우에는 무시 (선택만 하도록)
        if let hitView = mapView.hitTest(touchPoint, with: nil) {
            if hitView is UIControl { return }
            if let annotationView = (hitView as? MKAnnotationView) ?? hitView.findSuperview(of: MKAnnotationView.self) {
                if let markerAnnotation = annotationView.annotation as? MarkerAnnotation {
                    // 마커 탭 시 바로 상세 화면 표시 (Scan Here / View Data)
                    selectedMarker = markerAnnotation.marker
                    showMarkerDetail(marker: markerAnnotation.marker)
                    return
                }
                if let poiAnnotation = annotationView.annotation as? POIAnnotation {
                    mapView.selectAnnotation(poiAnnotation, animated: true)
                    prepareSuggestion(from: poiAnnotation.mapItem)
                    showAddMarkerDialog(at: poiAnnotation.coordinate)
                    return
                }
                if annotationView.annotation === temporaryAnnotation {
                    mapView.selectAnnotation(annotationView.annotation!, animated: true)
                    return
                }
                if let annotation = annotationView.annotation {
                    placeTemporaryAnnotation(at: annotation.coordinate, feedbackStyle: .light)
                    return
                }
            }
        }
        
        let coordinate = mapView.convert(touchPoint, toCoordinateFrom: mapView)
        placeTemporaryAnnotation(at: coordinate, feedbackStyle: .light)
    }
    
    private func addMarkers() {
        for marker in markers {
            let isProjectLocation = projects.contains { project in
                guard let projLoc = project.location else { return false }
                return distance(projLoc.coordinate, marker.coordinate) < 5.0
            }
            
            if !isProjectLocation {
                let annotation = MarkerAnnotation(marker: marker)
                mapView.addAnnotation(annotation)
            }
        }
        
        for project in projects {
            if let location = project.location {
                let annotation = ProjectAnnotation(project: project)
                mapView.addAnnotation(annotation)
            }
        }
        
        schedulePOIFetch()
    }
    
    @objc private func closeButtonTapped() {
        clearTemporarySelection()
        dismiss(animated: true)
    }
}

// MARK: - MKMapViewDelegate
extension MapViewController: MKMapViewDelegate {
    
    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        if annotation is MKUserLocation {
            return nil
        }
        
        if let tempAnnotation = annotation as? MKPointAnnotation, tempAnnotation === temporaryAnnotation {
            let identifier = "TempAnnotation"
            var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            
            if annotationView == nil {
                annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                annotationView?.canShowCallout = true
                annotationView?.markerTintColor = .systemGreen

                let addButton = UIButton(type: .system)
                addButton.setTitle("Add", for: .normal)
                addButton.titleLabel?.font = .systemFont(ofSize: 14, weight: .semibold)
                addButton.frame = CGRect(x: 0, y: 0, width: 50, height: 30)
                annotationView?.rightCalloutAccessoryView = addButton
            } else {
                annotationView?.annotation = annotation
            }
            
            return annotationView
        }
        
        if let poiAnnotation = annotation as? POIAnnotation {
            let identifier = "POIAnnotation"
            var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            if annotationView == nil {
                annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                annotationView?.canShowCallout = true
                annotationView?.markerTintColor = .systemPurple
                annotationView?.glyphText = "★"
                annotationView?.displayPriority = .required
                annotationView?.clusteringIdentifier = "POICluster"
                let addButton = UIButton(type: .contactAdd)
                annotationView?.rightCalloutAccessoryView = addButton
            } else {
                annotationView?.annotation = annotation
            }
            return annotationView
        }
        
        if let projectAnnotation = annotation as? ProjectAnnotation {
            let identifier = "ProjectAnnotation"
            var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            
            if annotationView == nil {
                annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                annotationView?.canShowCallout = true
                annotationView?.markerTintColor = .systemBlue
                annotationView?.glyphImage = UIImage(systemName: "cube.transparent")
                annotationView?.clusteringIdentifier = "ProjectCluster"
                
                let viewButton = UIButton(type: .system)
                if #available(iOS 15.0, *) {
                    var config = UIButton.Configuration.filled()
                    config.title = "View"
                    config.baseBackgroundColor = .systemBlue
                    viewButton.configuration = config
                } else {
                    viewButton.setTitle("View", for: .normal)
                }
                viewButton.frame = CGRect(x: 0, y: 0, width: 60, height: 30)
                annotationView?.rightCalloutAccessoryView = viewButton
            } else {
                annotationView?.annotation = annotation
            }
            
            return annotationView
        }
        
        if let markerAnnotation = annotation as? MarkerAnnotation {
            let identifier = "MarkerAnnotation"
            var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: identifier) as? MKMarkerAnnotationView
            
            if annotationView == nil {
                annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: identifier)
                annotationView?.canShowCallout = true
                annotationView?.markerTintColor = .systemOrange
                annotationView?.clusteringIdentifier = "MarkerCluster"
                
                let infoButton = UIButton(type: .detailDisclosure)
                annotationView?.rightCalloutAccessoryView = infoButton
            } else {
                annotationView?.annotation = annotation
            }
            
            return annotationView
        }
        
        return nil
    }
    
    func mapView(_ mapView: MKMapView, annotationView view: MKAnnotationView, calloutAccessoryControlTapped control: UIControl) {
        // 임시 annotation의 "Add Here" 버튼 클릭
        if let tempAnnotation = view.annotation as? MKPointAnnotation, tempAnnotation === temporaryAnnotation {
            showAddMarkerDialog(at: tempAnnotation.coordinate)
            return
        }
        
        if let poiAnnotation = view.annotation as? POIAnnotation {
            mapView.deselectAnnotation(poiAnnotation, animated: true)
            prepareSuggestion(from: poiAnnotation.mapItem)
            if let tempAnnotation = temporaryAnnotation {
                mapView.removeAnnotation(tempAnnotation)
                temporaryAnnotation = nil
            }
            showAddMarkerDialog(at: poiAnnotation.coordinate)
            return
        }
        
        if let projectAnnotation = view.annotation as? ProjectAnnotation {
            print("📊 Project Selected: \(projectAnnotation.project.name)")
            dismiss(animated: true) {
                NotificationCenter.default.post(
                    name: NSNotification.Name("ShowMarkerProject"),
                    object: projectAnnotation.project.location
                )
            }
            return
        }
        
        // 저장된 마커의 정보 버튼 클릭
        if let markerAnnotation = view.annotation as? MarkerAnnotation {
            mapView.deselectAnnotation(markerAnnotation, animated: true)
            selectedMarker = markerAnnotation.marker
            showMarkerDetail(marker: markerAnnotation.marker)
        }
    }
    
    func mapView(_ mapView: MKMapView, didSelect annotation: MKAnnotation) {
        // 마커 어노테이션 선택 시 바로 상세 화면 표시
        if let markerAnnotation = annotation as? MarkerAnnotation {
            mapView.deselectAnnotation(annotation, animated: false)
            selectedMarker = markerAnnotation.marker
            showMarkerDetail(marker: markerAnnotation.marker)
        }
    }

    func mapView(_ mapView: MKMapView, regionDidChangeAnimated animated: Bool) {
        schedulePOIFetch()
    }
    
    private func showAddMarkerDialog(at coordinate: CLLocationCoordinate2D) {
        let alert = UIAlertController(
            title: "마커 추가",
            message: "마커 정보를 입력하세요",
            preferredStyle: .alert
        )
        
        alert.addTextField { [weak self] textField in
            textField.placeholder = "마커 이름 (예: 서울역)"
            textField.autocapitalizationType = .words
            if let suggested = self?.lastSuggestedName {
                textField.text = suggested
            }
        }
        
        alert.addTextField { [weak self] textField in
            textField.placeholder = "설명 (선택사항)"
            textField.autocapitalizationType = .sentences
            if let description = self?.lastSuggestedDescription {
                textField.text = description
            }
        }
        
        alert.addAction(UIAlertAction(title: "취소", style: .cancel) { [weak self] _ in
            self?.clearTemporarySelection()
        })
        
        alert.addAction(UIAlertAction(title: "추가", style: .default) { [weak self] _ in
            guard let self = self,
                  let name = alert.textFields?[0].text?.trimmingCharacters(in: .whitespacesAndNewlines),
                  !name.isEmpty else {
                self?.clearTemporarySelection()
                return
            }
            
            let descriptionInput = alert.textFields?[1].text?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
            let description = descriptionInput.isEmpty ? (self.lastSuggestedDescription ?? "User-added marker") : descriptionInput
            let address = self.lastSuggestedAddress
            
            // 새 마커 생성
            let newMarker = LocationMarker(
                name: name,
                description: description,
                latitude: coordinate.latitude,
                longitude: coordinate.longitude,
                address: address,
                placeId: nil,
                imageUrls: []
            )
            
            // 실제 마커 추가
            self.addNewMarker(newMarker)
            self.clearTemporarySelection()
            
            // 성공 햅틱 피드백
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.success)
        })
        
        present(alert, animated: true)
    }
    
    private func showMarkerDetail(marker: LocationMarker) {
        let detailView = MarkerDetailView(
            marker: marker,
            onScanHere: { [weak self] in
                self?.handleScanHere(marker: marker)
            },
            onViewData: { [weak self] in
                self?.handleViewData(marker: marker)
            }
        )
        
        let hostingController = UIHostingController(rootView: detailView)
        hostingController.modalPresentationStyle = .pageSheet
        
        if let sheet = hostingController.sheetPresentationController {
            sheet.detents = [.medium()]
            sheet.prefersGrabberVisible = true
        }
        
        present(hostingController, animated: true)
    }
    
    private func handleScanHere(marker: LocationMarker) {
        print("🎯 Scan Here 버튼 클릭: \(marker.name)")
        // MarkerDetailView(sheet) + MapViewController 한 번에 닫기
        // presentingViewController(MainVC)에서 dismiss하면 전체 체인이 닫힘
        let presenter = self.presentingViewController
        presenter?.dismiss(animated: true) {
            NotificationCenter.default.post(
                name: NSNotification.Name("StartScanAtLocation"),
                object: marker
            )
        }
    }
    
    private func handleViewData(marker: LocationMarker) {
        print("📊 View Data 버튼 클릭: \(marker.name)")
        // MarkerDetailView(sheet)만 닫고, MapVC 위에 스캔 목록 표시
        dismiss(animated: true) { [weak self] in
            guard let self = self else { return }
            let locationScansVC = LocationScansViewController(marker: marker)
            let navController = UINavigationController(rootViewController: locationScansVC)
            navController.modalPresentationStyle = .pageSheet
            self.present(navController, animated: true)
        }
    }
}

// MARK: - Custom Annotation
class POIAnnotation: NSObject, MKAnnotation {
    let mapItem: MKMapItem
    var coordinate: CLLocationCoordinate2D { mapItem.placemark.coordinate }
    var title: String? { mapItem.name }
    var subtitle: String? { mapItem.placemark.title }
    
    init(mapItem: MKMapItem) {
        self.mapItem = mapItem
        super.init()
    }
}

class MarkerAnnotation: NSObject, MKAnnotation {
    let marker: LocationMarker
    var coordinate: CLLocationCoordinate2D {
        marker.coordinate
    }
    var title: String? {
        marker.name
    }
    var subtitle: String? {
        marker.description
    }
    
    init(marker: LocationMarker) {
        self.marker = marker
        super.init()
    }
}

class ProjectAnnotation: NSObject, MKAnnotation {
    let project: Project
    var coordinate: CLLocationCoordinate2D {
        project.location?.coordinate ?? CLLocationCoordinate2D(latitude: 0, longitude: 0)
    }
    var title: String? {
        project.name
    }
    var subtitle: String? {
        "Scanned on \(DateFormatter.localizedString(from: project.createdAt, dateStyle: .short, timeStyle: .short))"
    }
    
    init(project: Project) {
        self.project = project
        super.init()
    }
}

// MARK: - Helpers
private extension MapViewController {
    func placeTemporaryAnnotation(at coordinate: CLLocationCoordinate2D, feedbackStyle: UIImpactFeedbackGenerator.FeedbackStyle) {
        // 기존 임시 annotation 제거
        if let tempAnnotation = temporaryAnnotation {
            mapView.removeAnnotation(tempAnnotation)
        }
        poiSearch?.cancel()
        poiSearch = nil
        geocoder.cancelGeocode()
        lastSuggestedName = nil
        lastSuggestedDescription = nil
        lastSuggestedAddress = nil
        
        // 새 임시 annotation 생성
        let annotation = MKPointAnnotation()
        annotation.coordinate = coordinate
        annotation.title = "Add Here"
        annotation.subtitle = "Tap \"Add Here\" to save this spot"
        
        temporaryAnnotation = annotation
        mapView.addAnnotation(annotation)
        mapView.selectAnnotation(annotation, animated: true)
        
        let generator = UIImpactFeedbackGenerator(style: feedbackStyle)
        generator.impactOccurred()
        
        print("📍 임시 마커 생성: \(coordinate.latitude), \(coordinate.longitude)")
        fetchPlaceSuggestion(near: coordinate)
    }
    
    func clearTemporarySelection() {
        if let tempAnnotation = temporaryAnnotation {
            mapView.deselectAnnotation(tempAnnotation, animated: true)
            mapView.removeAnnotation(tempAnnotation)
        }
        temporaryAnnotation = nil
        poiSearch?.cancel()
        poiSearch = nil
        geocoder.cancelGeocode()
        lastSuggestedName = nil
        lastSuggestedDescription = nil
        lastSuggestedAddress = nil
    }
    
    func fetchPlaceSuggestion(near coordinate: CLLocationCoordinate2D) {
        if #available(iOS 13.0, *) {
            let request = MKLocalPointsOfInterestRequest(center: coordinate, radius: 200)
            let search = MKLocalSearch(request: request)
            poiSearch = search
            search.start { [weak self] response, error in
                guard let self = self else { return }
                defer { self.poiSearch = nil }
                guard let currentAnnotation = self.temporaryAnnotation, self.distance(currentAnnotation.coordinate, coordinate) < 250 else { return }
                if let items = response?.mapItems, let nearest = items.min(by: { self.distance($0.placemark.coordinate, coordinate) < self.distance($1.placemark.coordinate, coordinate) }) {
                    let subtitle = nearest.placemark.title ?? nearest.placemark.subtitle
                    let categoryDescription = nearest.pointOfInterestCategory?.rawValue.replacingOccurrences(of: "MKPointOfInterestCategory", with: "")
                    self.applySuggestion(name: nearest.name, description: categoryDescription?.replacingOccurrences(of: "_", with: " "), address: subtitle)
                    self.updateTemporaryAnnotation(title: nearest.name, subtitle: subtitle)
                    return
                }
                self.reverseGeocode(coordinate: coordinate)
            }
        } else {
            reverseGeocode(coordinate: coordinate)
        }
    }
    
    func updateTemporaryAnnotation(title: String?, subtitle: String?) {
        DispatchQueue.main.async { [weak self] in
            guard let self = self, let annotation = self.temporaryAnnotation else { return }
            if let title = title, !title.isEmpty {
                annotation.title = title
            }
            if let subtitle = subtitle, !subtitle.isEmpty {
                annotation.subtitle = subtitle
            }
            self.mapView.selectAnnotation(annotation, animated: true)
        }
    }
    
    func applySuggestion(name: String?, description: String?, address: String?) {
        lastSuggestedName = name
        lastSuggestedAddress = address
        if let description = description, !description.isEmpty {
            let cleaned = description.replacingOccurrences(of: "_", with: " ").trimmingCharacters(in: .whitespacesAndNewlines)
            lastSuggestedDescription = cleaned.capitalized
        } else {
            lastSuggestedDescription = address
        }
    }
    
    func reverseGeocode(coordinate: CLLocationCoordinate2D) {
        let location = CLLocation(latitude: coordinate.latitude, longitude: coordinate.longitude)
        geocoder.reverseGeocodeLocation(location) { [weak self] placemarks, _ in
            guard let self = self else { return }
            guard let currentAnnotation = self.temporaryAnnotation, self.distance(currentAnnotation.coordinate, coordinate) < 250 else { return }
            guard let placemark = placemarks?.first else { return }
            let name = placemark.name ?? placemark.areasOfInterest?.first
            let address = placemark.compactAddress
            self.applySuggestion(name: name, description: placemark.locality ?? placemark.subLocality, address: address)
            self.updateTemporaryAnnotation(title: name ?? "Add Here", subtitle: address ?? "Tap \"Add Here\" to save this spot")
        }
    }
    
    func distance(_ lhs: CLLocationCoordinate2D, _ rhs: CLLocationCoordinate2D) -> CLLocationDistance {
        let lhsLocation = CLLocation(latitude: lhs.latitude, longitude: lhs.longitude)
        let rhsLocation = CLLocation(latitude: rhs.latitude, longitude: rhs.longitude)
        return lhsLocation.distance(from: rhsLocation)
    }

    func prepareSuggestion(from mapItem: MKMapItem) {
        lastSuggestedName = mapItem.name
        if let category = mapItem.pointOfInterestCategory?.rawValue.replacingOccurrences(of: "MKPointOfInterestCategory", with: "") {
            let cleaned = category.replacingOccurrences(of: "_", with: " ").trimmingCharacters(in: .whitespacesAndNewlines)
            lastSuggestedDescription = cleaned.capitalized
        } else if let subtitle = mapItem.placemark.title {
            lastSuggestedDescription = subtitle
        }
        lastSuggestedAddress = mapItem.placemark.title ?? mapItem.placemark.subtitle
    }
}

// MARK: - UIGestureRecognizerDelegate
extension MapViewController {
    func gestureRecognizer(_ gestureRecognizer: UIGestureRecognizer, shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer) -> Bool {
        return true
    }
}

// MARK: - UISearchBarDelegate
extension MapViewController: UISearchBarDelegate {
    func searchBar(_ searchBar: UISearchBar, textDidChange searchText: String) {
        if searchText.isEmpty {
            searchResults.removeAll()
            searchResultsTable.isHidden = true
            searchTableHeightConstraint.constant = 0
        } else {
            searchCompleter.queryFragment = searchText
        }
    }

    func searchBarSearchButtonClicked(_ searchBar: UISearchBar) {
        searchBar.resignFirstResponder()
        guard let query = searchBar.text, !query.isEmpty else { return }

        let request = MKLocalSearch.Request()
        request.naturalLanguageQuery = query
        request.region = mapView.region

        MKLocalSearch(request: request).start { [weak self] response, _ in
            guard let self = self, let item = response?.mapItems.first else { return }
            let region = MKCoordinateRegion(
                center: item.placemark.coordinate,
                span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01)
            )
            self.mapView.setRegion(region, animated: true)
            self.placeTemporaryAnnotation(at: item.placemark.coordinate, feedbackStyle: .medium)
            self.prepareSuggestion(from: item)
            self.hideSearchResults()
        }
    }

    func searchBarCancelButtonClicked(_ searchBar: UISearchBar) {
        searchBar.text = ""
        searchBar.resignFirstResponder()
        hideSearchResults()
    }

    private func hideSearchResults() {
        searchResults.removeAll()
        searchResultsTable.isHidden = true
        searchTableHeightConstraint.constant = 0
        searchBar.resignFirstResponder()
    }
}

// MARK: - MKLocalSearchCompleterDelegate
extension MapViewController: MKLocalSearchCompleterDelegate {
    func completerDidUpdateResults(_ completer: MKLocalSearchCompleter) {
        searchResults = Array(completer.results.prefix(5))
        searchResultsTable.reloadData()
        let rowHeight: CGFloat = 50
        let height = min(CGFloat(searchResults.count) * rowHeight, 250)
        searchTableHeightConstraint.constant = height
        searchResultsTable.isHidden = searchResults.isEmpty
    }

    func completer(_ completer: MKLocalSearchCompleter, didFailWithError error: Error) {
        print("⚠️ 검색 자동완성 실패: \(error.localizedDescription)")
    }
}

// MARK: - UITableViewDelegate, UITableViewDataSource
extension MapViewController: UITableViewDelegate, UITableViewDataSource {
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        searchResults.count
    }

    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "SearchCell", for: indexPath)
        let result = searchResults[indexPath.row]
        var config = cell.defaultContentConfiguration()
        config.text = result.title
        config.secondaryText = result.subtitle
        config.textProperties.font = .systemFont(ofSize: 15, weight: .medium)
        config.secondaryTextProperties.font = .systemFont(ofSize: 12)
        config.secondaryTextProperties.color = .secondaryLabel
        config.image = UIImage(systemName: "mappin.and.ellipse")
        config.imageProperties.tintColor = .systemBlue
        cell.contentConfiguration = config
        cell.backgroundColor = .clear
        return cell
    }

    func tableView(_ tableView: UITableView, heightForRowAt indexPath: IndexPath) -> CGFloat {
        50
    }

    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        tableView.deselectRow(at: indexPath, animated: true)
        let completion = searchResults[indexPath.row]

        let request = MKLocalSearch.Request(completion: completion)
        MKLocalSearch(request: request).start { [weak self] response, _ in
            guard let self = self, let item = response?.mapItems.first else { return }
            let region = MKCoordinateRegion(
                center: item.placemark.coordinate,
                span: MKCoordinateSpan(latitudeDelta: 0.005, longitudeDelta: 0.005)
            )
            self.mapView.setRegion(region, animated: true)
            self.placeTemporaryAnnotation(at: item.placemark.coordinate, feedbackStyle: .medium)
            self.prepareSuggestion(from: item)
            self.hideSearchResults()
            self.searchBar.text = completion.title
        }
    }
}

private extension UIView {
    func findSuperview<T: UIView>(of type: T.Type) -> T? {
        var current: UIView? = self
        while let candidate = current {
            if let match = candidate as? T {
                return match
            }
            current = candidate.superview
        }
        return nil
    }
}

private extension CLPlacemark {
    var compactAddress: String? {
        var components: [String] = []
        if let name = name { components.append(name) }
        if let thoroughfare = thoroughfare { components.append(thoroughfare) }
        if let locality = locality { components.append(locality) }
        if let administrativeArea = administrativeArea { components.append(administrativeArea) }
        if let country = country { components.append(country) }
        if components.isEmpty { return nil }
        return components.joined(separator: ", ")
    }
}
