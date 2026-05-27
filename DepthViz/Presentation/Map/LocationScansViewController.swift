//
//  LocationScansViewController.swift
//  DepthViz
//
//  특정 위치의 스캔 데이터 목록 화면
//

import UIKit
import SwiftUI

class LocationScansViewController: UIViewController {
    
    private let marker: LocationMarker
    private var scans: [ScanInfo] = []
    
    init(marker: LocationMarker) {
        self.marker = marker
        super.init(nibName: nil, bundle: nil)
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        setupNavigationBar()
        loadScans()
        setupUI()
    }
    
    private func setupNavigationBar() {
        title = marker.name
        navigationController?.navigationBar.prefersLargeTitles = true
        
        let closeButton = UIBarButtonItem(
            barButtonSystemItem: .close,
            target: self,
            action: #selector(closeButtonTapped)
        )
        navigationItem.rightBarButtonItem = closeButton
    }
    
    private func loadScans() {
        print("📊 \(marker.name)의 스캔 데이터 로드 중...")
        // 해당 마커 이름과 일치하는 프로젝트의 스캔만 필터링
        scans = ScanStorage.shared.infos.filter { $0.project == marker.name }
        print("📊 총 \(ScanStorage.shared.infos.count)개 중 \(scans.count)개 필터됨")
    }
    
    private func setupUI() {
        let hostingController = UIHostingController(
            rootView: LocationScansView(
                marker: marker,
                scans: scans,
                onClose: { [weak self] in
                    self?.dismiss(animated: true)
                }
            )
        )
        
        addChild(hostingController)
        view.addSubview(hostingController.view)
        hostingController.view.frame = view.bounds
        hostingController.view.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        hostingController.didMove(toParent: self)
    }
    
    @objc private func closeButtonTapped() {
        dismiss(animated: true)
    }
}

// MARK: - SwiftUI View
struct LocationScansView: View {
    let marker: LocationMarker
    let scans: [ScanInfo]
    let onClose: () -> Void
    
    var body: some View {
        VStack(spacing: 0) {
            // 위치 정보 헤더
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    Image(systemName: "mappin.circle.fill")
                        .font(.system(size: 32))
                        .foregroundColor(.orange)
                    
                    VStack(alignment: .leading, spacing: 4) {
                        Text(marker.name)
                            .font(.title3)
                            .fontWeight(.bold)
                        
                        Text(marker.description)
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                    }
                    
                    Spacer()
                }
                
                if let address = marker.address {
                    Text(address)
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding()
            .background(Color(UIColor.secondarySystemBackground))
            
            Divider()
            
            // 스캔 데이터 목록
            if scans.isEmpty {
                VStack(spacing: 16) {
                    Spacer()
                    
                    Image(systemName: "doc.text.magnifyingglass")
                        .font(.system(size: 64))
                        .foregroundColor(.secondary)
                    
                    Text("No Scans Found")
                        .font(.title3)
                        .fontWeight(.semibold)
                    
                    Text("There are no scans for this location yet.\nTap 'Scan Here' to create one.")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                    
                    Spacer()
                }
                .padding()
            } else {
                List(scans) { scan in
                    ScanRow(scan: scan)
                }
            }
        }
        .navigationBarTitleDisplayMode(.inline)
    }
}

struct ScanRow: View {
    let scan: ScanInfo
    
    var body: some View {
        HStack {
            Image(systemName: "cube.fill")
                .font(.title2)
                .foregroundColor(.blue)
            
            VStack(alignment: .leading, spacing: 4) {
                Text(scan.fileName)
                    .font(.body)
                    .fontWeight(.medium)
                
                Text(scan.date.formatted())
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            
            Spacer()
        }
        .padding(.vertical, 8)
    }
}

#Preview {
    LocationScansView(
        marker: LocationMarker.samples[0],
        scans: [],
        onClose: {}
    )
}

