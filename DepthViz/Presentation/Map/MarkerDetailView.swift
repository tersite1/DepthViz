//
//  MarkerDetailView.swift
//  DepthViz
//
//  마커 상세 정보 화면 (SwiftUI)
//

import SwiftUI

struct MarkerDetailView: View {
    let marker: LocationMarker
    let onScanHere: () -> Void
    let onViewData: () -> Void
    
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        VStack(spacing: 0) {
            // 헤더
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Image(systemName: "mappin.circle.fill")
                        .font(.system(size: 40))
                        .foregroundColor(.orange)
                    
                    VStack(alignment: .leading, spacing: 4) {
                        Text(marker.name)
                            .font(.title2)
                            .fontWeight(.bold)
                        
                        Text(marker.description)
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                    }
                    
                    Spacer()
                }
                
                if let address = marker.address {
                    HStack(spacing: 6) {
                        Image(systemName: "location.fill")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        Text(address)
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                    .padding(.top, 4)
                }
            }
            .padding(20)
            
            Divider()
            
            // 좌표 정보
            HStack(spacing: 20) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Latitude")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Text(String(format: "%.6f", marker.latitude))
                        .font(.system(.body, design: .monospaced))
                        .fontWeight(.medium)
                }
                
                Divider()
                    .frame(height: 30)
                
                VStack(alignment: .leading, spacing: 4) {
                    Text("Longitude")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Text(String(format: "%.6f", marker.longitude))
                        .font(.system(.body, design: .monospaced))
                        .fontWeight(.medium)
                }
                
                Spacer()
            }
            .padding(.horizontal, 20)
            .padding(.vertical, 16)
            
            Divider()
            
            // 버튼 영역
            VStack(spacing: 12) {
                // Scan Here 버튼
                Button(action: onScanHere) {
                    HStack {
                        Image(systemName: "camera.fill")
                            .font(.system(size: 18, weight: .semibold))
                        Text("Scan Here")
                            .font(.system(size: 17, weight: .semibold))
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .frame(height: 54)
                    .background(
                        LinearGradient(
                            gradient: Gradient(colors: [Color.blue, Color.blue.opacity(0.8)]),
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(12)
                }
                
                // View Data 버튼
                Button(action: onViewData) {
                    HStack {
                        Image(systemName: "folder.fill")
                            .font(.system(size: 18, weight: .semibold))
                        Text("View Data")
                            .font(.system(size: 17, weight: .semibold))
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .frame(height: 54)
                    .background(
                        LinearGradient(
                            gradient: Gradient(colors: [Color.orange, Color.orange.opacity(0.8)]),
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(12)
                }
            }
            .padding(20)
            
            Spacer()
        }
        .background(Color(UIColor.systemBackground))
    }
}

#Preview {
    MarkerDetailView(
        marker: LocationMarker.samples[0],
        onScanHere: { print("Scan Here") },
        onViewData: { print("View Data") }
    )
}

