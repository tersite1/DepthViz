//
//  AlgorithmSelectionView.swift
//  DepthViz
//
//  Algorithm selection settings UI (SwiftUI)
//

import SwiftUI
import UIKit

// SLAMAlgorithm enum is defined in Domain/Entity/ScanSettings.swift

private extension SLAMAlgorithm {
    var displayName: String {
        switch self {
        case .depthViz: return "DV-SLAM (Recommended)"
        case .arkit: return "ARKit (Apple Default)"
        }
    }
}

struct AlgorithmSelectionView: View {
    @State private var selectedAlgorithm: SLAMAlgorithm = .depthViz
    @State private var confidenceThreshold: Double = 0.5
    @State private var maxDistance: Double = 5.0
    @State private var selectedFormat: FileFormat = .plyBinary
    @ObservedObject private var premiumManager = PremiumManager.shared
    @State private var showPremiumPopup = false
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        NavigationView {
            ScrollView {
                VStack(spacing: 20) {
                    algorithmSection
                    confidenceSection
                    distanceSection
                    fileFormatSection
                    premiumSection
                    developerSection
                    Spacer(minLength: 40)
                }
                .padding(.horizontal, 16)
                .padding(.top, 16)
            }
            .background(Color(uiColor: .systemGroupedBackground))
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button(action: { dismiss() }) {
                        Image(systemName: "xmark.circle.fill")
                            .font(.system(size: 22))
                            .foregroundStyle(.secondary)
                    }
                }
            }
        }
        .interactiveDismissDisabled(false)
        .onAppear {
            loadSettings()
        }
    }

    private func loadSettings() {
        selectedAlgorithm = ScanSettings.shared.algorithm
        confidenceThreshold = Double(ScanSettings.shared.confidenceLevel.slamThreshold)
        maxDistance = Double(ScanSettings.shared.distanceLimit.distanceValue)
        if maxDistance > 10 { maxDistance = 10 }
        selectedFormat = ScanSettings.shared.fileFormat
    }

    // MARK: - Sections

    private var algorithmSection: some View {
        sectionCard {
            sectionHeader("SLAM Algorithm", icon: "cpu.fill")

            ForEach(SLAMAlgorithm.allCases, id: \.self) { algorithm in
                algorithmRow(algorithm)
            }
        }
    }

    private var confidenceSection: some View {
        sectionCard {
            sectionHeader("Confidence Threshold", icon: "eye.fill")

            Text("Points with confidence below this threshold will be filtered out.")
                .font(.system(size: 13))
                .foregroundColor(.secondary)
                .fixedSize(horizontal: false, vertical: true)

            HStack {
                Slider(value: $confidenceThreshold, in: 0...1, step: 0.05)
                    .accentColor(.blue)
                    .onChange(of: confidenceThreshold) { _ in saveSettings() }

                Text(confidenceThreshold < 0.33 ? "Low" : confidenceThreshold < 0.66 ? "Medium" : "High")
                    .font(.system(size: 15, weight: .medium))
                    .foregroundColor(.primary)
                    .frame(width: 70, alignment: .trailing)
            }

            Text("Higher values = stricter filtering")
                .font(.system(size: 11))
                .foregroundColor(.secondary)
        }
    }

    private var distanceSection: some View {
        sectionCard {
            sectionHeader("Max Scan Distance", icon: "scope")

            Text("Maximum distance for valid points (in meters).")
                .font(.system(size: 13))
                .foregroundColor(.secondary)
                .fixedSize(horizontal: false, vertical: true)

            HStack {
                Slider(value: $maxDistance, in: 1...10, step: 0.5)
                    .accentColor(.blue)
                    .onChange(of: maxDistance) { _ in saveSettings() }

                Text(String(format: "%.1f m", maxDistance))
                    .font(.system(size: 15, design: .monospaced))
                    .foregroundColor(.primary)
                    .frame(width: 70, alignment: .trailing)
            }
        }
    }

    private var fileFormatSection: some View {
        sectionCard {
            sectionHeader("Export Format", icon: "doc.fill")

            ForEach(FileFormat.allCases, id: \.self) { format in
                Button(action: {
                    selectedFormat = format
                    ScanSettings.shared.fileFormat = format
                }) {
                    HStack(spacing: 12) {
                        Image(systemName: selectedFormat == format ? "checkmark.circle.fill" : "circle")
                            .foregroundColor(selectedFormat == format ? .blue : .gray.opacity(0.5))
                            .font(.system(size: 18))

                        VStack(alignment: .leading, spacing: 2) {
                            Text(format.displayName)
                                .font(.system(size: 15, weight: selectedFormat == format ? .semibold : .regular))
                                .foregroundColor(selectedFormat == format ? .blue : .primary)
                            Text(".\(format.fileExtension)")
                                .font(.system(size: 12))
                                .foregroundColor(.secondary)
                        }
                        Spacer()
                    }
                    .padding(.vertical, 8)
                }
                .buttonStyle(.plain)
            }
        }
    }

    private var premiumSection: some View {
        sectionCard {
            sectionHeader("Premium", icon: "crown.fill")

            if premiumManager.isPremium {
                Toggle(isOn: $premiumManager.showOdometry) {
                    Label("Odometry 시각화", systemImage: "point.topleft.down.curvedto.point.bottomright.up")
                        .font(.system(size: 15))
                }
                .tint(.blue)

                Toggle(isOn: $premiumManager.showIMUData) {
                    Label("IMU 데이터 표시", systemImage: "chart.bar.fill")
                        .font(.system(size: 15))
                }
                .tint(.blue)

                Toggle(isOn: Binding(
                    get: { premiumManager.usePremiumIcon },
                    set: { premiumManager.setPremiumIcon($0) }
                )) {
                    Label("프리미엄 아이콘", systemImage: "app.badge.checkmark")
                        .font(.system(size: 15))
                }
                .tint(.blue)
            } else {
                VStack(alignment: .leading, spacing: 8) {
                    Text("프리미엄 기능을 잠금 해제하세요")
                        .font(.system(size: 13))
                        .foregroundColor(.secondary)

                    HStack(spacing: 6) {
                        Image(systemName: "lock.fill").font(.caption).foregroundColor(.orange)
                        Text("Odometry, IMU 데이터, 프리미엄 아이콘")
                            .font(.system(size: 12)).foregroundColor(.secondary)
                    }

                    Button(action: { showPremiumPopup = true }) {
                        Text("프리미엄 잠금 해제")
                            .font(.system(size: 15, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .frame(height: 44)
                            .background(LinearGradient(colors: [.blue, .blue.opacity(0.7)], startPoint: .leading, endPoint: .trailing))
                            .cornerRadius(10)
                    }
                    .padding(.top, 4)
                }
            }
        }
        .fullScreenCover(isPresented: $showPremiumPopup) {
            PremiumVideoPopup()
                .background(ClearBackgroundView())
        }
    }

    private var developerSection: some View {
        sectionCard {
            sectionHeader("Developer", icon: "person.fill")

            VStack(alignment: .leading, spacing: 4) {
                Text("장민석")
                    .font(.system(size: 15, weight: .semibold))
                Text("항상 피곤한 대학원생입니다.")
                    .font(.system(size: 13))
                    .foregroundColor(.secondary)
                Text("소통 및 코웍 제안 환영합니다!")
                    .font(.system(size: 13))
                    .foregroundColor(.secondary)
            }

            // 개발자에게 커피 사주기
            Button(action: {
                // 카카오뱅크 계좌 복사
                UIPasteboard.general.string = "3333-10-0243235"
            }) {
                HStack(spacing: 12) {
                    Image(systemName: "cup.and.saucer.fill")
                        .font(.system(size: 20))
                        .foregroundColor(.orange)

                    VStack(alignment: .leading, spacing: 2) {
                        Text("개발자에게 커피 사주기")
                            .font(.system(size: 15, weight: .medium))
                            .foregroundColor(.primary)
                        Text("카카오뱅크 3333-10-0243235 장민석")
                            .font(.system(size: 12))
                            .foregroundColor(.secondary)
                        Text("탭하면 계좌번호가 복사됩니다")
                            .font(.system(size: 11))
                            .foregroundColor(.blue)
                    }

                    Spacer()

                    Image(systemName: "doc.on.doc")
                        .font(.system(size: 14))
                        .foregroundColor(.blue)
                }
                .padding(.vertical, 8)
            }
            .buttonStyle(.plain)

            // 개발자 웹사이트
            Button(action: {
                if let url = URL(string: "https://tersite1.github.io") {
                    UIApplication.shared.open(url)
                }
            }) {
                HStack(spacing: 12) {
                    Image(systemName: "globe")
                        .font(.system(size: 20))
                        .foregroundColor(.blue)

                    VStack(alignment: .leading, spacing: 2) {
                        Text("개발자 웹사이트")
                            .font(.system(size: 15, weight: .medium))
                            .foregroundColor(.primary)
                        Text("tersite1.github.io")
                            .font(.system(size: 12))
                            .foregroundColor(.secondary)
                    }

                    Spacer()

                    Image(systemName: "arrow.up.right.square")
                        .font(.system(size: 14))
                        .foregroundColor(.blue)
                }
                .padding(.vertical, 8)
            }
            .buttonStyle(.plain)
        }
    }

    // MARK: - Components

    private func sectionCard<Content: View>(@ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 12) {
            content()
        }
        .padding(16)
        .background(
            RoundedRectangle(cornerRadius: 12)
                .fill(Color(uiColor: .secondarySystemGroupedBackground))
        )
    }

    private func sectionHeader(_ title: String, icon: String) -> some View {
        HStack(spacing: 8) {
            Image(systemName: icon)
                .font(.system(size: 15))
                .foregroundColor(.blue)

            Text(title)
                .font(.system(size: 15, weight: .semibold))
                .foregroundColor(.primary)
        }
    }

    private func algorithmRow(_ algorithm: SLAMAlgorithm) -> some View {
        Button(action: {
            selectedAlgorithm = algorithm
            saveSettings()
        }) {
            HStack(spacing: 12) {
                Image(systemName: selectedAlgorithm == algorithm ? "checkmark.circle.fill" : "circle")
                    .foregroundColor(selectedAlgorithm == algorithm ? .blue : .gray.opacity(0.5))
                    .font(.system(size: 18))

                VStack(alignment: .leading, spacing: 3) {
                    Text(algorithm.displayName)
                        .font(.system(size: 15, weight: selectedAlgorithm == algorithm ? .semibold : .regular))
                        .foregroundColor(selectedAlgorithm == algorithm ? .blue : .primary)

                    if selectedAlgorithm == algorithm {
                        Text(algorithm.description)
                            .font(.system(size: 12))
                            .foregroundColor(.secondary)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }

                Spacer()
            }
            .padding(.vertical, 8)
        }
        .buttonStyle(.plain)
    }

    private func saveSettings() {
        ScanSettings.shared.algorithm = selectedAlgorithm

        if confidenceThreshold < 0.33 {
            ScanSettings.shared.confidenceLevel = .low
        } else if confidenceThreshold < 0.66 {
            ScanSettings.shared.confidenceLevel = .medium
        } else {
            ScanSettings.shared.confidenceLevel = .high
        }

        UserDefaults.standard.set(maxDistance, forKey: "ScanDistanceLimit")
        SLAMService.sharedInstance().reloadSettings()

        #if DEBUG
        print("Settings saved: algo=\(selectedAlgorithm.rawValue), conf=\(ScanSettings.shared.confidenceLevel.rawValue), dist=\(maxDistance)")
        #endif
    }
}

struct AlgorithmSelectionView_Previews: PreviewProvider {
    static var previews: some View {
        AlgorithmSelectionView()
    }
}
