//
//  ScanSettingsView.swift
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

import SwiftUI
import UIKit

struct ScanSettingsView: View {
    @ObservedObject var settings = ScanSettings.shared
    @Environment(\.presentationMode) var presentationMode
    
    var body: some View {
        ZStack {
            VisualEffectView(effect: UIBlurEffect(style: .systemThinMaterialDark))
                .ignoresSafeArea()
            
            VStack(spacing: 24) {
                // Header
                HStack {
                    Text("Scan Settings")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundColor(.white)
                    Spacer()
                    Button(action: {
                        presentationMode.wrappedValue.dismiss()
                    }) {
                        Image(systemName: "xmark.circle.fill")
                            .font(.system(size: 28))
                            .foregroundColor(.gray)
                    }
                }
                .padding(.top, 20)
                .padding(.horizontal)
                
                ScrollView {
                    VStack(spacing: 30) {
                        // Section 1: Engine Selector
                        VStack(alignment: .leading, spacing: 12) {
                            Text("ENGINE")
                                .font(.caption)
                                .fontWeight(.heavy)
                                .foregroundColor(.gray)
                                .padding(.horizontal)
                            
                            LazyVGrid(columns: [
                                GridItem(.flexible(), spacing: 16),
                                GridItem(.flexible(), spacing: 16)
                            ], spacing: 16) {
                                ForEach(SLAMAlgorithm.allCases, id: \.self) { algo in
                                    EngineCard(
                                        algorithm: algo,
                                        isSelected: settings.algorithm == algo
                                    ) {
                                        withAnimation(.spring()) {
                                            settings.algorithm = algo
                                        }
                                    }
                                }
                            }
                            .padding(.horizontal)
                            
                            Text(settings.algorithm.description)
                                .font(.footnote)
                                .foregroundColor(.white.opacity(0.7))
                                .padding(.horizontal)
                                .animation(.easeInOut, value: settings.algorithm)
                        }
                        
                        // Section 2: Range Control
                        VStack(alignment: .leading, spacing: 12) {
                            HStack {
                                Text("RANGE LIMIT")
                                    .font(.caption)
                                    .fontWeight(.heavy)
                                    .foregroundColor(.gray)
                                Spacer()
                                Text(settings.distanceLimit.displayName)
                                    .font(.caption)
                                    .foregroundColor(Color(UIColor.systemCyan))
                            }
                            .padding(.horizontal)
                            
                            HStack(spacing: 0) {
                                ForEach(DistanceLimit.allCases, id: \.self) { limit in
                                    RangeButton(
                                        limit: limit,
                                        isSelected: settings.distanceLimit == limit
                                    ) {
                                        withAnimation(.interactiveSpring()) {
                                            settings.distanceLimit = limit
                                        }
                                    }
                                }
                            }
                            .background(Color.black.opacity(0.3))
                            .cornerRadius(12)
                            .padding(.horizontal)
                        }
                        
                        // Advanced section removed — only DV-SLAM and ARKit remain
                    }
                    .padding(.bottom, 40)
                }
            }
        }
    }
}

// MARK: - Components

struct EngineCard: View {
    let algorithm: SLAMAlgorithm
    let isSelected: Bool
    let action: () -> Void
    
    var body: some View {
        Button(action: action) {
            ZStack {
                RoundedRectangle(cornerRadius: 16, style: .continuous)
                    .fill(Color(UIColor.systemGray6).opacity(0.15))
                    .overlay(
                        RoundedRectangle(cornerRadius: 16, style: .continuous)
                            .stroke(isSelected ? glowColor : Color.clear, lineWidth: 2)
                    )
                    .shadow(color: isSelected ? glowColor.opacity(0.5) : Color.clear, radius: 10, x: 0, y: 0)
                
                VStack(spacing: 12) {
                    Image(systemName: iconName)
                        .font(.system(size: 32))
                        .foregroundColor(isSelected ? glowColor : .gray)
                    
                    Text(algorithm.rawValue)
                        .font(.headline)
                        .foregroundColor(isSelected ? .white : .gray)
                        .lineLimit(1)
                        .minimumScaleFactor(0.8)
                    
                    Text(algorithm.badge)
                        .font(.caption2)
                        .fontWeight(.bold)
                        .padding(.horizontal, 8)
                        .padding(.vertical, 4)
                        .background(isSelected ? glowColor.opacity(0.2) : Color.black.opacity(0.2))
                        .cornerRadius(8)
                        .foregroundColor(isSelected ? glowColor : .gray)
                }
                .padding()
            }
            .frame(height: 140)
        }
        .buttonStyle(ScaleButtonStyle())
    }
    
    var glowColor: Color {
        switch algorithm {
        case .depthViz: return Color(UIColor.systemCyan)
        case .arkit: return Color(UIColor.systemOrange)
        }
    }

    var iconName: String {
        switch algorithm {
        case .depthViz: return "cpu.fill"
        case .arkit: return "arkit"
        }
    }
}

struct RangeButton: View {
    let limit: DistanceLimit
    let isSelected: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            ZStack {
                if isSelected {
                    RoundedRectangle(cornerRadius: 10)
                        .fill(Color(UIColor.systemCyan).opacity(0.3))
                        .padding(4)
                }

                Text(limit == .noLimit ? "∞" : limit.displayName)
                    .font(.system(size: 14, weight: isSelected ? .bold : .medium))
                    .foregroundColor(isSelected ? Color(UIColor.systemCyan) : .gray)
                    .frame(maxWidth: .infinity)
                    .frame(height: 44)
            }
        }
    }
}

struct ScaleButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .scaleEffect(configuration.isPressed ? 0.95 : 1.0)
            .animation(.easeInOut(duration: 0.1), value: configuration.isPressed)
    }
}

// Helper for UIVisualEffectView in SwiftUI
struct VisualEffectView: UIViewRepresentable {
    var effect: UIVisualEffect?
    func makeUIView(context: UIViewRepresentableContext<Self>) -> UIVisualEffectView { UIVisualEffectView() }
    func updateUIView(_ uiView: UIVisualEffectView, context: UIViewRepresentableContext<Self>) { uiView.effect = effect }
}
