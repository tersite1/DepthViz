import SwiftUI
import AVFoundation

// MARK: - UIHostingController 배경 투명화

struct ClearBackgroundView: UIViewRepresentable {
    func makeUIView(context: Context) -> UIView {
        let view = UIView()
        DispatchQueue.main.async {
            view.superview?.superview?.backgroundColor = .clear
        }
        return view
    }
    func updateUIView(_ uiView: UIView, context: Context) {}
}

// MARK: - Looping Video Player (UIViewRepresentable)

struct LoopingVideoPlayer: UIViewRepresentable {
    let fileName: String
    var contentOffsetX: CGFloat = 0

    func makeUIView(context: Context) -> UIView {
        let container = UIView()
        container.backgroundColor = .clear
        container.clipsToBounds = true

        guard let url = Bundle.main.url(forResource: fileName, withExtension: "mp4") else {
            return container
        }

        let item = AVPlayerItem(url: url)
        let player = AVQueuePlayer(playerItem: item)
        player.isMuted = true

        let templateItem = AVPlayerItem(url: url)
        let looper = AVPlayerLooper(player: player, templateItem: templateItem)
        context.coordinator.looper = looper
        context.coordinator.player = player

        let playerLayer = AVPlayerLayer(player: player)
        playerLayer.videoGravity = .resizeAspectFill
        container.layer.addSublayer(playerLayer)
        context.coordinator.playerLayer = playerLayer
        context.coordinator.offsetX = contentOffsetX

        player.play()
        return container
    }

    func updateUIView(_ uiView: UIView, context: Context) {
        DispatchQueue.main.async {
            let bounds = uiView.bounds
            let offsetX = context.coordinator.offsetX
            context.coordinator.playerLayer?.frame = CGRect(
                x: offsetX,
                y: 0,
                width: bounds.width - offsetX,
                height: bounds.height
            )
        }
    }

    func makeCoordinator() -> Coordinator { Coordinator() }

    class Coordinator {
        var player: AVQueuePlayer?
        var looper: AVPlayerLooper?
        var playerLayer: AVPlayerLayer?
        var offsetX: CGFloat = 0

        deinit {
            player?.pause()
            looper?.disableLooping()
        }
    }
}

// MARK: - Premium Video Popup

struct PremiumVideoPopup: View {
    @Environment(\.dismiss) private var dismiss
    @StateObject private var purchaseManager = PremiumPurchaseManager.shared

    var body: some View {
        ZStack {
            // 배경 투명 — 현재 화면이 그대로 보임
            Color.clear.ignoresSafeArea()
                .onTapGesture { dismiss() }

            // 플로팅 카드
            VStack(spacing: 0) {
                // 닫기 + 타이틀
                ZStack(alignment: .topTrailing) {
                    VStack(spacing: 6) {
                        Text(NSLocalizedString("premium_title", comment: ""))
                            .font(.system(size: 20, weight: .bold))
                            .foregroundColor(.white)
                        Text(NSLocalizedString("premium_subtitle", comment: ""))
                            .font(.system(size: 14))
                            .foregroundColor(.white.opacity(0.7))
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.top, 20)
                    .padding(.bottom, 4)

                    Button(action: { dismiss() }) {
                        Image(systemName: "xmark.circle.fill")
                            .font(.system(size: 24))
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .padding(.top, 12)
                    .padding(.trailing, 12)
                }

                // 비디오 영역
                HStack(spacing: 0) {
                    LoopingVideoPlayer(fileName: "imu_demo", contentOffsetX: -20)
                        .frame(height: 200)
                    LoopingVideoPlayer(fileName: "trajectory_demo")
                        .frame(height: 200)
                }
                .clipShape(RoundedRectangle(cornerRadius: 0))
                .padding(.top, 12)

                // 기능 설명
                VStack(alignment: .leading, spacing: 6) {
                    featureRow(NSLocalizedString("premium_feature_imu", comment: ""))
                    featureRow(NSLocalizedString("premium_feature_odometry", comment: ""))
                    featureRow(NSLocalizedString("premium_feature_export", comment: ""))
                    featureRow(NSLocalizedString("premium_feature_icon", comment: ""))
                    featureRow(NSLocalizedString("premium_feature_no_ads", comment: ""))
                }
                .padding(.horizontal, 20)
                .padding(.top, 16)

                // 구매 버튼
                Button(action: {
                    Task { await purchaseManager.purchase() }
                }) {
                    HStack {
                        if purchaseManager.isPurchasing {
                            ProgressView()
                                .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        } else if let product = purchaseManager.product {
                            Text(product.displayPrice)
                                .fontWeight(.bold)
                            Text(NSLocalizedString("premium_unlock", comment: ""))
                        } else {
                            ProgressView()
                                .progressViewStyle(CircularProgressViewStyle(tint: .white))
                                .frame(width: 16, height: 16)
                            Text(NSLocalizedString("premium_unlock", comment: ""))
                                .onAppear {
                                    Task { await purchaseManager.loadProduct() }
                                }
                        }
                    }
                    .font(.system(size: 16, weight: .semibold))
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .frame(height: 48)
                    .background(
                        LinearGradient(
                            colors: [Color.blue, Color.blue.opacity(0.7)],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(12)
                }
                .disabled(purchaseManager.isPurchasing)
                .padding(.horizontal, 20)
                .padding(.top, 18)

                // 에러 메시지
                if let error = purchaseManager.errorMessage {
                    Text(error)
                        .font(.system(size: 12))
                        .foregroundColor(.red.opacity(0.8))
                        .padding(.top, 4)
                }

                Spacer().frame(height: 20)
            }
            .background(
                RoundedRectangle(cornerRadius: 20)
                    .fill(Color(white: 0.12))
                    .shadow(color: .black.opacity(0.6), radius: 20, x: 0, y: 8)
            )
            .clipShape(RoundedRectangle(cornerRadius: 20))
            .padding(.horizontal, 20)
        }
        .onChange(of: PremiumManager.shared.isPremium) { isPremium in
            if isPremium { dismiss() }
        }
    }

    // MARK: - Components

    private func featureRow(_ text: String) -> some View {
        HStack(spacing: 8) {
            Circle()
                .fill(Color.blue)
                .frame(width: 5, height: 5)
            Text(text)
                .font(.system(size: 13))
                .foregroundColor(.white.opacity(0.85))
        }
    }
}
