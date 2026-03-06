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

// MARK: - Premium Popup (Buy Me a Coffee + 코드 입력)

struct PremiumPopup: View {
    @Environment(\.dismiss) private var dismiss
    @ObservedObject private var premiumManager = PremiumManager.shared

    @State private var showCodeAlert = false
    @State private var codeInput = ""
    @State private var codeResultMessage: String?
    @State private var showCodeResult = false

    private let bmcURL = "https://buymeacoffee.com/tersite"

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

                // 안내 문구
                Text(NSLocalizedString("premium_code_hint", comment: ""))
                    .font(.system(size: 12))
                    .foregroundColor(.white.opacity(0.5))
                    .multilineTextAlignment(.center)
                    .padding(.horizontal, 20)
                    .padding(.top, 10)

                // Buy Me a Coffee 버튼
                Button(action: {
                    if let url = URL(string: bmcURL) {
                        UIApplication.shared.open(url)
                    }
                }) {
                    HStack(spacing: 10) {
                        Image(systemName: "cup.and.saucer.fill")
                            .font(.system(size: 18))
                        Text(NSLocalizedString("premium_buy_coffee", comment: ""))
                            .font(.system(size: 16, weight: .semibold))
                    }
                    .foregroundColor(.black)
                    .frame(maxWidth: .infinity)
                    .frame(height: 48)
                    .background(
                        LinearGradient(
                            colors: [Color.yellow, Color.orange],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(12)
                }
                .padding(.horizontal, 20)
                .padding(.top, 18)

                // 코드 입력 버튼
                Button(action: { showCodeAlert = true }) {
                    Text(NSLocalizedString("premium_enter_code", comment: ""))
                        .font(.system(size: 13))
                        .foregroundColor(.white.opacity(0.5))
                }
                .padding(.top, 8)

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
        .alert("프리미엄 코드 입력", isPresented: $showCodeAlert) {
            TextField("Premium Code", text: $codeInput)
                .autocapitalization(.allCharacters)
                .disableAutocorrection(true)
            Button("확인") {
                let success = premiumManager.validateAndApplyCode(codeInput)
                codeResultMessage = success
                    ? NSLocalizedString("premium_code_success", comment: "")
                    : NSLocalizedString("premium_code_fail", comment: "")
                codeInput = ""
                showCodeResult = true
            }
            Button("취소", role: .cancel) {
                codeInput = ""
            }
        } message: {
            Text(NSLocalizedString("premium_code_prompt", comment: ""))
        }
        .alert(codeResultMessage ?? "", isPresented: $showCodeResult) {
            Button("확인", role: .cancel) {
                if premiumManager.isPremium {
                    dismiss()
                }
            }
        }
        .onChange(of: premiumManager.isPremium) { isPremium in
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
