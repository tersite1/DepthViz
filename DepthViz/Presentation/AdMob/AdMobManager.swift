//
//  AdMobManager.swift
//  DepthViz
//
//  Google AdMob 광고 관리 클래스
//

import UIKit
import GoogleMobileAds

/// AdMob 배너 광고를 관리하는 매니저 클래스
final class AdMobManager: NSObject {
    static let shared = AdMobManager()
    
    // MARK: - 광고 ID 설정
    #if DEBUG
    // 테스트용 광고 ID (개발 중에 사용)
    private let bannerAdUnitID = "ca-app-pub-3940256099942544/2934735716" // Google 테스트 배너 ID
    #else
    // 실제 광고 ID (배포용)
    private let bannerAdUnitID = "ca-app-pub-2516597008794244/6421361743"
    #endif
    
    private var bannerView: GADBannerView?
    
    private override init() {
        super.init()
    }
    
    /// AdMob SDK 초기화
    func initializeAdMob(completion: @escaping () -> Void) {
        GADMobileAds.sharedInstance().start { status in
            print("AdMob SDK initialized successfully")
            completion()
        }
    }
    
    /// 배너 광고 생성 및 반환
    /// - Parameters:
    ///   - viewController: 광고를 표시할 View Controller
    ///   - position: 광고 위치 (top 또는 bottom)
    /// - Returns: 설정된 GADBannerView
    func createBannerAd(for viewController: UIViewController, at position: BannerPosition = .bottom) -> GADBannerView {
        let bannerView = GADBannerView(adSize: GADAdSizeBanner)
        bannerView.adUnitID = bannerAdUnitID
        bannerView.rootViewController = viewController
        bannerView.delegate = self
        bannerView.translatesAutoresizingMaskIntoConstraints = false
        
        self.bannerView = bannerView
        
        // 광고 요청
        let request = GADRequest()
        bannerView.load(request)
        
        return bannerView
    }
    
    /// 배너 광고를 View Controller에 추가
    /// - Parameters:
    ///   - viewController: 광고를 추가할 View Controller
    ///   - position: 광고 위치
    func addBannerToViewController(_ viewController: UIViewController, at position: BannerPosition = .bottom) {
        let bannerView = createBannerAd(for: viewController, at: position)
        viewController.view.addSubview(bannerView)
        
        // 배너 제약조건 설정
        NSLayoutConstraint.activate([
            bannerView.centerXAnchor.constraint(equalTo: viewController.view.centerXAnchor),
            bannerView.widthAnchor.constraint(equalToConstant: GADAdSizeBanner.size.width),
            bannerView.heightAnchor.constraint(equalToConstant: GADAdSizeBanner.size.height)
        ])
        
        switch position {
        case .top:
            NSLayoutConstraint.activate([
                bannerView.topAnchor.constraint(equalTo: viewController.view.safeAreaLayoutGuide.topAnchor)
            ])
        case .bottom:
            NSLayoutConstraint.activate([
                bannerView.bottomAnchor.constraint(equalTo: viewController.view.safeAreaLayoutGuide.bottomAnchor)
            ])
        }
    }
    
    /// 배너 광고 제거
    func removeBanner() {
        bannerView?.removeFromSuperview()
        bannerView = nil
    }
}

// MARK: - GADBannerViewDelegate
extension AdMobManager: GADBannerViewDelegate {
    func bannerViewDidReceiveAd(_ bannerView: GADBannerView) {
        print("✅ AdMob Banner loaded successfully")
    }
    
    func bannerView(_ bannerView: GADBannerView, didFailToReceiveAdWithError error: Error) {
        print("❌ AdMob Banner failed to load: \(error.localizedDescription)")
    }
    
    func bannerViewDidRecordImpression(_ bannerView: GADBannerView) {
        print("📊 AdMob Banner impression recorded")
    }
    
    func bannerViewWillPresentScreen(_ bannerView: GADBannerView) {
        print("📱 AdMob Banner will present screen")
    }
    
    func bannerViewWillDismissScreen(_ bannerView: GADBannerView) {
        print("📱 AdMob Banner will dismiss screen")
    }
    
    func bannerViewDidDismissScreen(_ bannerView: GADBannerView) {
        print("📱 AdMob Banner did dismiss screen")
    }
}

// MARK: - Supporting Types
enum BannerPosition {
    case top
    case bottom
}

