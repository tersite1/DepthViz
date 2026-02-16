//
//  AdMobManager.swift
//  DepthViz
//
//  Google AdMob 광고 관리 클래스
//

import UIKit
#if canImport(GoogleMobileAds)
import GoogleMobileAds
#endif

// MARK: - Supporting Types
enum BannerPosition {
    case top
    case bottom
}

/// AdMob 배너 광고를 관리하는 매니저 클래스
final class AdMobManager: NSObject {
    static let shared = AdMobManager()

    #if DEBUG
    private let bannerAdUnitID = "ca-app-pub-3940256099942544/2934735716"
    #else
    private let bannerAdUnitID = "ca-app-pub-2516597008794244/6421361743"
    #endif

    #if canImport(GoogleMobileAds)
    private var bannerView: GADBannerView?
    #endif

    private override init() {
        super.init()
    }

    func initializeAdMob(completion: @escaping () -> Void) {
        #if canImport(GoogleMobileAds)
        GADMobileAds.sharedInstance().start { status in
            print("AdMob SDK initialized successfully")
            completion()
        }
        #else
        completion()
        #endif
    }

    #if canImport(GoogleMobileAds)
    func createBannerAd(for viewController: UIViewController, at position: BannerPosition = .bottom) -> GADBannerView {
        let bannerView = GADBannerView(adSize: GADAdSizeBanner)
        bannerView.adUnitID = bannerAdUnitID
        bannerView.rootViewController = viewController
        bannerView.delegate = self
        bannerView.translatesAutoresizingMaskIntoConstraints = false

        self.bannerView = bannerView
        bannerView.load(GADRequest())

        return bannerView
    }

    func addBannerToViewController(_ viewController: UIViewController, at position: BannerPosition = .bottom) {
        let bannerView = createBannerAd(for: viewController, at: position)
        viewController.view.addSubview(bannerView)

        NSLayoutConstraint.activate([
            bannerView.centerXAnchor.constraint(equalTo: viewController.view.centerXAnchor),
            bannerView.widthAnchor.constraint(equalToConstant: GADAdSizeBanner.size.width),
            bannerView.heightAnchor.constraint(equalToConstant: GADAdSizeBanner.size.height)
        ])

        switch position {
        case .top:
            bannerView.topAnchor.constraint(equalTo: viewController.view.safeAreaLayoutGuide.topAnchor).isActive = true
        case .bottom:
            bannerView.bottomAnchor.constraint(equalTo: viewController.view.safeAreaLayoutGuide.bottomAnchor).isActive = true
        }
    }
    #endif

    func removeBanner() {
        #if canImport(GoogleMobileAds)
        bannerView?.removeFromSuperview()
        bannerView = nil
        #endif
    }
}

#if canImport(GoogleMobileAds)
extension AdMobManager: GADBannerViewDelegate {
    func bannerViewDidReceiveAd(_ bannerView: GADBannerView) {
        print("✅ AdMob Banner loaded successfully")
    }
    func bannerView(_ bannerView: GADBannerView, didFailToReceiveAdWithError error: Error) {
        print("❌ AdMob Banner failed to load: \(error.localizedDescription)")
    }
    func bannerViewDidRecordImpression(_ bannerView: GADBannerView) {}
    func bannerViewWillPresentScreen(_ bannerView: GADBannerView) {}
    func bannerViewWillDismissScreen(_ bannerView: GADBannerView) {}
    func bannerViewDidDismissScreen(_ bannerView: GADBannerView) {}
}
#endif

