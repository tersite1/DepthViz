import Foundation
import StoreKit

/// StoreKit 2 인앱결제 매니저
@MainActor
class PremiumPurchaseManager: ObservableObject {
    static let shared = PremiumPurchaseManager()

    static let productIDs = ["depthviz.jaden"]

    @Published var product: Product?
    @Published var isPurchasing = false
    @Published var isLoadingProduct = false
    @Published var productLoadFailed = false
    @Published var errorMessage: String?

    private var transactionListener: Task<Void, Never>?
    private var loadRetryCount = 0
    private let maxRetries = 3
    private var _sk1Delegate: SK1Delegate?

    private init() {
        transactionListener = listenForTransactions()
        Task { await loadProduct() }
        Task { await checkEntitlement() }
    }

    deinit {
        transactionListener?.cancel()
    }

    // MARK: - Product Loading

    func loadProduct() async {
        guard !isLoadingProduct else { return }
        isLoadingProduct = true
        productLoadFailed = false
        errorMessage = nil

        let canPay = SKPaymentQueue.canMakePayments()
        // StoreKit Configuration 파일 확인
        let storekitFiles = Bundle.main.paths(forResourcesOfType: "storekit", inDirectory: nil)
        let receiptURL = Bundle.main.appStoreReceiptURL?.absoluteString ?? "nil"
        let isSandbox = receiptURL.contains("sandboxReceipt")
        print("🔍 IAP 디버그: productIDs=\(Self.productIDs), Bundle='\(Bundle.main.bundleIdentifier ?? "nil")', canMakePayments=\(canPay)")
        print("🔍 IAP 환경: sandbox=\(isSandbox), receiptURL=\(receiptURL)")
        print("🔍 IAP 번들 내 storekit 파일: \(storekitFiles)")

        if !canPay {
            print("❌ IAP: 결제 불가 상태 (기기 제한 또는 설정 문제)")
        }

        // StoreKit 1 fallback 진단
        print("🔍 IAP: SKProductsRequest로도 시도...")
        let sk1Result = await withCheckedContinuation { (continuation: CheckedContinuation<[String], Never>) in
            let request = SKProductsRequest(productIdentifiers: Set(Self.productIDs))
            let delegate = SK1Delegate(continuation: continuation)
            self._sk1Delegate = delegate
            request.delegate = delegate
            request.start()
        }
        print("🔍 IAP SK1 결과: valid=\(sk1Result)")

        for attempt in 1...maxRetries {
            do {
                print("🔍 IAP 시도 \(attempt): Product.products(for: \(Self.productIDs)) 호출...")
                let products = try await Product.products(for: Self.productIDs)
                print("🔍 IAP 시도 \(attempt): 반환된 상품 수 = \(products.count)")
                for (i, p) in products.enumerated() {
                    print("🔍 IAP 상품[\(i)]: id='\(p.id)' displayName='\(p.displayName)' price=\(p.displayPrice) type=\(p.type)")
                }
                if let p = products.first {
                    self.product = p
                    self.isLoadingProduct = false
                    self.loadRetryCount = 0
                    print("✅ 상품 로드 성공: \(p.displayName) (\(p.displayPrice))")
                    return
                } else {
                    print("⚠️ 상품 없음 (시도 \(attempt)/\(maxRetries)) — 빈 배열 반환됨, 두 ID 모두 실패")
                }
            } catch {
                print("❌ 상품 로드 실패 (시도 \(attempt)/\(maxRetries)): \(error) — \(error.localizedDescription)")
            }

            if attempt < maxRetries {
                try? await Task.sleep(nanoseconds: 2_000_000_000)
            }
        }

        // 모든 재시도 실패
        isLoadingProduct = false
        productLoadFailed = true
        errorMessage = NSLocalizedString("iap_load_failed", comment: "")
    }

    // MARK: - Purchase

    func purchase() async {
        guard let product = product else {
            errorMessage = NSLocalizedString("iap_product_unavailable", comment: "")
            return
        }
        isPurchasing = true
        errorMessage = nil

        do {
            let result = try await product.purchase()
            switch result {
            case .success(let verification):
                let transaction = try checkVerified(verification)
                await transaction.finish()
                PremiumManager.shared.unlock()
                print("✅ 구매 성공: \(product.id)")
            case .userCancelled:
                print("ℹ️ 사용자가 구매 취소")
            case .pending:
                errorMessage = NSLocalizedString("iap_pending", comment: "")
                print("⏳ 구매 대기 중 (Ask to Buy 등)")
            @unknown default:
                break
            }
        } catch {
            errorMessage = String(format: NSLocalizedString("iap_purchase_failed", comment: ""), error.localizedDescription)
            print("❌ 구매 실패: \(error)")
        }

        isPurchasing = false
    }

    // MARK: - Restore

    func restorePurchases() async {
        isPurchasing = true
        errorMessage = nil
        var found = false

        for await result in Transaction.currentEntitlements {
            if let transaction = try? checkVerified(result) {
                if Self.productIDs.contains(transaction.productID) {
                    PremiumManager.shared.unlock()
                    found = true
                    print("✅ 구매 복원 성공")
                }
            }
        }

        if !found {
            errorMessage = NSLocalizedString("iap_restore_empty", comment: "")
        }
        isPurchasing = false
    }

    // MARK: - Entitlement Check

    func checkEntitlement() async {
        for await result in Transaction.currentEntitlements {
            if let transaction = try? checkVerified(result) {
                if Self.productIDs.contains(transaction.productID) {
                    PremiumManager.shared.unlock()
                    return
                }
            }
        }
    }

    // MARK: - Transaction Listener

    private func listenForTransactions() -> Task<Void, Never> {
        Task.detached { [weak self] in
            for await result in Transaction.updates {
                guard let self = self else { return }
                if let transaction = try? await self.checkVerified(result) {
                    if await Self.productIDs.contains(transaction.productID) {
                        await MainActor.run {
                            PremiumManager.shared.unlock()
                        }
                    }
                    await transaction.finish()
                }
            }
        }
    }

    // MARK: - Helpers

    private func checkVerified<T>(_ result: VerificationResult<T>) throws -> T {
        switch result {
        case .unverified(_, let error):
            throw error
        case .verified(let safe):
            return safe
        }
    }
}

// MARK: - StoreKit 1 진단용 Delegate

private class SK1Delegate: NSObject, SKProductsRequestDelegate {
    private var continuation: CheckedContinuation<[String], Never>?

    init(continuation: CheckedContinuation<[String], Never>) {
        self.continuation = continuation
    }

    func productsRequest(_ request: SKProductsRequest, didReceive response: SKProductsResponse) {
        let validIDs = response.products.map { $0.productIdentifier }
        let invalidIDs = response.invalidProductIdentifiers
        print("🔍 SK1 응답: valid=\(validIDs), invalid=\(invalidIDs)")
        continuation?.resume(returning: validIDs)
        continuation = nil
    }

    func request(_ request: SKRequest, didFailWithError error: Error) {
        print("❌ SK1 에러: \(error)")
        continuation?.resume(returning: [])
        continuation = nil
    }
}
