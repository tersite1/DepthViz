import Foundation
import StoreKit

/// StoreKit 2 인앱결제 매니저
@MainActor
class PremiumPurchaseManager: ObservableObject {
    static let shared = PremiumPurchaseManager()

    static let productID = "depthviz.kaist"

    @Published var product: Product?
    @Published var isPurchasing = false
    @Published var errorMessage: String?

    private var transactionListener: Task<Void, Never>?

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
        do {
            let products = try await Product.products(for: [Self.productID])
            if let p = products.first {
                self.product = p
                print("✅ 상품 로드 성공: \(p.displayName) (\(p.displayPrice))")
            } else {
                print("⚠️ 상품 ID '\(Self.productID)'에 해당하는 상품 없음 — App Store Connect 확인 필요")
                print("   체크리스트:")
                print("   1. Xcode > Signing & Capabilities > + In-App Purchase 추가")
                print("   2. App Store Connect > 수익 창출 > 인앱 구매 > 상품 등록")
                print("   3. 상품 ID가 '\(Self.productID)'과 정확히 일치하는지 확인")
                print("   4. 유료 앱 계약(Paid Applications Agreement) 서명 완료 확인")
                // 3초 후 재시도
                try? await Task.sleep(nanoseconds: 3_000_000_000)
                let retry = try await Product.products(for: [Self.productID])
                if let p = retry.first {
                    self.product = p
                    print("✅ 재시도 성공: \(p.displayName)")
                }
            }
        } catch {
            print("❌ 상품 로드 실패: \(error.localizedDescription)")
            errorMessage = "상품을 불러올 수 없습니다. 네트워크를 확인해주세요."
        }
    }

    // MARK: - Purchase

    func purchase() async {
        guard let product = product else {
            errorMessage = "상품을 불러올 수 없습니다."
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
                print("⏳ 구매 대기 중 (Ask to Buy 등)")
            @unknown default:
                break
            }
        } catch {
            errorMessage = "구매 실패: \(error.localizedDescription)"
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
                if transaction.productID == Self.productID {
                    PremiumManager.shared.unlock()
                    found = true
                    print("✅ 구매 복원 성공")
                }
            }
        }

        if !found {
            errorMessage = "복원할 구매 내역이 없습니다."
        }
        isPurchasing = false
    }

    // MARK: - Entitlement Check

    func checkEntitlement() async {
        for await result in Transaction.currentEntitlements {
            if let transaction = try? checkVerified(result) {
                if transaction.productID == Self.productID {
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
                    if await transaction.productID == Self.productID {
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
