//
//  NetworkInterceptor.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/04/21.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation
import Alamofire

/// Network request 직전 Auth 설정 및 Token 발급담당 객체
final class NetworkInterceptor: RequestInterceptor {
    /// KakaoApi를 사용하는 경우 Kakao Auth 를 추가 후 request 진행 설정
    func adapt(_ urlRequest: URLRequest, for session: Session, completion: @escaping (Result<URLRequest, Error>) -> Void) {
        guard urlRequest.url?.absoluteString.hasPrefix(NetworkURL.Domain.kakao) == true else {
            completion(.success(urlRequest))
            return
        }
        
        guard let kakaoApiAuth = Bundle.main.infoDictionary?["KAKAO_AUTH"] as? String else {
            print("/// kakao auth 값 오류")
            completion(.failure(AFError.explicitlyCancelled as Error))
            return
        }
        
        var urlRequest = urlRequest
        urlRequest.setValue("KakaoAK \(kakaoApiAuth)", forHTTPHeaderField: "Authorization")
        
        completion(.success(urlRequest))
    }
}
