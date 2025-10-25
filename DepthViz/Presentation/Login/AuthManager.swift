//
//  AuthManager.swift
//  DepthViz
//
//  Created by HYUNAHKO on 9/7/24.
//  Copyright © 2024 Apple. All rights reserved.
//

import Foundation

class AuthManager {
    static let shared = AuthManager()

    func login(email: String, password: String) -> Bool {
        let trimmedEmail = email.trimmingCharacters(in: .whitespacesAndNewlines)
        let trimmedPassword = password.trimmingCharacters(in: .whitespacesAndNewlines)
        
        // UserModel을 통해 로그인 검증
        if let user = UserModel.shared.loginCheck(id: trimmedEmail, pwd: trimmedPassword) {
            UserModel.shared.currentUser = user  // 로그인한 사용자를 현재 사용자로 설정
            UserDefaults.standard.set(true, forKey: "isLoggedIn")  // 로그인 상태 저장
            return true
        }
        
        UserDefaults.standard.set(false, forKey: "isLoggedIn")  // 로그인 실패 시 false 저장
        return false
    }

    func logout() {
        UserModel.shared.currentUser = nil
        UserDefaults.standard.set(false, forKey: "isLoggedIn")  // 로그아웃 시 false 저장
    }
}
