//
//  UserModel.swift
//  DepthViz
//
//  Created by snlcom on 6/6/24.
//

import Foundation

class UserModel: ObservableObject {
    static let shared = UserModel()
    @Published var currentUser: User?

    struct User {
        var name: String
        var email: String?
        var password: String
        var role: UserRole
    }

    enum UserRole: String {
        case manager = "Manager"
        case worker = "Worker"
    }

    var users: [User] = [
        User(name: "홍길동", email: "abc1234@naver.com", password: "qwerty1234", role: .manager),
        User(name: "고현아", email: "admin@gmail.com", password: "1234", role: .worker)
    ]
    
    func isValidEmail(id: String) -> Bool {
        let emailRegEx = "[A-Z0-9a-z._%+-]+@[A-Za-z0-9.-]+\\.[A-Za-z]{2,}"
        let emailTest = NSPredicate(format: "SELF MATCHES %@", emailRegEx)
        return emailTest.evaluate(with: id)
    }
    
    func isValidPassword(pwd: String) -> Bool {
        let passwordRegEx = "^[a-zA-Z0-9]{8,}$"
        let passwordTest = NSPredicate(format: "SELF MATCHES %@", passwordRegEx)
        return passwordTest.evaluate(with: pwd)
    }

    func loginCheck(id: String, pwd: String) -> User? {
        print("로그인 시도 이메일: \(id), 비밀번호: \(pwd)")  // 디버그용 로그
        return users.first { $0.email == id && $0.password == pwd }
    }
    
    func registerUser(name: String, email: String, password: String, role: UserRole) -> Bool {
        guard !users.contains(where: { $0.email == email }) else {
            return false
        }
        guard isValidEmail(id: email) && isValidPassword(pwd: password) else {
            return false
        }
        let newUser = User(name: name, email: email, password: password, role: role)
        users.append(newUser)
        return true
    }
    
    func fetchCurrentUser(email: String) {
        currentUser = users.first { $0.email == email }
    }
}
