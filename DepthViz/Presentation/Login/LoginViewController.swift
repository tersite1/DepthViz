//
//  LoginViewController.swift
//  DepthViz
//
//  Created by HYUNAHKO on 9/7/24.
//  Copyright © 2024 Apple. All rights reserved.
//

import UIKit

class LoginViewController: UIViewController, UITextFieldDelegate {

    @IBOutlet weak var emailTextField: UITextField!
    @IBOutlet weak var passwordTextField: UITextField!
    @IBOutlet weak var loginButton: UIButton!

    override func viewDidLoad() {
        super.viewDidLoad()

        // 텍스트 필드 delegate 설정
        emailTextField.delegate = self
        passwordTextField.delegate = self

        // UITapGestureRecognizer 추가하여 빈 화면을 탭했을 때 키보드를 내림
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
        view.addGestureRecognizer(tapGesture)
    }

    // 빈 화면을 터치하면 키보드를 내림
    @objc func dismissKeyboard() {
        view.endEditing(true)
    }

    // Return 키를 눌렀을 때 호출되는 메서드
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if textField == emailTextField {
            // 이메일 입력 후 비밀번호 필드로 이동
            passwordTextField.becomeFirstResponder()
        } else if textField == passwordTextField {
            // 비밀번호 입력 후 키보드를 내림
            textField.resignFirstResponder()
            // 로그인 처리
            handleLogin(loginButton)
        }
        return true
    }

    @IBAction func handleLogin(_ sender: UIButton) {
        guard let email = emailTextField.text, !email.isEmpty,
              let password = passwordTextField.text, !password.isEmpty else {
            showAlert(message: "Please enter your email and password.")
            return
        }

        // 로그인 처리
        loginUser(email: email, password: password)
    }

    private func loginUser(email: String, password: String) {
        // AuthManager를 통한 로그인 시도 (동기 방식으로 가정)
        let success = AuthManager.shared.login(email: email, password: password)

        if success {
            // 로그인 성공 시 MainVC로 전환
            switchToMainViewController()
        } else {
            // 로그인 실패 시 알림 표시
            showAlert(message: "Invalid email or password.")
        }//
        //  LoginViewController.swift
        //  DepthViz
        //
        //  Created by HYUNAHKO on 9/7/24.
        //  Copyright © 2024 Apple. All rights reserved.
        //


        class LoginViewController: UIViewController, UITextFieldDelegate {

            @IBOutlet weak var emailTextField: UITextField!
            @IBOutlet weak var passwordTextField: UITextField!
            @IBOutlet weak var loginButton: UIButton!

            override func viewDidLoad() {
                super.viewDidLoad()

                // 텍스트 필드 delegate 설정
                emailTextField.delegate = self
                passwordTextField.delegate = self

                // UITapGestureRecognizer 추가하여 빈 화면을 탭했을 때 키보드를 내림
                let tapGesture = UITapGestureRecognizer(target: self, action: #selector(dismissKeyboard))
                view.addGestureRecognizer(tapGesture)
            }

            // 빈 화면을 터치하면 키보드를 내림
            @objc func dismissKeyboard() {
                view.endEditing(true)
            }

            // Return 키를 눌렀을 때 호출되는 메서드
            func textFieldShouldReturn(_ textField: UITextField) -> Bool {
                if textField == emailTextField {
                    // 이메일 입력 후 비밀번호 필드로 이동
                    passwordTextField.becomeFirstResponder()
                } else if textField == passwordTextField {
                    // 비밀번호 입력 후 키보드를 내림
                    textField.resignFirstResponder()
                    // 로그인 처리
                    handleLogin(loginButton)
                }
                return true
            }

            @IBAction func handleLogin(_ sender: UIButton) {
                guard let email = emailTextField.text, !email.isEmpty,
                      let password = passwordTextField.text, !password.isEmpty else {
                    showAlert(message: "Please enter your email and password.")
                    return
                }

                // 로그인 처리
                loginUser(email: email, password: password)
            }

            private func loginUser(email: String, password: String) {
                // AuthManager를 통한 로그인 시도 (동기 방식으로 가정)
                let success = AuthManager.shared.login(email: email, password: password)

                if success {
                    // 로그인 성공 시 MainVC로 전환
                    switchToMainViewController()
                } else {
                    // 로그인 실패 시 알림 표시
                    showAlert(message: "Invalid email or password.")
                }
            }

            private func switchToMainViewController() {
                if let windowScene = view.window?.windowScene {
                    let storyboard = UIStoryboard(name: "Main", bundle: nil)
                    if let mainVC = storyboard.instantiateViewController(withIdentifier: "MainVC") as? MainVC {
                        let navigationVC = UINavigationController(rootViewController: mainVC)
                        windowScene.windows.first?.rootViewController = navigationVC
                        windowScene.windows.first?.makeKeyAndVisible()
                    } else {
                        showAlert(message: "Unable to load MainVC.")
                    }
                }
            }

            private func showAlert(message: String) {
                let alert = UIAlertController(title: "Login Error", message: message, preferredStyle: .alert)
                alert.addAction(UIAlertAction(title: "OK", style: .default))
                present(alert, animated: true)
            }
        }
    }

    private func switchToMainViewController() {
        if let windowScene = view.window?.windowScene {
            let storyboard = UIStoryboard(name: "Main", bundle: nil)
            if let mainVC = storyboard.instantiateViewController(withIdentifier: "MainVC") as? MainVC {
                let navigationVC = UINavigationController(rootViewController: mainVC)
                windowScene.windows.first?.rootViewController = navigationVC
                windowScene.windows.first?.makeKeyAndVisible()
            } else {
                showAlert(message: "Unable to load MainVC.")
            }
        }
    }

    private func showAlert(message: String) {
        let alert = UIAlertController(title: "Login Error", message: message, preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: "OK", style: .default))
        present(alert, animated: true)
    }
}
