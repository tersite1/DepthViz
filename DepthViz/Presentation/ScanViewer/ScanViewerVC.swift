//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//

import UIKit
import SceneKit

class ScanViewerVC: UIViewController, UIGestureRecognizerDelegate, SCNSceneRendererDelegate {
    private let frameGalleryButton: UIButton = {
        let button = UIButton(type: .system)
        button.setTitle("Show Frames", for: .normal)
        button.translatesAutoresizingMaskIntoConstraints = false
        return button
    }()

    @IBOutlet weak var sceneView: SCNView!
    static let identifier = "ScanViewerVC"
    var scene = SCNScene()
    var lightingEnvironmentContent = UIImage(named: "white-env.jpg")
    var lightingEnvironmentIntensity: CGFloat = 1.0
    var rightFingerView: UIImageView!
    var leftFingerView: UIImageView!
    var idleTimer: Timer?
    let panSequenceKey: String = "panSequence"
    let camera = SCNCamera()
    var cameraNode = SCNNode()
    var cameraOrbitFinal = SCNNode()
    let cameraOrbitStart = SCNNode()
    var widthAngle: Float = 0.0
    var heightAngle: Float = 0.0
    var lastWidthAngle: Float = 0.0
    var lastHeightAngle: Float = 0.0
    var maxHeightAngleXUp: Float = 1
    var maxHeightAngleXDown: Float = -1
    var cameraCurrentZoomScale = 50.0
    var cameraZoomScaleMax = 60.0
    var cameraZoomScaleMin = 0.0
    var maxXPositionRight: Float = 0.0
    var maxXPositionLeft: Float = 0.0
    var maxYPositionUp: Float = 0.0
    var maxYPositionDown: Float = 0.0
    var originalCameraZoomScale: Double!
    var originalWidthAngle: Float!
    var originalHeightAngle: Float!
    var originalPositionX: Float!
    var originalPositionY: Float!
    var positionX: Float = 0.0
    var positionY: Float = 0.0
    
    var fileURL: URL?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Ensure sceneView is not nil
        guard let sceneView = sceneView else {
            print("sceneView is nil")
            return
        }

        if let fileURL = fileURL {
            show3DModel(fileURL: fileURL)
        }

        // Add frame gallery button
        view.addSubview(frameGalleryButton)
        NSLayoutConstraint.activate([
            frameGalleryButton.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -16),
            frameGalleryButton.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16),
            frameGalleryButton.widthAnchor.constraint(equalToConstant: 120),
            frameGalleryButton.heightAnchor.constraint(equalToConstant: 44)
        ])
        frameGalleryButton.addTarget(self, action: #selector(openFrameGallery), for: .touchUpInside)
        
        let panGesture = UIPanGestureRecognizer(target: self, action: #selector(self.panGestureRecognized(gesture:)) )
        panGesture.delegate = self
        sceneView.addGestureRecognizer(panGesture)
        
        let pinchGesture = UIPinchGestureRecognizer(target: self, action: #selector(self.pinchGestureRecognized(gesture:)) )
        pinchGesture.delegate = self
        sceneView.addGestureRecognizer(pinchGesture)
        
        let doubleTapGesture = UITapGestureRecognizer(target: self, action: #selector(self.doubleTapGestureRecognized(gesture:)) )
        doubleTapGesture.delegate = self
        doubleTapGesture.numberOfTapsRequired = 2
        sceneView.addGestureRecognizer(doubleTapGesture)
    }

    @objc private func openFrameGallery() {
        let galleryVC = FrameGalleryVC()
        navigationController?.pushViewController(galleryVC, animated: true)
    }
    
    func show3DModel(fileURL: URL) {
        // Ensure scene can be loaded
        guard let scene = try? SCNScene(url: fileURL, options: nil) else {
            print("Failed to load 3D model from URL: \(fileURL)")
            return
        }
        
        // Setup Camera
        cameraNode.camera = camera
        cameraNode.position = SCNVector3(0, 0, Float(cameraCurrentZoomScale))
        cameraOrbitStart.position = cameraNode.position
        cameraOrbitFinal.addChildNode(cameraNode)
        cameraOrbitFinal.position = SCNVector3(x: 0, y: 0, z: 0)
        cameraOrbitFinal.eulerAngles.y = Float(-2 * Double.pi) * lastWidthAngle
        cameraOrbitFinal.eulerAngles.x = Float(-Double.pi) * lastHeightAngle
        cameraOrbitStart.eulerAngles.x = cameraOrbitFinal.eulerAngles.x
        cameraOrbitStart.eulerAngles.y = cameraOrbitFinal.eulerAngles.y
        scene.rootNode.addChildNode(cameraOrbitFinal)
        scene.rootNode.addChildNode(cameraOrbitStart)
        originalCameraZoomScale = cameraCurrentZoomScale
        originalWidthAngle = widthAngle
        originalHeightAngle = heightAngle
        originalPositionX = positionX
        originalPositionY = positionY
        // Setup Sceneview
        sceneView.delegate = self
        sceneView.backgroundColor = UIColor.white
        sceneView.layer.backgroundColor = UIColor.clear.cgColor
        sceneView.antialiasingMode = .multisampling4X
        sceneView.scene = scene
        startIdleTimer()
    }
    
    @objc func panGestureRecognized(gesture: UIPanGestureRecognizer) {
        guard let sceneView = gesture.view else { return }
        
        if gesture.numberOfTouches == 1 {
            stopFingerAnimationSequence()
            let translation = gesture.translation(in: sceneView)
            widthAngle = Float(translation.x) / Float(sceneView.frame.size.width) + lastWidthAngle
            heightAngle = Float(translation.y) / Float(sceneView.frame.size.height) + lastHeightAngle
            if (heightAngle >= maxHeightAngleXUp ) {
                heightAngle = maxHeightAngleXUp
                lastHeightAngle = heightAngle
                gesture.setTranslation(CGPoint(x: translation.x, y: 0.0), in: sceneView)
            }
            if (heightAngle <= maxHeightAngleXDown ) {
                heightAngle = maxHeightAngleXDown
                lastHeightAngle = heightAngle
                gesture.setTranslation(CGPoint(x: translation.x, y: 0.0), in: sceneView)
            }
            cameraOrbitStart.eulerAngles.y = Float(-2 * Double.pi) * widthAngle
            cameraOrbitStart.eulerAngles.x = Float(-Double.pi) * heightAngle
        }
        else {
            gesture.setTranslation(CGPoint(x: 0.0, y: 0.0), in: sceneView)
            lastWidthAngle = widthAngle
            lastHeightAngle = heightAngle
        }
    }
    
    @objc func pinchGestureRecognized(gesture: UIPinchGestureRecognizer) {
        if gesture.numberOfTouches == 2 {
            stopFingerAnimationSequence()
            var pinchVelocity = Double(gesture.velocity)
            if (pinchVelocity.isNaN) || (pinchVelocity.isInfinite) {
                pinchVelocity = 0.0
            }
            cameraCurrentZoomScale  -= pinchVelocity / 10.0
            if cameraCurrentZoomScale <= cameraZoomScaleMin {
                cameraCurrentZoomScale = cameraZoomScaleMin
            }
            if cameraCurrentZoomScale >= cameraZoomScaleMax {
                cameraCurrentZoomScale = cameraZoomScaleMax
            }
            cameraOrbitStart.position = SCNVector3(x: positionX, y: positionY, z: Float(cameraCurrentZoomScale))
        }
    }
    
    @objc func doubleTapGestureRecognized(gesture: UITapGestureRecognizer) {
        cameraOrbitStart.eulerAngles.y = Float(-2 * Double.pi) * originalWidthAngle
        cameraOrbitStart.eulerAngles.x = Float(-Double.pi) * originalHeightAngle
        cameraOrbitStart.position = SCNVector3(x: originalPositionX, y: originalPositionY, z: Float(originalCameraZoomScale))
        cameraCurrentZoomScale = originalCameraZoomScale
        lastWidthAngle = originalWidthAngle
        lastHeightAngle = originalHeightAngle
    }
    
    func updatePositions() {
        let lerpY = (cameraOrbitStart.eulerAngles.y - cameraOrbitFinal.eulerAngles.y) * 0.5
        let lerpX = (cameraOrbitStart.eulerAngles.x - cameraOrbitFinal.eulerAngles.x) * 0.5
        cameraOrbitFinal.eulerAngles.y += lerpY
        cameraOrbitFinal.eulerAngles.x += lerpX
        
        let lerpZ = (cameraOrbitStart.position.z - cameraNode.position.z) * 0.5
        cameraNode.position.z += lerpZ
    }
    
    func startIdleTimer() {
        idleTimer?.invalidate()
        idleTimer = Timer.scheduledTimer(
            timeInterval: 1.5,
            target: self,
            selector: #selector(handleIdleTimeout),
            userInfo: nil,
            repeats: false)
    }
    
    func fingerAnimationSequenceFinished() {
        startIdleTimer()  // Loop
    }
    
    func stopFingerAnimationSequence() {
        idleTimer?.invalidate()
        idleTimer = nil
        if let rightFingerView = rightFingerView {
            self.rightFingerView = nil
            rightFingerView.layer.removeAllAnimations()
            UIView.animate(withDuration: 0.3, animations: {
                rightFingerView.alpha = 0.0
            }) { _ in
                rightFingerView.removeFromSuperview()
            }
        }
        if let leftFingerView = leftFingerView {
            self.leftFingerView = nil
            leftFingerView.layer.removeAllAnimations()
            UIView.animate(withDuration: 0.3, animations: {
                leftFingerView.alpha = 0.0
            }) { _ in
                leftFingerView.removeFromSuperview()
            }
        }
        cameraOrbitStart.removeAction(forKey: panSequenceKey)
    }
    
    func renderer(_ renderer: SCNSceneRenderer, didApplyAnimationsAtTime time: TimeInterval) {
        updatePositions()
    }
    
    @objc func handleIdleTimeout() {
        // Create Animation Finger
        let center = sceneView.center
        let panAmount: CGFloat = 45.0
        let panDuration: TimeInterval = 0.75
        let panDelay: TimeInterval = 0.1
        rightFingerView = UIImageView(image: UIImage(named: "finger")!)
        rightFingerView.center = center
        rightFingerView.alpha = 0
        sceneView.addSubview(rightFingerView)
        leftFingerView = UIImageView(image: UIImage(named: "finger")!.withHorizontallyFlippedOrientation())
        leftFingerView.center = center
        leftFingerView.alpha = 0
        sceneView.addSubview(leftFingerView)
        let rightFingerFadeIn = CABasicAnimation(keyPath: "opacity")
        rightFingerFadeIn.fromValue = 0
        rightFingerFadeIn.toValue = 1
        rightFingerFadeIn.duration = 0.3
        rightFingerFadeIn.fillMode = .forwards
        rightFingerFadeIn.isRemovedOnCompletion = false
        let rightFingerPanLeft = CABasicAnimation(keyPath: "position.x")
        rightFingerPanLeft.fromValue = center.x
        rightFingerPanLeft.toValue = center.x - panAmount
        rightFingerPanLeft.duration = panDuration
        rightFingerPanLeft.beginTime = CACurrentMediaTime()
        rightFingerPanLeft.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        rightFingerPanLeft.fillMode = .forwards
        rightFingerPanLeft.isRemovedOnCompletion = false
        let rightFingerPanLeftBack = CABasicAnimation(keyPath: "position.x")
        rightFingerPanLeftBack.fromValue = center.x - panAmount
        rightFingerPanLeftBack.toValue = center.x
        rightFingerPanLeftBack.duration = panDuration
        rightFingerPanLeftBack.beginTime = rightFingerPanLeft.beginTime + rightFingerPanLeft.duration + panDelay
        rightFingerPanLeftBack.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        rightFingerPanLeftBack.fillMode = .forwards
        rightFingerPanLeftBack.isRemovedOnCompletion = false
        let rightFingerPanRight = CABasicAnimation(keyPath: "position.x")
        rightFingerPanRight.fromValue = center.x
        rightFingerPanRight.toValue = center.x + panAmount
        rightFingerPanRight.duration = panDuration
        rightFingerPanRight.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        rightFingerPanRight.beginTime = rightFingerPanLeftBack.beginTime + rightFingerPanLeftBack.duration
        rightFingerPanRight.fillMode = .forwards
        rightFingerPanRight.isRemovedOnCompletion = false
        let rightFingerPanRightBack = CABasicAnimation(keyPath: "position.x")
        rightFingerPanRightBack.fromValue = center.x + panAmount
        rightFingerPanRightBack.toValue = center.x
        rightFingerPanRightBack.duration = panDuration
        rightFingerPanRightBack.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        rightFingerPanRightBack.beginTime = rightFingerPanRight.beginTime + rightFingerPanRight.duration + panDelay
        rightFingerPanRightBack.fillMode = .forwards
        rightFingerPanRightBack.isRemovedOnCompletion = false
        let rightFingerPinchTop = CABasicAnimation(keyPath: "position.y")
        rightFingerPinchTop.fromValue = center.y
        rightFingerPinchTop.toValue = center.y - panAmount
        rightFingerPinchTop.duration = panDuration
        rightFingerPinchTop.beginTime = rightFingerPanRightBack.beginTime + rightFingerPanRightBack.duration + panDelay
        rightFingerPinchTop.fillMode = .forwards
        rightFingerPinchTop.isRemovedOnCompletion = false
        let rightFingerPinchRight = CABasicAnimation(keyPath: "position.x")
        rightFingerPinchRight.fromValue = center.x
        rightFingerPinchRight.toValue = center.x + panAmount
        rightFingerPinchRight.duration = panDuration
        rightFingerPinchRight.beginTime = rightFingerPanRightBack.beginTime + rightFingerPanRightBack.duration + panDelay
        rightFingerPinchRight.fillMode = .forwards
        rightFingerPinchRight.isRemovedOnCompletion = false
        let rightFingerPinchTopBack = CABasicAnimation(keyPath: "position.y")
        rightFingerPinchTopBack.fromValue = center.y - panAmount
        rightFingerPinchTopBack.toValue = center.y
        rightFingerPinchTopBack.duration = panDuration
        rightFingerPinchTopBack.beginTime = rightFingerPinchTop.beginTime + rightFingerPinchTop.duration + panDelay + panDelay
        rightFingerPinchTopBack.fillMode = .forwards
        rightFingerPinchTopBack.isRemovedOnCompletion = false
        let rightFingerPinchRightBack = CABasicAnimation(keyPath: "position.x")
        rightFingerPinchRightBack.fromValue = center.x + panAmount
        rightFingerPinchRightBack.toValue = center.x
        rightFingerPinchRightBack.duration = panDuration
        rightFingerPinchRightBack.beginTime = rightFingerPinchRight.beginTime + rightFingerPinchRight.duration + panDelay + panDelay
        rightFingerPinchRightBack.fillMode = .forwards
        rightFingerPinchRightBack.isRemovedOnCompletion = false
        let rightFingerFadeOut = CABasicAnimation(keyPath: "opacity")
        rightFingerFadeOut.fromValue = 1
        rightFingerFadeOut.toValue = 0
        rightFingerFadeOut.duration = 0.3
        rightFingerFadeOut.beginTime = rightFingerPinchRightBack.beginTime + rightFingerPinchRightBack.duration + panDelay
        rightFingerFadeOut.fillMode = .forwards
        rightFingerFadeOut.isRemovedOnCompletion = false
        let leftFingerFadeStay = CABasicAnimation(keyPath: "opacity")
        leftFingerFadeStay.fromValue = 1
        leftFingerFadeStay.toValue = 1
        leftFingerFadeStay.beginTime = rightFingerPanRightBack.beginTime + rightFingerPanRightBack.duration + panDelay
        leftFingerFadeStay.duration = rightFingerPinchRight.beginTime + rightFingerPinchRight.duration + rightFingerPinchRightBack.beginTime + rightFingerPinchRightBack.duration + panDelay
        leftFingerFadeStay.fillMode = .forwards
        leftFingerFadeStay.isRemovedOnCompletion = false
        let leftFingerPinchBottom = CABasicAnimation(keyPath: "position.y")
        leftFingerPinchBottom.fromValue = center.y
        leftFingerPinchBottom.toValue = center.y + panAmount
        leftFingerPinchBottom.duration = panDuration
        leftFingerPinchBottom.beginTime = rightFingerPanRightBack.beginTime + rightFingerPanRightBack.duration + panDelay
        rightFingerPanLeft.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        leftFingerPinchBottom.fillMode = .forwards
        leftFingerPinchBottom.isRemovedOnCompletion = false
        let leftFingerPinchLeft = CABasicAnimation(keyPath: "position.x")
        leftFingerPinchLeft.fromValue = center.x
        leftFingerPinchLeft.toValue = center.x - panAmount
        leftFingerPinchLeft.duration = panDuration
        leftFingerPinchLeft.beginTime = rightFingerPanRightBack.beginTime + rightFingerPanRightBack.duration + panDelay
        rightFingerPanLeft.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        leftFingerPinchLeft.fillMode = .forwards
        leftFingerPinchLeft.isRemovedOnCompletion = false
        let leftFingerPinchBottomBack = CABasicAnimation(keyPath: "position.y")
        leftFingerPinchBottomBack.fromValue = center.y + panAmount
        leftFingerPinchBottomBack.toValue = center.y
        leftFingerPinchBottomBack.duration = panDuration
        leftFingerPinchBottomBack.beginTime = leftFingerPinchBottom.beginTime + leftFingerPinchBottom.duration + panDelay + panDelay
        leftFingerPinchBottomBack.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        leftFingerPinchBottomBack.fillMode = .forwards
        leftFingerPinchBottomBack.isRemovedOnCompletion = false
        let leftFingerPinchLeftBack = CABasicAnimation(keyPath: "position.x")
        leftFingerPinchLeftBack.fromValue = center.x - panAmount
        leftFingerPinchLeftBack.toValue = center.x
        leftFingerPinchLeftBack.duration = panDuration
        leftFingerPinchLeftBack.beginTime = leftFingerPinchLeft.beginTime + leftFingerPinchLeft.duration + panDelay + panDelay
        leftFingerPinchLeftBack.timingFunction = CAMediaTimingFunction(name: .easeInEaseOut)
        leftFingerPinchLeftBack.fillMode = .forwards
        leftFingerPinchLeftBack.isRemovedOnCompletion = false
        let leftFingerFadeOut = CABasicAnimation(keyPath: "opacity")
        leftFingerFadeOut.fromValue = 1
        leftFingerFadeOut.toValue = 0
        leftFingerFadeOut.duration = 0.3
        leftFingerFadeOut.beginTime = leftFingerPinchBottomBack.beginTime + leftFingerPinchBottomBack.duration + panDelay
        leftFingerFadeOut.fillMode = .forwards
        leftFingerFadeOut.isRemovedOnCompletion = false
        // Play Animation Finger
        rightFingerView.layer.add(rightFingerFadeIn, forKey: nil)
        rightFingerView.layer.add(rightFingerPanLeft, forKey: nil)
        rightFingerView.layer.add(rightFingerPanLeftBack, forKey: nil)
        rightFingerView.layer.add(rightFingerPanRight, forKey: nil)
        rightFingerView.layer.add(rightFingerPanRightBack, forKey: nil)
        rightFingerView.layer.add(rightFingerPinchTop, forKey: nil)
        rightFingerView.layer.add(rightFingerPinchRight, forKey: nil)
        rightFingerView.layer.add(rightFingerPinchTopBack, forKey: nil)
        rightFingerView.layer.add(rightFingerPinchRightBack, forKey: nil)
        rightFingerView.layer.add(rightFingerFadeOut, forKey: nil)
        leftFingerView.layer.add(leftFingerFadeStay, forKey: nil)
        leftFingerView.layer.add(leftFingerPinchLeft, forKey: nil)
        leftFingerView.layer.add(leftFingerPinchBottom, forKey: nil)
        leftFingerView.layer.add(leftFingerPinchLeftBack, forKey: nil)
        leftFingerView.layer.add(leftFingerPinchBottomBack, forKey: nil)
        leftFingerView.layer.add(leftFingerFadeOut, forKey: nil)
        // Animation 3D Model
        let panLeft = SCNAction.rotate(toAxisAngle: SCNVector4(0, 1, 0, 0.35), duration: rightFingerPanLeft.duration)
        let panLeftBack = SCNAction.rotate(toAxisAngle: SCNVector4(0, 0, 0, 0), duration: rightFingerPanLeftBack.duration)
        let panIdle = SCNAction.rotate(toAxisAngle: SCNVector4(0, 0, 0, 0), duration: panDelay)
        let panRight = SCNAction.rotate(toAxisAngle: SCNVector4(x:0, y:-1 , z:0, w:0.35), duration: rightFingerPanRight.duration)
        let panRightBack = SCNAction.rotate(toAxisAngle: SCNVector4(x:0, y:0 , z:0, w:0), duration: rightFingerPanRightBack.duration)
        let zoomIn = SCNAction.move(by: SCNVector3(x:0, y:0, z: -5), duration: leftFingerPinchLeft.duration)
        let zoomIdle = SCNAction.move(by: SCNVector3(0, 0, 0), duration: panDelay*4)
        let zoomBack = SCNAction.move(by: SCNVector3(x:0, y:0, z: 5), duration: leftFingerPinchLeftBack.duration)
        let completeAction = SCNAction.run { [weak self] _ in
            DispatchQueue.main.async {
                self?.fingerAnimationSequenceFinished()
            }
        }
        let panSequence = SCNAction.sequence([panLeft, panLeftBack, panIdle, panRight, panRightBack, zoomIn, zoomIdle, zoomBack, completeAction])
        cameraOrbitStart.runAction(panSequence, forKey: panSequenceKey )
    }
}
