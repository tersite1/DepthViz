/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Main view controller for the AR experience.
*/

import UIKit
import Metal
import MetalKit
import ARKit

final class MainVC: UIViewController, ARSessionDelegate {
    // MARK: View
    private let confidenceControl = UISegmentedControl(items: ["Low", "Medium", "High"])
    private let rgbRadiusSlider = UISlider()
    private let session = ARSession()
    private var renderer: Renderer!
    private let recordButton = UIButton()
    // MARK: Property
    private var isRecording = false
    
    /// MainVC 최초 접근시 configure
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureMetalKitView()
        self.configureRendererDelegegate()
        self.configureUI()
    }
    
    /// MainVC 화면 진입시 configure
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.configureARWorldTracking()
    }
}

// MARK: Configure
extension MainVC {
    /// MetalKitView 설정 및 view 설정
    private func configureMetalKitView() {
        guard let device = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported on this device")
            return
        }
        
        self.session.delegate = self
        
        // Set the view to use the default device
        if let view = view as? MTKView {
            view.device = device
            
            view.backgroundColor = UIColor.clear
            // we need this to enable depth test
            view.depthStencilPixelFormat = .depth32Float
            view.contentScaleFactor = 1
            view.delegate = self
            
            // Configure the renderer to draw to the view
            self.renderer = Renderer(session: self.session, metalDevice: device, renderDestination: view)
            self.renderer.drawRectResized(size: view.bounds.size)
        }
    }
    
    /// Renderer -> MainVC 연결
    private func configureRendererDelegegate() {
        self.renderer.delegate = self
    }
    
    /// MainVC 표시할 UI 설정
    private func configureUI() {
        // Confidence control
        confidenceControl.backgroundColor = .clear
        confidenceControl.tintColor = .label
        confidenceControl.selectedSegmentIndex = renderer.confidenceThreshold
        confidenceControl.addTarget(self, action: #selector(viewValueChanged), for: .valueChanged)
        
        // RGB Radius control
        rgbRadiusSlider.minimumValue = 0
        rgbRadiusSlider.maximumValue = 1.5
        rgbRadiusSlider.isContinuous = true
        rgbRadiusSlider.value = renderer.rgbRadius
        rgbRadiusSlider.addTarget(self, action: #selector(viewValueChanged), for: .valueChanged)

        // UIButton
        recordButton.setTitle("START", for: .normal)
        recordButton.backgroundColor = .systemBlue
        recordButton.layer.cornerRadius = 5
        recordButton.addTarget(self, action: #selector(onButtonClick), for: .touchUpInside)
        
        let stackView = UIStackView(arrangedSubviews: [
            confidenceControl, rgbRadiusSlider, recordButton])
        stackView.isHidden = false
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .vertical
        stackView.spacing = 10
        view.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            stackView.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -16)
        ])
    }
    
    /// session 설정 및 화면꺼짐방지
    private func configureARWorldTracking() {
        // Create a world-tracking configuration, and
        // enable the scene depth frame-semantic.
        let configuration = ARWorldTrackingConfiguration()
        // MARK: SAVE PLY
        configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]

        // Run the view's session
        self.session.run(configuration)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
}

// MARK: Action
extension MainVC {
    /// UI
    @objc private func viewValueChanged(view: UIView) {
        switch view {
            
        case confidenceControl:
            self.renderer.confidenceThreshold = self.confidenceControl.selectedSegmentIndex
            
        case rgbRadiusSlider:
            self.renderer.rgbRadius = self.rgbRadiusSlider.value
            
        default:
            return
        }
    }
}

// MARK: HomeBar & StatusBar Hidden
extension MainVC {
    // Auto-hide the home indicator to maximize immersion in AR experiences.
    override var prefersHomeIndicatorAutoHidden: Bool {
        return true
    }
    
    // Hide the status bar to maximize immersion in AR experiences.
    override var prefersStatusBarHidden: Bool {
        return true
    }
}

// MARK: MTKViewDelegate
extension MainVC: MTKViewDelegate {
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        renderer.drawRectResized(size: size)
    }
    
    // Called whenever the view needs to render
    func draw(in view: MTKView) {
        renderer.draw()
    }
}

// MARK: RenderDestinationProvider
protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var depthStencilPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

extension MTKView: RenderDestinationProvider {
    
}

// MARK: SAVE PLY
extension MainVC {
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        
        print("memory warning!!!")
        memoryAlert()
        updateIsRecording(_isRecording: false)
    }
    
    private func memoryAlert() {
        let alert = UIAlertController(title: "Low Memory Warning", message: "The recording has been paused. Do not quit the app until all files have been saved.", preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: NSLocalizedString("OK", comment: "Default action"), style: .default, handler: { _ in
            NSLog("The \"OK\" alert occured.")
        }))
        self.present(alert, animated: true, completion: nil)
    }
    
    @objc
    private func onButtonClick(_ sender: UIButton) {
        if (sender != recordButton) {
            return
        }
        updateIsRecording(_isRecording: !isRecording)
    }
    
    private func updateIsRecording(_isRecording: Bool) {
        isRecording = _isRecording
        renderer.isRecording = isRecording
        if (isRecording){
            recordButton.setTitle("PAUSE", for: .normal)
            recordButton.backgroundColor = .systemRed
        } else {
            recordButton.setTitle("START", for: .normal)
            recordButton.backgroundColor = .systemBlue
            renderer.savePointCloud()
        }
    }
}

// update textlabel on tasks start/finish
extension MainVC: TaskDelegate {
    func showUploadResult(result: NetworkResult) {
        switch result.status {
        case .SUCCESS:
            self.showAlert(title: "Upload Success", text: "You can see the record historys in the SCANS page")
        case .ERROR:
            self.showAlert(title: "Upload Fail", text: "\(result.status.rawValue)")
        }
    }
    
    func sharePLY(file: Any) {
        let activityViewController = UIActivityViewController(activityItems: [file], applicationActivities: nil)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            activityViewController.popoverPresentationController?.sourceView = self.view
            activityViewController.popoverPresentationController?.sourceRect = self.recordButton.frame
        }
        
        DispatchQueue.main.async {
            self.present(activityViewController, animated: true)
        }
    }
    
    func startMakingPlyFile() {
        print("start making ply file")
    }
    
    func finishMakingPlyFile() {
        print("finish making ply file")
    }
    
    func startUploadingData() {
        print("start uploading ply file")
    }
}
