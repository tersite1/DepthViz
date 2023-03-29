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
    private let statusLabel = StatusIndicatorLabel()
    private let recordingButton = RecordingButton()
    
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
        // recordingButton
        self.recordingButton.addTarget(self, action: #selector(tapRecordingButton), for: .touchUpInside)
        self.view.addSubview(self.recordingButton)
        NSLayoutConstraint.activate([
            self.recordingButton.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.recordingButton.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -12)
        ])
        
        // statusLabel
        self.view.addSubview(self.statusLabel)
        NSLayoutConstraint.activate([
            self.statusLabel.centerXAnchor.constraint(equalTo: self.view.centerXAnchor),
            self.statusLabel.topAnchor.constraint(equalTo: self.recordingButton.topAnchor, constant: -60)
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
    /// RecordingButton Tab 액션
    @objc private func tapRecordingButton(_ sender: UIButton) {
        if self.recordingButton.status == .ready {
            self.recordingButton.changeStatus(to: .recording)
        } else if self.recordingButton.status == .recording {
            self.recordingButton.changeStatus(to: .loading)
        } else { return }
        self.updateRecording()
    }
    
    private func updateRecording() {
        let recording: Bool = self.recordingButton.status == .recording
        self.renderer.isRecording = recording
        
        if recording {
            self.statusLabel.changeText(to: .recording)
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
        self.recordingButton.changeStatus(to: .ready)
        self.updateRecording()
    }
    
    private func memoryAlert() {
        let alert = UIAlertController(title: "Low Memory Warning", message: "The recording has been paused. Do not quit the app until all files have been saved.", preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: NSLocalizedString("OK", comment: "Default action"), style: .default, handler: { _ in
            NSLog("The \"OK\" alert occured.")
        }))
        self.present(alert, animated: true, completion: nil)
    }
}

// update textlabel on tasks start/finish
extension MainVC: TaskDelegate {
    func showUploadResult(result: NetworkResult) {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .removed)
            self?.recordingButton.changeStatus(to: .ready)
            switch result.status {
            case .SUCCESS:
                self?.showAlert(title: "Upload Success", text: "You can see the record historys in the SCANS page")
            case .ERROR:
                self?.showAlert(title: "Upload Fail", text: "\(result.status.rawValue)")
            }
            
        }
    }
    
    func sharePLY(file: Any) {
        let activityViewController = UIActivityViewController(activityItems: [file], applicationActivities: nil)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            activityViewController.popoverPresentationController?.sourceView = self.view
            activityViewController.popoverPresentationController?.sourceRect = self.recordingButton.frame
        }
        
        DispatchQueue.main.async {
            self.present(activityViewController, animated: true)
        }
    }
    
    func startMakingPlyFile() {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .loading)
        }
    }
    
    func finishMakingPlyFile() {
        print("finish making ply file")
    }
    
    func startUploadingData() {
        DispatchQueue.main.async { [weak self] in
            self?.statusLabel.changeText(to: .uploading)
        }
    }
}
