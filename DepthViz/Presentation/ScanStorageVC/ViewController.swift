// 포인트클라우드 뷰어


import UIKit
import SceneKit

class ViewController: UIViewController {
    
    var sceneView: SCNView!
    var pointCloud: PointCloud?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        setupSceneView()
        addCloseButton()
    }
    
    func setupSceneView() {
        sceneView = SCNView(frame: self.view.frame)
        sceneView.scene = SCNScene()
        sceneView.backgroundColor = UIColor.black
        sceneView.allowsCameraControl = true // 제스처로 확대/축소 및 회전 가능
        self.view.addSubview(sceneView)
    }
    
    func READFILE(from filePath: String) {
        pointCloud = PointCloud()
        pointCloud?.load(file: filePath)
        
        if let node = pointCloud?.getNode() {
            sceneView.scene?.rootNode.addChildNode(node)
        } else {
            print("Failed to load point cloud")
        }
    }
    
    @objc func closeButtonTapped() {
        self.dismiss(animated: true, completion: nil)
    }
    
    func addCloseButton() {
        let closeButton = UIButton(type: .system)
        closeButton.setTitle("Close", for: .normal)
        closeButton.frame = CGRect(x: 20, y: 40, width: 80, height: 40)
        closeButton.addTarget(self, action: #selector(closeButtonTapped), for: .touchUpInside)
        self.view.addSubview(closeButton)
    }
}
