import UIKit

class FrameDetailVC: UIViewController {
    var metadata: AccumulatedFrameData.Metadata?
    var image: UIImage?
    
    private let imageView = UIImageView()
    private let textView = UITextView()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        imageView.contentMode = .scaleAspectFit
        imageView.translatesAutoresizingMaskIntoConstraints = false
        textView.translatesAutoresizingMaskIntoConstraints = false
        textView.isEditable = false
        view.addSubview(imageView)
        view.addSubview(textView)
        NSLayoutConstraint.activate([
            imageView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 10),
            imageView.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            imageView.widthAnchor.constraint(equalToConstant: 240),
            imageView.heightAnchor.constraint(equalToConstant: 180),
            textView.topAnchor.constraint(equalTo: imageView.bottomAnchor, constant: 10),
            textView.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 16),
            textView.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -16),
            textView.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -16)
        ])
        imageView.image = image
        if let m = metadata {
            textView.text = """
Frame Index: \(m.index)
Timestamp: \(m.timestamp)
Image File: \(m.imageFileName)
Camera Pose: \(m.cameraPose)
Camera Intrinsics: \(m.cameraIntrinsics)
View Matrix: \(m.viewMatrix)
Projection Matrix: \(m.projectionMatrix)
"""
        }
    }
}
