import UIKit
import SwiftUI

final class ProjectDetailVC: UIViewController {
    private var project: String

    init(project: String) {
        self.project = project
        super.init(nibName: nil, bundle: nil)
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
    }
    
    private func configureUI() {
        self.title = project
        self.view.backgroundColor = .systemBackground
        
        let hostingVC = UIHostingController(rootView: ProjectDetailView(project: project))
        self.addChild(hostingVC)
        hostingVC.didMove(toParent: self)
        
        hostingVC.view.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(hostingVC.view)
        
        NSLayoutConstraint.activate([
            hostingVC.view.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor),
            hostingVC.view.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor),
            hostingVC.view.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor),
            hostingVC.view.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor)
        ])
    }
}
