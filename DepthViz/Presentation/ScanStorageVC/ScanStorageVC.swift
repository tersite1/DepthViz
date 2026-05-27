import UIKit
import SwiftUI
import Combine

final class ScanStorageVC: UIViewController {
    private let listener = ScanInfoRowEventListener()
    private var cancellables: [AnyCancellable] = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.bind()
    }

    private func configureUI() {
        self.title = "Projects"
        self.view.backgroundColor = .systemBackground
        
        let hostingVC = UIHostingController(rootView: ProjectListView().environmentObject(self.listener))
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

    private func bind() {
        // 프로젝트를 선택했을 때 해당 프로젝트의 스캔 데이터를 표시
        listener.$selectedProject
            .sink { [weak self] (project: String?) in
                guard let project = project else { return }
                print("Selected Project: \(project)") // 디버그 로그 추가
                self?.moveToProjectDetails(project: project)
            }
            .store(in: &self.cancellables)
    }

    private func moveToProjectDetails(project: String) {
        let projectDetailVC = ProjectDetailVC(project: project)
        self.navigationController?.pushViewController(projectDetailVC, animated: true)
    }
}
