// 프로젝트명들 보는 부분


import SwiftUI

struct ProjectListView: View {
    @Environment(\.presentationMode) var presentationMode
    @EnvironmentObject var listener: ScanInfoRowEventListener
    @State private var projects: [String] = ScanStorage.shared.getProjects()
    @State private var newProjectName: String = ""
    @State private var isAddingProject: Bool = false

    var body: some View {
        NavigationView {
            VStack {
                List {
                    ForEach(projects, id: \.self) { project in
                        NavigationLink(destination: ProjectDetailView(project: project)) {
                            Text(project)
                        }
                        .swipeActions(edge: .trailing) {
                            Button(role: .destructive) {
                                deleteProject(project)
                            } label: {
                                Label("삭제", systemImage: "trash")
                            }
                        }
                    }
                }
                .navigationTitle("Projects")
                .navigationBarTitleDisplayMode(.inline)
                .toolbar {
                    ToolbarItem(placement: .navigationBarLeading) {
                        Button(action: {
                            // UIKit의 MainVC로 전환하며 초기화 메서드 호출
                            if let window = UIApplication.shared.windows.first {
                                let mainVC = MainVC()
                                mainVC.OresetScanningState() // 초기화 메서드 호출
                                
                                let navigationController = UINavigationController(rootViewController: mainVC)
                                window.rootViewController = navigationController
                                window.makeKeyAndVisible()
                                mainVC.OresetScanningState()
                            }
                        }) {
                            HStack {
                                Image(systemName: "chevron.left")
                                Text("Back")
                                
                            }
                        }
                    }
                }

                Spacer()

                Button(action: {
                    showNewProjectNameAlert()
                }) {
                    Text("프로젝트 추가")
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                        .padding(.horizontal)
                }
            }
        }
        .onAppear {
            projects = ScanStorage.shared.getProjects()
        }
    }

    private func showNewProjectNameAlert() {
        // SwiftUI의 View에서 UIKit의 AlertController를 사용하기 위해
        if let window = UIApplication.shared.windows.first {
            let rootVC = window.rootViewController

            let nameAlert = UIAlertController(title: "새 프로젝트 이름", message: "새 프로젝트 이름을 입력하세요.", preferredStyle: .alert)
            nameAlert.addTextField { textField in
                textField.placeholder = "프로젝트 이름"
            }

            let createAction = UIAlertAction(title: "생성", style: .default) { [weak rootVC] _ in
                guard let newProjectName = nameAlert.textFields?.first?.text, !newProjectName.isEmpty else {
                    return
                }
                ScanStorage.shared.addProject(newProjectName)
                projects = ScanStorage.shared.getProjects()
            }

            let cancelAction = UIAlertAction(title: "취소", style: .cancel, handler: nil)

            nameAlert.addAction(createAction)
            nameAlert.addAction(cancelAction)

            rootVC?.present(nameAlert, animated: true, completion: nil)
        }
    }

    private func deleteProject(_ project: String) {
        ScanStorage.shared.removeProject(project)
        projects = ScanStorage.shared.getProjects()
    }
}
