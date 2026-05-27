// 프로젝트명들 보는 부분


import SwiftUI

struct ProjectListView: View {
    @Environment(\.presentationMode) var presentationMode
    @EnvironmentObject var listener: ScanInfoRowEventListener
    @State private var projects: [String] = ScanStorage.shared.getProjects()
    @State private var newProjectName: String = ""
    @State private var isAddingProject: Bool = false
    @State private var projectToDelete: String?
    @State private var showDeleteConfirm = false

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
                                projectToDelete = project
                                showDeleteConfirm = true
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
                    Text(NSLocalizedString("add_project", comment: ""))
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                        .padding(.horizontal)
                }
            }
        }
        .alert(NSLocalizedString("delete_project", comment: ""), isPresented: $showDeleteConfirm) {
            Button(NSLocalizedString("delete", comment: ""), role: .destructive) {
                if let project = projectToDelete {
                    deleteProject(project)
                }
                projectToDelete = nil
            }
            Button(NSLocalizedString("cancel", comment: ""), role: .cancel) {
                projectToDelete = nil
            }
        } message: {
            Text(String(format: NSLocalizedString("delete_project_message", comment: ""), projectToDelete ?? ""))
        }
        .onAppear {
            projects = ScanStorage.shared.getProjects()
        }
    }

    private func showNewProjectNameAlert() {
        // SwiftUI의 View에서 UIKit의 AlertController를 사용하기 위해
        if let window = UIApplication.shared.windows.first {
            let rootVC = window.rootViewController

            let nameAlert = UIAlertController(title: NSLocalizedString("new_project_title", comment: ""), message: NSLocalizedString("new_project_message", comment: ""), preferredStyle: .alert)
            nameAlert.addTextField { textField in
                textField.placeholder = NSLocalizedString("new_project_placeholder", comment: "")
            }

            let createAction = UIAlertAction(title: NSLocalizedString("create", comment: ""), style: .default) { [weak rootVC] _ in
                guard let newProjectName = nameAlert.textFields?.first?.text, !newProjectName.isEmpty else {
                    return
                }
                ScanStorage.shared.addProject(newProjectName)
                projects = ScanStorage.shared.getProjects()
            }

            let cancelAction = UIAlertAction(title: NSLocalizedString("cancel", comment: ""), style: .cancel, handler: nil)

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
