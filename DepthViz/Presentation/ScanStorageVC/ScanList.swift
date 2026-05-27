import SwiftUI

struct ScanList: View {
    @EnvironmentObject var listener: ScanInfoRowEventListener
    @State private var selectedProject: String?
    @State private var newProjectName: String = "" // 새로운 프로젝트 이름을 입력할 상태
    @State private var projects: [String] = ScanStorage.shared.getProjects()
    @State private var isLoading: Bool = true // 로딩 상태 변수
    @State private var showActionSheet: Bool = false // 액션 시트를 표시하기 위한 상태 변수
    @State private var selectedFile: ScanInfo? // 선택된 파일 정보를 저장

    var body: some View {
        NavigationView {
            VStack {
                // 프로젝트 추가 UI
                HStack {
                    TextField("Enter new project name", text: $newProjectName)
                        .textFieldStyle(RoundedBorderTextFieldStyle())
                    Button(action: {
                        addProject()
                    }) {
                        Text("Add Project")
                    }
                    .disabled(newProjectName.isEmpty) // 입력된 이름이 없으면 버튼 비활성화
                }
                .padding()

                if isLoading {
                    Text("Loading...") // 로딩 중일 때 표시
                        .font(.headline)
                } else {
                    List {
                        // 프로젝트별 섹션
                        ForEach(projects, id: \.self) { project in
                            Section(header: Text(project)) {
                                ForEach(getFiles(for: project), id: \.fileName) { file in
                                    ScanInfoRow(info: file)
                                        .onTapGesture {
                                            selectedFile = file
                                            showActionSheet = true
                                        }
                                }
                            }
                        }
                    }
                    .navigationTitle("Scan List")
                }
            }
        }
        .onAppear {
            loadData()
        }
        .actionSheet(isPresented: $showActionSheet) {
            ActionSheet(
                title: Text(selectedFile?.fileName ?? ""),
                buttons: [
                    .default(Text("파일 보기")) {
                        viewFile(file: selectedFile)
                    },
                    .default(Text("파일 공유")) {
                        shareFile(file: selectedFile)
                    },
                    .destructive(Text("파일 삭제")) {
                        deleteFile(file: selectedFile)
                    },
                    .cancel()
                ]
            )
        }
    }

    private func loadData() {
        isLoading = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { // 간단한 비동기 처리 예제
            projects = ScanStorage.shared.getProjects()
            isLoading = false // 데이터 로딩이 완료되면 로딩 상태를 false로 설정
        }
    }

    private func addProject() {
        guard !newProjectName.isEmpty else { return }
        ScanStorage.shared.addProject(newProjectName)
        loadData() // 프로젝트 목록 갱신
        newProjectName = "" // 입력 필드 초기화
    }

    private func getFiles(for project: String) -> [ScanInfo] {
        let files = ScanStorage.shared.getFiles(byProject: project)
        print("Loaded files for project \(project): \(files)") // 디버깅용 출력
        return files
    }

    private func viewFile(file: ScanInfo?) {
        guard let fileName = file?.fileName else { return }
        // 파일 보기 로직을 여기에 추가
        print("Viewing file: \(fileName)")
    }

    private func shareFile(file: ScanInfo?) {
        guard let fileName = file?.fileName else { return }
        // 파일 공유 로직을 여기에 추가
        print("Sharing file: \(fileName)")
    }

    private func deleteFile(file: ScanInfo?) {
        guard let fileName = file?.fileName else { return }
        // 파일 삭제 로직을 여기에 추가
        ScanStorage.shared.remove(fileName: fileName)
        loadData() // 삭제 후 데이터 갱신
    }
}
