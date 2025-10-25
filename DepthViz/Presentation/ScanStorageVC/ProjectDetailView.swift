// 프로젝트 내 파일 리스트 보기 - 검색, 정렬, 필터링 기능 추가

import SwiftUI
import UIKit

enum SortOption: String, CaseIterable {
    case dateNewest = "최신순"
    case dateOldest = "오래된순"
    case sizeSmallest = "작은 크기순"
    case sizeLargest = "큰 크기순"
    case nameAsc = "이름 오름차순"
    case nameDesc = "이름 내림차순"
    case pointsHighest = "포인트 많은순"
    case pointsLowest = "포인트 적은순"
}

struct ProjectDetailView: View {
    var project: String
    @State private var scanInfos: [ScanInfo] = []
    @State private var filteredInfos: [ScanInfo] = []
    @State private var searchText: String = ""
    @State private var selectedSortOption: SortOption = .dateNewest
    @State private var showActionSheet = false
    @State private var selectedFile: ScanInfo?
    @State private var showShareSheet = false
    @State private var showRenameAlert = false
    @State private var newFileName = ""
    @State private var showFileView = false
    @State private var showSortPicker = false

    var body: some View {
        VStack(spacing: 0) {
            // 검색 바
            SearchBar(text: $searchText, placeholder: "파일 이름 검색...")
                .padding(.horizontal)
                .padding(.vertical, 8)
            
            // 정렬 버튼 및 결과 표시
            HStack {
                Button(action: {
                    showSortPicker.toggle()
                }) {
                    HStack {
                        Image(systemName: "arrow.up.arrow.down")
                        Text(selectedSortOption.rawValue)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.blue.opacity(0.1))
                    .cornerRadius(8)
                }
                .actionSheet(isPresented: $showSortPicker) {
                    ActionSheet(
                        title: Text("정렬 옵션"),
                        buttons: SortOption.allCases.map { option in
                            .default(Text(option.rawValue)) {
                                selectedSortOption = option
                                applySortAndFilter()
                            }
                        } + [.cancel()]
                    )
                }
                
                Spacer()
                
                Text("\(filteredInfos.count)개 파일")
                    .foregroundColor(.secondary)
                    .font(.subheadline)
            }
            .padding(.horizontal)
            .padding(.vertical, 8)
            
            Divider()
            
            // 파일 리스트
            List {
                ForEach(filteredInfos, id: \.fileName) { info in
                    Button(action: {
                        selectedFile = info
                        showActionSheet = true
                    }) {
                        EnhancedScanInfoRow(info: info)
                    }
                }
                .onDelete(perform: deleteFiles)
            }
        }
        .onAppear {
            loadScanInfos()
        }
        .onChange(of: searchText) { _ in
            applySortAndFilter()
        }
        .actionSheet(isPresented: $showActionSheet) {
            ActionSheet(
                title: Text(selectedFile?.fileName ?? ""),
                buttons: [
                    .default(Text("파일 보기")) {
                        showFileView = true
                    },
                    .default(Text("파일 이름 변경")) {
                        showRenameAlert = true
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
        .sheet(isPresented: $showRenameAlert) {
            RenameFileView(
                currentFileName: selectedFile?.fileName.replacingOccurrences(of: ".ply", with: "") ?? "",
                onRename: { newName in
                    renameFile(file: selectedFile, newFileName: newName + ".ply")
                    showRenameAlert = false
                },
                onCancel: {
                    showRenameAlert = false
                }
            )
        }
        .sheet(isPresented: $showFileView) {
            if let filePath = getFileURL(for: selectedFile)?.path {
                ViewControllerWrapper(filePath: filePath)
            }
        }
        .sheet(isPresented: $showShareSheet) {
            if let fileURL = getFileURL(for: selectedFile) {
                ActivityViewController(activityItems: [fileURL])
            }
        }
        .navigationTitle(project)
        .navigationBarTitleDisplayMode(.inline)
    }

    private func loadScanInfos() {
        // 데이터를 초기화하고 스캔 데이터를 다시 로드
        DispatchQueue.global(qos: .userInitiated).async {
            let loadedInfos = ScanStorage.shared.getFiles(byProject: project)
            DispatchQueue.main.async {
                self.scanInfos = loadedInfos
                self.applySortAndFilter()
                print("Loaded Scan Infos: \(loadedInfos)")
            }
        }
    }
    
    private func applySortAndFilter() {
        var results = scanInfos
        
        // 검색 필터링
        if !searchText.isEmpty {
            results = results.filter { info in
                info.fileName.lowercased().contains(searchText.lowercased())
            }
        }
        
        // 정렬
        switch selectedSortOption {
        case .dateNewest:
            results.sort { $0.date > $1.date }
        case .dateOldest:
            results.sort { $0.date < $1.date }
        case .sizeSmallest:
            results.sort { parseFileSize($0.fileSize) < parseFileSize($1.fileSize) }
        case .sizeLargest:
            results.sort { parseFileSize($0.fileSize) > parseFileSize($1.fileSize) }
        case .nameAsc:
            results.sort { $0.fileName < $1.fileName }
        case .nameDesc:
            results.sort { $0.fileName > $1.fileName }
        case .pointsHighest:
            results.sort { $0.points > $1.points }
        case .pointsLowest:
            results.sort { $0.points < $1.points }
        }
        
        filteredInfos = results
    }
    
    private func parseFileSize(_ sizeString: String) -> Double {
        let components = sizeString.split(separator: " ")
        guard let numberString = components.first,
              let number = Double(numberString) else {
            return 0
        }
        
        if sizeString.contains("MB") {
            return number
        } else if sizeString.contains("KB") {
            return number / 1024.0
        } else if sizeString.contains("GB") {
            return number * 1024.0
        }
        
        return number
    }

    private func renameFile(file: ScanInfo?, newFileName: String) {
        guard let file = file else { return }
        ScanStorage.shared.renameFile(oldFileName: file.fileName, newFileName: newFileName)
        loadScanInfos() // 변경 후 목록 업데이트
    }

    private func shareFile(file: ScanInfo?) {
        guard let fileURL = getFileURL(for: file) else { return }
        // 파일이 존재하는 경우 공유 시트를 표시
        showShareSheet = true
    }

    private func getFileURL(for file: ScanInfo?) -> URL? {
        guard let fileName = file?.fileName else { return nil }
        return ScanStorage.shared.fileUrl(fileName: fileName)
    }

    private func deleteFile(file: ScanInfo?) {
        guard let fileName = file?.fileName else { return }
        ScanStorage.shared.remove(fileName: fileName)
        loadScanInfos() // 데이터 갱신
    }
    
    private func deleteFiles(at offsets: IndexSet) {
        for index in offsets {
            let file = scanInfos[index]
            deleteFile(file: file)
        }
    }
}

// MARK: - Supporting Views

struct SearchBar: View {
    @Binding var text: String
    var placeholder: String
    
    var body: some View {
        HStack {
            Image(systemName: "magnifyingglass")
                .foregroundColor(.gray)
            
            TextField(placeholder, text: $text)
                .textFieldStyle(PlainTextFieldStyle())
            
            if !text.isEmpty {
                Button(action: {
                    text = ""
                }) {
                    Image(systemName: "xmark.circle.fill")
                        .foregroundColor(.gray)
                }
            }
        }
        .padding(8)
        .background(Color(.systemGray6))
        .cornerRadius(10)
    }
}

struct EnhancedScanInfoRow: View {
    let info: ScanInfo
    
    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(info.fileName)
                .font(.headline)
                .foregroundColor(.primary)
            
            HStack {
                Label("\(info.points) pts", systemImage: "point.3.connected.trianglepath.dotted")
                    .font(.caption)
                    .foregroundColor(.secondary)
                
                Spacer()
                
                Text(info.fileSize)
                    .font(.caption)
                    .foregroundColor(.secondary)
                
                Text(formatDate(info.date))
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .padding(.vertical, 4)
    }
    
    private func formatDate(_ date: Date) -> String {
        let formatter = DateFormatter()
        formatter.dateFormat = "MM/dd HH:mm"
        return formatter.string(from: date)
    }
}

struct RenameFileView: View {
    @State private var fileName: String
    let onRename: (String) -> Void
    let onCancel: () -> Void
    
    init(currentFileName: String, onRename: @escaping (String) -> Void, onCancel: @escaping () -> Void) {
        _fileName = State(initialValue: currentFileName)
        self.onRename = onRename
        self.onCancel = onCancel
    }
    
    var body: some View {
        NavigationView {
            VStack(spacing: 20) {
                Text("파일 이름 변경")
                    .font(.title2)
                    .fontWeight(.bold)
                    .padding(.top)
                
                TextField("새 파일 이름", text: $fileName)
                    .textFieldStyle(RoundedBorderTextFieldStyle())
                    .padding(.horizontal)
                
                HStack(spacing: 16) {
                    Button("취소") {
                        onCancel()
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.gray.opacity(0.2))
                    .cornerRadius(10)
                    
                    Button("변경") {
                        if !fileName.isEmpty {
                            onRename(fileName)
                        }
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(10)
                    .disabled(fileName.isEmpty)
                }
                .padding(.horizontal)
                
                Spacer()
            }
            .padding()
        }
    }
}

struct ViewControllerWrapper: UIViewControllerRepresentable {
    var filePath: String
    
    func makeUIViewController(context: Context) -> ViewController {
        let viewController = ViewController()
        return viewController
    }

    func updateUIViewController(_ uiViewController: ViewController, context: Context) {
        uiViewController.READFILE(from: filePath)
    }
}
