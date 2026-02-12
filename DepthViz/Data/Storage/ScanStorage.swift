import Foundation
import simd

/// 스캔된 Point Cloud Data 저장소
final class ScanStorage: ObservableObject {
    static let shared = ScanStorage()

    let infosRoot = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("scanInfos", isDirectory: true)
    let filesRoot = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("scanFiles", isDirectory: true)

    @Published var infos: [ScanInfo] = [] {
        didSet {
            print("Infos count updated: \(infos.count)")
        }
    }
    
    // 프로젝트 관리 변수
    @Published var projects: [String] = [] {
        didSet {
            print("Projects updated: \(projects)")
        }
    }

    
    func createDirectoryIfNeeded() {
        let fileManager = FileManager.default
        
        if !fileManager.fileExists(atPath: infosRoot.path) {
            do {
                try fileManager.createDirectory(at: infosRoot, withIntermediateDirectories: true, attributes: nil)
                print("Created directory at: \(infosRoot.path)")
            } catch {
                print("Failed to create directory at: \(infosRoot.path), error: \(error.localizedDescription)")
            }
        } else {
            print("Directory already exists at: \(infosRoot.path)")
        }

        if !fileManager.fileExists(atPath: filesRoot.path) {
            do {
                try fileManager.createDirectory(at: filesRoot, withIntermediateDirectories: true, attributes: nil)
                print("Created directory at: \(filesRoot.path)")
            } catch {
                print("Failed to create directory at: \(filesRoot.path), error: \(error.localizedDescription)")
            }
        } else {
            print("Directory already exists at: \(filesRoot.path)")
        }
    }

    
    
    private init() {
        self.updateInfos()
        self.loadProjects()
    }

    private func updateInfos() {
        do {
            if FileManager.default.fileExists(atPath: self.infosRoot.path) == false {
                print("Creating directory at \(self.infosRoot.path)")
                self.createDirectory()
            }

            let urls = try FileManager.default.contentsOfDirectory(at: infosRoot, includingPropertiesForKeys: nil)
            print("Found files at \(infosRoot): \(urls)")

            var infos: [ScanInfo] = []

            for url in urls {
                if let info = getInfo(url: url) {
                    print("Loaded info from \(url): \(info)")
                    infos.append(info)
                } else {
                    print("Failed to load info from \(url)")
                }
            }
            self.infos = infos
            print("Infos loaded: \(infos)")
        } catch {
            print("Failed to update infos: \(error.localizedDescription)")
        }
    }


    private func loadProjects() {
        // 기존에 저장된 프로젝트 목록을 불러옴
        if let savedProjectsData = UserDefaults.standard.data(forKey: "projects") {
            if let savedProjects = try? JSONDecoder().decode([String].self, from: savedProjectsData) {
                self.projects = savedProjects
            }
        }
    }

    private func saveProjects() {
        // 현재 프로젝트 목록을 UserDefaults에 저장
        if let encodedProjects = try? JSONEncoder().encode(self.projects) {
            UserDefaults.standard.set(encodedProjects, forKey: "projects")
        }
    }

    func addProject(_ name: String) {
        guard !projects.contains(name) else { return }
        projects.append(name)
        saveProjects()
    }

    func getProjects() -> [String] {
        return projects
    }
    

    func createDirectory() {
        do {
            try FileManager.default.createDirectory(at: infosRoot, withIntermediateDirectories: true)
            try FileManager.default.createDirectory(at: filesRoot, withIntermediateDirectories: true)
        } catch {
            print("Failed to create directory: \(error.localizedDescription)")
        }
    }

    func getFiles(byProject project: String) -> [ScanInfo] {
        let filteredFiles = infos.filter { $0.project == project }
        print("Filtered Files for \(project): \(filteredFiles)")
        return filteredFiles
    }

    func getFiles(byDate date: Date) -> [ScanInfo] {
        let calendar = Calendar.current
        return infos.filter { calendar.isDate($0.date, inSameDayAs: date) }
    }

    func moveFile(fileName: String, toProject newProject: String) {
        guard let index = infos.firstIndex(where: { $0.fileName == fileName }) else { return }
        infos[index].project = newProject
        saveUpdatedInfo(infos[index])
    }

    private func saveUpdatedInfo(_ updatedInfo: ScanInfo) {
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        encoder.dateEncodingStrategy = .iso8601

        let infoUrl = infosRoot.appendingPathComponent(updatedInfo.fileName)

        do {
            let infoData = try encoder.encode(updatedInfo)
            if FileManager.default.fileExists(atPath: infoUrl.path) {
                try FileManager.default.removeItem(at: infoUrl)
            }
            FileManager.default.createFile(atPath: infoUrl.path, contents: infoData)
            updateInfos()
        } catch {
            print("Failed to update info: \(error.localizedDescription)")
        }
    }

    func hasEnoughStorage(for dataSize: Int) -> Bool {
        let urls = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)
        guard let url = urls.first else { return false }
        do {
            let values = try url.resourceValues(forKeys: [.volumeAvailableCapacityForImportantUsageKey])
            let available = values.volumeAvailableCapacityForImportantUsage ?? 0
            return available > Int64(dataSize + 1_000_000)  // 1MB margin
        } catch {
            return true  // Allow save if we can't check
        }
    }

    func save(_ ply: ScanData) -> Bool {
        guard hasEnoughStorage(for: ply.lidarData.count) else {
            print("❌ 저장 공간 부족: \(ply.lidarData.count) bytes 필요")
            return false
        }

        let infoUrl = infosRoot.appendingPathComponent(ply.fileName)
        let fileUrl = filesRoot.appendingPathComponent(ply.fileName)
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        encoder.dateEncodingStrategy = .iso8601

        do {
            if FileManager.default.fileExists(atPath: infosRoot.path) == false {
                createDirectory()
            }

            let infoData = try encoder.encode(ply.info)
            if FileManager.default.fileExists(atPath: infoUrl.path) {
                try FileManager.default.removeItem(at: infoUrl)
            }

            FileManager.default.createFile(atPath: infoUrl.path, contents: infoData, attributes: nil)
            if FileManager.default.fileExists(atPath: fileUrl.path) {
                try FileManager.default.removeItem(at: fileUrl)
            }
            FileManager.default.createFile(atPath: fileUrl.path, contents: ply.lidarData)
            
            // 파일 크기 및 포인트 개수 디버그 로그 추가
            print("Stored LiDAR Data at \(fileUrl.path) with size: \(ply.fileSize) and points: \(ply.points)")

            self.updateInfos()
            return true
        } catch let error {
            print("Failed to store data: \(error.localizedDescription)")
            return false
        }
    }


    func getInfo(url: URL) -> ScanInfo? {
        guard FileManager.default.fileExists(atPath: url.path) else {
            print("File does not exist at path: \(url.path)")
            return nil
        }
        guard let data = FileManager.default.contents(atPath: url.path) else {
            print("Failed to load data at path: \(url.path)")
            return nil
        }

        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601

        do {
            let scanInfo = try decoder.decode(ScanInfo.self, from: data)
            return scanInfo
        } catch let error {
            print("Failed to decode info at path \(url.path): \(error.localizedDescription)")
            return nil
        }
    }

    
    func removeProject(_ name: String) {
        projects.removeAll { $0 == name }
        saveProjects()

        // 해당 프로젝트에 포함된 파일들도 삭제
        let filesToRemove = infos.filter { $0.project == name }
        for file in filesToRemove {
            remove(fileName: file.fileName)
        }
    }

    func remove(fileName: String) {
        do {
            let infoUrl = infosRoot.appendingPathComponent(fileName)
            try FileManager.default.removeItem(at: infoUrl)
            let fileUrl = filesRoot.appendingPathComponent(fileName)
            try FileManager.default.removeItem(at: fileUrl)

            self.updateInfos()
        } catch let error {
            print("Failed to remove file: \(error.localizedDescription)")
        }
    }

    func fileUrl(fileName: String) -> URL {
        return filesRoot.appendingPathComponent(fileName)
    }

    // 파일 이름 변경 기능 추가
    func renameFile(oldFileName: String, newFileName: String) {
        guard let index = infos.firstIndex(where: { $0.fileName == oldFileName }) else { return }

        let oldInfoUrl = infosRoot.appendingPathComponent(oldFileName)
        let oldFileUrl = filesRoot.appendingPathComponent(oldFileName)

        let newInfoUrl = infosRoot.appendingPathComponent(newFileName)
        let newFileUrl = filesRoot.appendingPathComponent(newFileName)

        // 파일 이름 변경
        do {
            // ScanInfo 업데이트
            infos[index].fileName = newFileName
            saveUpdatedInfo(infos[index])

            // 실제 파일 시스템에서 파일 이름 변경
            if FileManager.default.fileExists(atPath: oldInfoUrl.path) {
                try FileManager.default.moveItem(at: oldInfoUrl, to: newInfoUrl)
            }
            if FileManager.default.fileExists(atPath: oldFileUrl.path) {
                try FileManager.default.moveItem(at: oldFileUrl, to: newFileUrl)
            }
        } catch {
            print("Failed to rename file: \(error.localizedDescription)")
        }

        updateInfos()
    }

    /// Decode a simd_float4x4 transform from a JSON file.
    static func decodeTransform(from url: URL) -> simd_float4x4? {
        guard FileManager.default.fileExists(atPath: url.path),
              let data = try? Data(contentsOf: url),
              let columns = try? JSONDecoder().decode([[Float]].self, from: data),
              columns.count == 4 else {
            return nil
        }
        return simd_float4x4(
            SIMD4<Float>(columns[0][0], columns[0][1], columns[0][2], columns[0][3]),
            SIMD4<Float>(columns[1][0], columns[1][1], columns[1][2], columns[1][3]),
            SIMD4<Float>(columns[2][0], columns[2][1], columns[2][2], columns[2][3]),
            SIMD4<Float>(columns[3][0], columns[3][1], columns[3][2], columns[3][3])
        )
    }
}
