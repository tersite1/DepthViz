import Foundation

/// 스캔된 Point Cloud Data 저장소
final class ScanStorage: ObservableObject {
    static let shared = ScanStorage()
    
    let rootURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("scanData", isDirectory: true)
    
    @Published var infos: [ScanInfo] = [] {
        didSet {
            print(infos.count)
        }
    }
    
    private init() {
        self.updateInfos()
    }
    
    private func updateInfos() {
        do {
            if !FileManager.default.fileExists(atPath: rootURL.path) {
                try FileManager.default.createDirectory(at: rootURL, withIntermediateDirectories: true)
            }
            
            let projectFolders = try FileManager.default.contentsOfDirectory(at: rootURL, includingPropertiesForKeys: nil, options: .skipsHiddenFiles)
            var allInfos: [ScanInfo] = []
            
            for projectFolder in projectFolders {
                let scanFiles = try FileManager.default.contentsOfDirectory(at: projectFolder, includingPropertiesForKeys: nil, options: .skipsHiddenFiles)
                
                for fileURL in scanFiles {
                    let data = try Data(contentsOf: fileURL)
                    let info = try JSONDecoder().decode(ScanInfo.self, from: data)
                    allInfos.append(info)
                }
            }
            
            self.infos = allInfos
        } catch let error {
            print("---> Failed to load scan data: \(error.localizedDescription)")
        }
    }
    
    func save(_ info: ScanInfo, projectName: String, date: Date) -> Bool {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let folderURL = rootURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName, isDirectory: true)
        
        do {
            if !FileManager.default.fileExists(atPath: folderURL.path) {
                try FileManager.default.createDirectory(at: folderURL, withIntermediateDirectories: true)
            }
            
            let fileURL = folderURL.appendingPathComponent(info.id + ".json")  // 수정된 부분
            let data = try JSONEncoder().encode(info)
            FileManager.default.createFile(atPath: fileURL.path, contents: data, attributes: nil)
            updateInfos() // 데이터를 저장한 후 업데이트
            return true
        } catch let error {
            print("---> Failed to save scan info: \(error.localizedDescription)")
            return false
        }
    }
    
    func remove(_ info: ScanInfo, projectName: String, date: Date) {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let fileURL = rootURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName).appendingPathComponent(info.id + ".json")  // 수정된 부분
        
        do {
            try FileManager.default.removeItem(at: fileURL)
            updateInfos() // 데이터를 삭제한 후 업데이트
        } catch let error {
            print("---> Failed to remove scan info: \(error.localizedDescription)")
        }
    }
}
