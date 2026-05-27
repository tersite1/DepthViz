import Foundation

/// 서버전송 오류로 인한 LiDAR 스캔파일 저장소
final class LidarStorage {
    private init() { }
    
    static let baseURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("lidarData", isDirectory: true)

    /// 데이터를 특정 프로젝트 및 날짜별로 저장
    static func save(_ obj: LiDARData, projectName: String, date: Date) -> Bool {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let folderURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName, isDirectory: true)
        
        do {
            // 폴더가 없다면 생성
            if !FileManager.default.fileExists(atPath: folderURL.path) {
                try FileManager.default.createDirectory(at: folderURL, withIntermediateDirectories: true, attributes: nil)
            }
            
            let fileURL = folderURL.appendingPathComponent("LiDARData.json")
            let encoder = JSONEncoder()
            encoder.outputFormatting = .prettyPrinted
            encoder.dateEncodingStrategy = .iso8601
            
            let data = try encoder.encode(obj)
            if FileManager.default.fileExists(atPath: fileURL.path) {
                try FileManager.default.removeItem(at: fileURL)
            }
            
            FileManager.default.createFile(atPath: fileURL.path, contents: data, attributes: nil)
            return true
        } catch let error {
            print("---> Failed to store msg: \(error.localizedDescription)")
            return false
        }
    }
    
    static func get(projectName: String, date: Date) -> LiDARData? {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let fileURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName).appendingPathComponent("LiDARData.json")
        
        guard FileManager.default.fileExists(atPath: fileURL.path) else { return nil }
        guard let data = FileManager.default.contents(atPath: fileURL.path) else { return nil }
        
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        
        do {
            let lidarData = try decoder.decode(LiDARData.self, from: data)
            return lidarData
        } catch let error {
            print("---> Failed to decode msg: \(error.localizedDescription)")
            return nil
        }
    }
    
    static func remove(projectName: String, date: Date) {
        do {
            let dateFormatter = DateFormatter()
            dateFormatter.dateFormat = "yyyy-MM-dd"
            let dateFolderName = dateFormatter.string(from: date)
            
            let fileURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName).appendingPathComponent("LiDARData.json")
            try FileManager.default.removeItem(at: fileURL)
        } catch let error {
            print("---> Failed to remove msg: \(error.localizedDescription)")
        }
    }
}
