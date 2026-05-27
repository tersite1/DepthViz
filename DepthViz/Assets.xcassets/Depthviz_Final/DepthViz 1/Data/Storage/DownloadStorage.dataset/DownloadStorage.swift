import Foundation

/// 서버에서 다운로드받은 PCD 또는 PLY 데이터 저장위치
final class DownloadStorage {
    private init() { }
    
    static let baseURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!.appendingPathComponent("download", isDirectory: true)

    /// 프로젝트와 날짜별로 폴더를 생성하고 파일 URL을 반환합니다.
    static func getURL(projectName: String, date: Date, fileName: String) -> URL? {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let folderURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName, isDirectory: true)
        
        do {
            if !FileManager.default.fileExists(atPath: folderURL.path) {
                try FileManager.default.createDirectory(at: folderURL, withIntermediateDirectories: true, attributes: nil)
            }
        } catch {
            print("Failed to create directory: \(error.localizedDescription)")
            return nil
        }
        
        return folderURL.appendingPathComponent(fileName)
    }
    
    /// 특정 프로젝트와 날짜별 폴더의 파일을 가져옵니다.
    static func getFileURL(projectName: String, date: Date, fileName: String) -> URL? {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let fileURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName).appendingPathComponent(fileName)
        
        if FileManager.default.fileExists(atPath: fileURL.path) {
            return fileURL
        } else {
            return nil
        }
    }
    
    /// 특정 파일 삭제
    static func remove(projectName: String, date: Date, fileName: String) -> Bool {
        do {
            let dateFormatter = DateFormatter()
            dateFormatter.dateFormat = "yyyy-MM-dd"
            let dateFolderName = dateFormatter.string(from: date)
            
            let fileURL = baseURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName).appendingPathComponent(fileName)
            
            try FileManager.default.removeItem(at: fileURL)
            return true
        } catch let error {
            print("---> Failed to remove file: \(error.localizedDescription)")
            return false
        }
    }
}
