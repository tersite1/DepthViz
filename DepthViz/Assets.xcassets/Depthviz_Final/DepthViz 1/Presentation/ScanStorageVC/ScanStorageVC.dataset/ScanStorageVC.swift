import Foundation
import UIKit

class ScanStorageVC: UIViewController {
    
    // MARK: - Properties
    var scanInfos: [ScanInfo] = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Example of loading scan data
        self.loadScanData()
    }
    
    // Example function to load scan data
    private func loadScanData() {
        scanInfos = ScanStorage.shared.infos
    }
    
    // Example function to save scan data
    func saveScanData(fileName: String) {
        let scanInfo = ScanInfo(id: UUID().uuidString, date: Date(), fileName: fileName, fileSize: "0 MB", points: 0)
        let success = ScanStorage.shared.save(scanInfo, projectName: "YourProjectName", date: Date())
        
        if success {
            // Handle successful save
            print("Data saved successfully.")
        } else {
            // Handle save failure
            print("Failed to save data.")
        }
    }
    
    // Example function to get file URL
    func getFileUrl(for info: ScanInfo) -> URL? {
        return ScanStorage.shared.fileUrl(for: info, projectName: "YourProjectName", date: info.date)
    }
}

// MARK: - ScanStorage extension to provide file URL
extension ScanStorage {
    func fileUrl(for info: ScanInfo, projectName: String, date: Date) -> URL? {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd"
        let dateFolderName = dateFormatter.string(from: date)
        
        let folderURL = rootURL.appendingPathComponent(projectName).appendingPathComponent(dateFolderName, isDirectory: true)
        return folderURL.appendingPathComponent(info.fileName)
    }
}
