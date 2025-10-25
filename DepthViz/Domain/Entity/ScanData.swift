import Foundation
import CoreLocation

class ScanData: NSObject, ObservableObject, CLLocationManagerDelegate {
    var date: Date
    var fileName: String
    var lidarData: Data
    var fileSize: String
    var points: Int
    var project: String
    
    var info: ScanInfo {
        return ScanInfo(
            fileName: fileName,
            project: project,
            date: date,
            fileSize: fileSize,
            points: points
        )
    }

    private let locationManager = CLLocationManager()
    private var currentLocation: CLLocation?
    private var completion: ((String) -> Void)?

    init(date: Date, fileName: String, lidarData: Data, fileSize: String, points: Int, project: String) {
        self.date = date
        self.fileName = fileName
        self.lidarData = lidarData
        self.fileSize = fileSize
        self.points = points
        self.project = project
        super.init()
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.requestWhenInUseAuthorization()

        // 디버그 로그 추가
        print("Initializing ScanData with fileName: \(fileName), project: \(project), fileSize: \(fileSize), initial points: \(points)")
        
        // 포인트 개수를 계산하여 저장
        self.points = self.calculatePointCount(from: lidarData)
        print("Calculated Points: \(self.points)") // 디버그 로그 추가
    }

    private func calculatePointCount(from data: Data) -> Int {
        guard let content = String(data: data, encoding: .utf8) else {
            print("LiDAR 데이터를 문자열로 변환하는데 실패했습니다.")
            return 0
        }
        
        print("LiDAR Data Content:\n\(content)") // 데이터 내용 출력

        var points = 0
        let lines = content.split(separator: "\n")
        var parsingVertices = false
        
        for line in lines {
            if line.hasPrefix("end_header") {
                parsingVertices = true
                continue
            }
            
            if parsingVertices {
                let components = line.split(separator: " ")
                if components.count >= 3 {
                    points += 1
                    print("Parsing Point: \(components)") // 파싱된 포인트 출력
                }
            }
        }
        
        print("Total Parsed Points: \(points)") // 디버그 로그 추가
        return points
    }

    func rename(to newFileName: String) {
        self.fileName = newFileName
    }

    func generateFileName(completion: @escaping (String) -> Void) {
        self.completion = completion
        locationManager.requestLocation()
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.first {
            currentLocation = location
            createFileName()
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("위치 정보를 가져오는데 실패했습니다: \(error.localizedDescription)")
        createFileName()
    }

    private func createFileName() {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let currentTime = dateFormatter.string(from: Date())

        var locationString = "UnknownLocation"
        if let location = currentLocation {
            let latitude = location.coordinate.latitude
            let longitude = location.coordinate.longitude
            locationString = "\(latitude)_\(longitude)"
        }

        let fileName = "\(locationString)-\(currentTime)"
        completion?(fileName)
    }
}
