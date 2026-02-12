import Foundation
import CoreLocation
import simd

class ScanData: NSObject, ObservableObject, CLLocationManagerDelegate {
    var date: Date
    var fileName: String
    @Published var lidarData: Data
    var fileSize: String
    var points: Int
    var project: String
    @Published var trajectoryPoints: [SIMD3<Float>] = []
    var startCameraTransform: simd_float4x4?
    
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
