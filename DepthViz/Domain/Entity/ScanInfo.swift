import Foundation

struct ScanInfo: Codable, Identifiable {
    let id: UUID
    var fileName: String // 'let'에서 'var'로 변경
    var project: String
    var date: Date
    var fileSize: String
    var points: Int

    init(fileName: String, project: String, date: Date, fileSize: String, points: Int) {
        self.id = UUID()
        self.fileName = fileName
        self.project = project
        self.date = date
        self.fileSize = fileSize
        self.points = points
    }
}
