import SwiftUI

struct ScanInfoRow: View {
    var info: ScanInfo
    
    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                Text(info.fileName)
                    .bold()
                Text("\(info.points.decimalString) Points | \(info.fileSize)")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            
            Spacer()
            
            Image(systemName: "chevron.right")
        }
        .contentShape(Rectangle())
    }
}

struct ScanInfoRow_Previews: PreviewProvider {
    static let info = ScanInfo(
        fileName: "2024-07-14-11-22-33.ply",
        project: "Default Project",
        date: Date(),
        fileSize: "12.3 MB",
        points: 12345
    )
    
    static var previews: some View {
        ScanInfoRow(info: info)
    }
}
