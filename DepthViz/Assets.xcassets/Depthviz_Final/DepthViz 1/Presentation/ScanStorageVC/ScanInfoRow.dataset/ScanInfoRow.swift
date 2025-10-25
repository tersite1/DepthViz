//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//

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
    static let info = ScanInfo(id: "2023-07-14-11-22-33.ply", date: Date(), fileName: "2023-07-14-11-22-33.ply", fileSize: "12.3 MB", points: 12345)
    
    static var previews: some View {
        ScanInfoRow(info: info)
    }
}
