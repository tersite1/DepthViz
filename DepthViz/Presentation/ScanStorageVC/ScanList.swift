//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import SwiftUI

struct ScanList: View {
    @EnvironmentObject var listener: ScanInfoRowEventListener
    @ObservedObject var scanStorage = ScanStorage.shared
    
    var body: some View {
        List {
            ForEach(scanStorage.infos) { info in
                ScanInfoRow(info: info)
                    .onTapGesture {
                        listener.selectedLidarFileName = info.fileName
                    }
            }
        }
    }
}

struct ScanList_Previews: PreviewProvider {
    static var previews: some View {
        ScanList()
    }
}
