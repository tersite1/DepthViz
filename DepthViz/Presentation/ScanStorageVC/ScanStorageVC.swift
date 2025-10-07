//
//  Utils.swift
//  DepthViz
//
//  Created by Group 9 on 2024/06/15.
//  Copyright © 2024 Apple. All rights reserved.
//


import UIKit
import SwiftUI
import Combine
import ZIPFoundation

final class ScanStorageVC: UIViewController {
    private let listener = ScanInfoRowEventListener()
    private var cancellables: [AnyCancellable] = []
    
    init() {
        super.init(nibName: nil, bundle: nil)
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.configureUI()
        self.bind()
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.navigationController?.setNavigationBarHidden(false, animated: true)
        AppDelegate.shared.shouldSupportAllOrientation = true
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        AppDelegate.shared.shouldSupportAllOrientation = false
    }
}

extension ScanStorageVC {
    private func configureUI() {
        self.title = "ScanList"
        self.view.backgroundColor = .systemBackground
        
        let hostingVC = UIHostingController(rootView: ScanList().environmentObject(self.listener))
        self.addChild(hostingVC)
        hostingVC.didMove(toParent: self)
        
        hostingVC.view.translatesAutoresizingMaskIntoConstraints = false
        self.view.addSubview(hostingVC.view)
        
        NSLayoutConstraint.activate([
            hostingVC.view.topAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.topAnchor),
            hostingVC.view.leadingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.leadingAnchor),
            hostingVC.view.trailingAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.trailingAnchor),
            hostingVC.view.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor)
        ])
    }
    
    private func bind() {
        self.listener.$selectedLidarFileName
            .sink { [weak self] fileName in
                guard let fileName = fileName else { return }
                
                self?.showActionSheet(fileName: fileName)
            }
            .store(in: &self.cancellables)
    }
    
    private func showActionSheet(fileName: String) {
        let alert = UIAlertController(title: fileName, message: "", preferredStyle: .actionSheet)
        // Added section 1
        alert.addAction(UIAlertAction(title: "View File", style: .default, handler: { [weak self] _ in
            self?.moveToScanView(fileName: fileName)
        }))
        // End of added section 1
        alert.addAction(UIAlertAction(title: "Share File", style: .default, handler: { [weak self] _ in
            self?.shareScanData(fileName: fileName)
        }))
        alert.addAction(UIAlertAction(title: "Export Frames (ZIP)", style: .default, handler: { [weak self] _ in
            self?.exportFramesZip()
        }))
        alert.addAction(UIAlertAction(title: "Delete File", style: .destructive, handler: { [weak self] _ in
            ScanStorage.shared.remove(fileName: fileName)
            // list reload needed
            self?.showAlert(title: "File Deleted", text: "")
        }))
        alert.addAction(UIAlertAction(title: "Cancel", style: .default))
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            alert.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
        }
        
        self.present(alert, animated: true)
    }

    private func exportFramesZip() {
        let fileManager = FileManager.default
        let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first!
    let framesDir = documentsURL.appendingPathComponent("AccumulatedFrames", isDirectory: true)
    // Find the selected pointcloud file (assuming last selected or currently selected)
    let pointcloudFileName = self.listener.selectedLidarFileName ?? ""
    let pointcloudFileURL = ScanStorage.shared.fileUrl(fileName: pointcloudFileName)
    // Use the pointcloud file name (without extension) for the zip
    let zipBaseName = (pointcloudFileName as NSString).deletingPathExtension
    let tempZipURL = documentsURL.appendingPathComponent(zipBaseName.isEmpty ? "Export.zip" : "\(zipBaseName).zip")

    // Remove old zip if exists
    try? fileManager.removeItem(at: tempZipURL)

    guard let fileURLs = try? fileManager.contentsOfDirectory(at: framesDir, includingPropertiesForKeys: nil) else { return }

        if #available(iOS 16.0, *) {
            do {
                // Create a temp folder to zip both frames and pointcloud
                let tempFolder = documentsURL.appendingPathComponent("TempExport", isDirectory: true)
                try? fileManager.removeItem(at: tempFolder)
                try fileManager.createDirectory(at: tempFolder, withIntermediateDirectories: true)
                // Copy frames
                let framesCopy = tempFolder.appendingPathComponent("AccumulatedFrames", isDirectory: true)
                try fileManager.copyItem(at: framesDir, to: framesCopy)
                // Copy pointcloud
                if fileManager.fileExists(atPath: pointcloudFileURL.path) {
                    let destURL = tempFolder.appendingPathComponent(pointcloudFileName)
                    try? fileManager.copyItem(at: pointcloudFileURL, to: destURL)
                }
                try fileManager.zipItem(at: tempFolder, to: tempZipURL)
                try? fileManager.removeItem(at: tempFolder)
                let activityVC = UIActivityViewController(activityItems: [tempZipURL], applicationActivities: nil)
                if UIDevice.current.userInterfaceIdiom == .pad {
                    activityVC.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
                }
                self.present(activityVC, animated: true)
            } catch {
                self.showAlert(title: "Export Failed", text: error.localizedDescription)
            }
        } else {
            // ZIPFoundation fallback
            do {
                try zipWithZIPFoundation(sourceURL: framesDir, destinationURL: tempZipURL, pointcloudFileURL: pointcloudFileURL, pointcloudFileName: pointcloudFileName)
                let activityVC = UIActivityViewController(activityItems: [tempZipURL], applicationActivities: nil)
                if UIDevice.current.userInterfaceIdiom == .pad {
                    activityVC.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
                }
                self.present(activityVC, animated: true)
            } catch {
                self.showAlert(title: "Export Failed", text: error.localizedDescription)
            }
        }
    }

    private func zipWithZIPFoundation(sourceURL: URL, destinationURL: URL, pointcloudFileURL: URL, pointcloudFileName: String) throws {
        let archive = try Archive(url: destinationURL, accessMode: .create)
        let fileManager = FileManager.default
        let resourceKeys: [URLResourceKey] = [.isDirectoryKey]
        let enumerator = fileManager.enumerator(at: sourceURL, includingPropertiesForKeys: resourceKeys)!
        for case let fileURL as URL in enumerator {
            let filePath = fileURL.path.replacingOccurrences(of: sourceURL.path + "/", with: "AccumulatedFrames/")
            let fileAttributes = try fileURL.resourceValues(forKeys: Set(resourceKeys))
            if fileAttributes.isDirectory == true {
                try archive.addEntry(
                    with: filePath + "/",
                    type: .directory,
                    uncompressedSize: 0,
                    modificationDate: Date(),
                    permissions: nil,
                    compressionMethod: .none,
                    bufferSize: 16_384,
                    progress: nil,
                    provider: { (_: Int64, _: Int) -> Data in Data() }
                )
            } else {
                try archive.addEntry(with: filePath, fileURL: fileURL, compressionMethod: .deflate)
            }
        }
        // Add the pointcloud file
        if fileManager.fileExists(atPath: pointcloudFileURL.path) {
            try archive.addEntry(with: pointcloudFileName, fileURL: pointcloudFileURL, compressionMethod: .deflate)
        }
    }
    
    private func shareScanData(fileName: String) {
        let activityViewController = UIActivityViewController(activityItems: [ScanStorage.shared.fileUrl(fileName: fileName)], applicationActivities: nil)
        
        if UIDevice.current.userInterfaceIdiom == .pad {
            activityViewController.popoverPresentationController?.sourceView = self.navigationController?.navigationBar
        }
        
        self.present(activityViewController, animated: true)
        
        activityViewController.completionWithItemsHandler = { (activityType: UIActivity.ActivityType?, completed: Bool, arrayReturnedItems: [Any]?, error: Error?) in
            print("share success")
        }
    }
    // 추가한 부분 2
    private func moveToScanView(fileName: String) {
        guard let scanViewerVC = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: ScanViewerVC.identifier) as? ScanViewerVC else {
            print("Failed to instantiate ScanViewerVC")
            return
        }
        
        let fileURL = ScanStorage.shared.fileUrl(fileName: fileName)
        scanViewerVC.fileURL = fileURL
        
        self.navigationController?.pushViewController(scanViewerVC, animated: true)
    }
    // 추가한 부분 2 끝
}
