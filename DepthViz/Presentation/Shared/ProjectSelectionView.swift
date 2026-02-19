//
//  ProjectSelectionView.swift
//  DepthViz
//
//  프로젝트 선택 + 파일명 입력 하단 시트
//

import SwiftUI

struct ProjectSelectionView: View {
    let defaultFileName: String
    let pointCount: Int
    let markerProjectName: String?
    let onSave: (String, String) -> Void   // (project, fileName)
    let onCancel: () -> Void

    @State private var selectedProject: String = ""
    @State private var fileName: String = ""
    @State private var isCreatingNew = false
    @State private var newProjectName: String = ""
    @State private var projects: [String] = []
    @State private var selectedFormat: FileFormat = .plyAscii

    var body: some View {
        NavigationView {
            VStack(spacing: 0) {
                ScrollView {
                    VStack(spacing: 20) {
                        // 포인트 수 정보
                        HStack(spacing: 8) {
                            Image(systemName: "point.3.connected.trianglepath.dotted")
                                .foregroundColor(.blue)
                                .font(.system(size: 14))
                            Text(formattedPointCount)
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            Spacer()
                        }
                        .padding(.horizontal, 20)
                        .padding(.top, 8)

                        // 프로젝트 선택
                        VStack(alignment: .leading, spacing: 10) {
                            Text("Project")
                                .font(.headline)
                                .padding(.horizontal, 20)

                            // 프로젝트 그리드 (2열)
                            let columns = [
                                GridItem(.flexible(), spacing: 10),
                                GridItem(.flexible(), spacing: 10)
                            ]
                            LazyVGrid(columns: columns, spacing: 10) {
                                ForEach(projects, id: \.self) { project in
                                    let isSelected = selectedProject == project && !isCreatingNew
                                    Button(action: {
                                        selectedProject = project
                                        isCreatingNew = false
                                    }) {
                                        VStack(spacing: 6) {
                                            Image(systemName: isSelected ? "folder.fill" : "folder")
                                                .font(.system(size: 24))
                                                .foregroundColor(isSelected ? .blue : .gray)
                                            Text(project)
                                                .font(.system(size: 13, weight: isSelected ? .semibold : .regular))
                                                .foregroundColor(isSelected ? .blue : .primary)
                                                .lineLimit(1)
                                                .truncationMode(.tail)
                                        }
                                        .frame(maxWidth: .infinity)
                                        .padding(.vertical, 14)
                                        .background(
                                            RoundedRectangle(cornerRadius: 10)
                                                .fill(isSelected ? Color.blue.opacity(0.1) : Color(.secondarySystemGroupedBackground))
                                        )
                                        .overlay(
                                            RoundedRectangle(cornerRadius: 10)
                                                .stroke(isSelected ? Color.blue.opacity(0.5) : Color.clear, lineWidth: 1.5)
                                        )
                                    }
                                    .buttonStyle(.plain)
                                }

                                // 새 프로젝트
                                Button(action: {
                                    isCreatingNew = true
                                    selectedProject = ""
                                }) {
                                    VStack(spacing: 6) {
                                        Image(systemName: isCreatingNew ? "folder.fill.badge.plus" : "folder.badge.plus")
                                            .font(.system(size: 24))
                                            .foregroundColor(isCreatingNew ? .blue : .gray)
                                        Text("New")
                                            .font(.system(size: 13, weight: isCreatingNew ? .semibold : .regular))
                                            .foregroundColor(isCreatingNew ? .blue : .secondary)
                                    }
                                    .frame(maxWidth: .infinity)
                                    .padding(.vertical, 14)
                                    .background(
                                        RoundedRectangle(cornerRadius: 10)
                                            .fill(isCreatingNew ? Color.blue.opacity(0.1) : Color(.secondarySystemGroupedBackground))
                                    )
                                    .overlay(
                                        RoundedRectangle(cornerRadius: 10)
                                            .stroke(isCreatingNew ? Color.blue.opacity(0.5) : Color.clear, lineWidth: 1.5)
                                    )
                                }
                                .buttonStyle(.plain)
                            }
                            .padding(.horizontal, 16)

                            // 새 프로젝트명 입력
                            if isCreatingNew {
                                TextField("Project Name", text: $newProjectName)
                                    .textFieldStyle(RoundedBorderTextFieldStyle())
                                    .padding(.horizontal, 20)
                            }
                        }

                        // 파일명 입력
                        VStack(alignment: .leading, spacing: 10) {
                            Text("File Name")
                                .font(.headline)
                                .padding(.horizontal, 20)

                            TextField("File Name", text: $fileName)
                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                .padding(.horizontal, 20)
                        }

                        // 저장 형식 선택
                        VStack(alignment: .leading, spacing: 10) {
                            Text("Format")
                                .font(.headline)
                                .padding(.horizontal, 20)

                            let formatColumns = [
                                GridItem(.flexible(), spacing: 8),
                                GridItem(.flexible(), spacing: 8),
                                GridItem(.flexible(), spacing: 8),
                                GridItem(.flexible(), spacing: 8)
                            ]
                            LazyVGrid(columns: formatColumns, spacing: 8) {
                                ForEach(FileFormat.allCases, id: \.self) { format in
                                    let isSelected = selectedFormat == format
                                    let isRecommended = format == .plyBinary
                                    Button(action: {
                                        selectedFormat = format
                                        ScanSettings.shared.fileFormat = format
                                    }) {
                                        VStack(spacing: 4) {
                                            Image(systemName: "doc.fill")
                                                .font(.system(size: 18))
                                                .foregroundColor(isSelected ? .white : .gray)
                                            Text(format.displayName)
                                                .font(.system(size: 11, weight: isSelected ? .semibold : .regular))
                                                .foregroundColor(isSelected ? .white : .primary)
                                                .lineLimit(1)
                                                .minimumScaleFactor(0.8)
                                            if isRecommended {
                                                Text("Recommended")
                                                    .font(.system(size: 8, weight: .medium))
                                                    .foregroundColor(isSelected ? .white.opacity(0.8) : .blue)
                                            } else {
                                                Text(" ")
                                                    .font(.system(size: 8))
                                            }
                                        }
                                        .frame(maxWidth: .infinity)
                                        .padding(.vertical, 10)
                                        .background(
                                            RoundedRectangle(cornerRadius: 8)
                                                .fill(isSelected ? Color.blue : Color(.secondarySystemGroupedBackground))
                                        )
                                        .overlay(
                                            RoundedRectangle(cornerRadius: 8)
                                                .stroke(isSelected ? Color.blue.opacity(0.5) : Color.clear, lineWidth: 1.5)
                                        )
                                    }
                                    .buttonStyle(.plain)
                                }
                            }
                            .padding(.horizontal, 16)
                        }
                    }
                    .padding(.bottom, 100)
                }

                // 하단 버튼 영역
                VStack(spacing: 10) {
                    Button(action: {
                        let project = isCreatingNew
                            ? newProjectName.trimmingCharacters(in: .whitespacesAndNewlines)
                            : selectedProject
                        let name = fileName.trimmingCharacters(in: .whitespacesAndNewlines)
                        guard !project.isEmpty, !name.isEmpty else { return }
                        if isCreatingNew {
                            ScanStorage.shared.addProject(project)
                        }
                        onSave(project, name)
                    }) {
                        Text("Save")
                            .font(.system(size: 17, weight: .semibold))
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .frame(height: 50)
                            .background(canSave ? Color.blue : Color.gray.opacity(0.4))
                            .cornerRadius(12)
                    }
                    .disabled(!canSave)

                    Button("Cancel") {
                        onCancel()
                    }
                    .foregroundColor(.secondary)
                    .font(.system(size: 15))
                }
                .padding(.horizontal, 20)
                .padding(.bottom, 16)
            }
            .background(Color(.systemGroupedBackground))
            .navigationTitle("Save Scan")
            .navigationBarTitleDisplayMode(.inline)
        }
        .onAppear {
            projects = ScanStorage.shared.getProjects()
            fileName = defaultFileName
            selectedFormat = ScanSettings.shared.fileFormat

            if let markerName = markerProjectName {
                // 마커 프로젝트 자동 선택
                if !projects.contains(markerName) {
                    ScanStorage.shared.addProject(markerName)
                    projects = ScanStorage.shared.getProjects()
                }
                selectedProject = markerName
            } else if let first = projects.first {
                selectedProject = first
            } else {
                // 프로젝트가 없으면 자동으로 새 프로젝트 생성 모드
                isCreatingNew = true
            }
        }
    }

    private var canSave: Bool {
        let project = isCreatingNew
            ? newProjectName.trimmingCharacters(in: .whitespacesAndNewlines)
            : selectedProject
        let name = fileName.trimmingCharacters(in: .whitespacesAndNewlines)
        return !project.isEmpty && !name.isEmpty
    }

    private var formattedPointCount: String {
        let formatter = NumberFormatter()
        formatter.numberStyle = .decimal
        return (formatter.string(from: NSNumber(value: pointCount)) ?? "\(pointCount)") + " Points"
    }
}
