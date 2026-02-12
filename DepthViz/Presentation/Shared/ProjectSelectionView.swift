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

                            VStack(spacing: 0) {
                                ForEach(projects, id: \.self) { project in
                                    Button(action: {
                                        selectedProject = project
                                        isCreatingNew = false
                                    }) {
                                        HStack {
                                            Text(project)
                                                .foregroundColor(.primary)
                                                .lineLimit(1)
                                            Spacer()
                                            if selectedProject == project && !isCreatingNew {
                                                Image(systemName: "checkmark")
                                                    .foregroundColor(.blue)
                                                    .fontWeight(.semibold)
                                            }
                                        }
                                        .padding(.horizontal, 16)
                                        .padding(.vertical, 13)
                                        .background(
                                            (selectedProject == project && !isCreatingNew)
                                                ? Color.blue.opacity(0.08)
                                                : Color.clear
                                        )
                                    }
                                    if project != projects.last {
                                        Divider().padding(.leading, 16)
                                    }
                                }

                                if !projects.isEmpty {
                                    Divider().padding(.leading, 16)
                                }

                                // 새 프로젝트 행
                                Button(action: {
                                    isCreatingNew = true
                                    selectedProject = ""
                                }) {
                                    HStack(spacing: 8) {
                                        Image(systemName: "plus.circle.fill")
                                            .foregroundColor(.blue)
                                        Text("New Project")
                                            .foregroundColor(.blue)
                                            .fontWeight(.medium)
                                        Spacer()
                                        if isCreatingNew {
                                            Image(systemName: "checkmark")
                                                .foregroundColor(.blue)
                                                .fontWeight(.semibold)
                                        }
                                    }
                                    .padding(.horizontal, 16)
                                    .padding(.vertical, 13)
                                    .background(isCreatingNew ? Color.blue.opacity(0.08) : Color.clear)
                                }
                            }
                            .background(Color(.secondarySystemGroupedBackground))
                            .cornerRadius(12)
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
                    }
                    .padding(.bottom, 20)
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
