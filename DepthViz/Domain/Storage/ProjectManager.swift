//
//  ProjectManager.swift
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

import Foundation

class ProjectManager: ObservableObject {
    static let shared = ProjectManager()
    
    @Published var projects: [Project] = []
    
    private let storageKey = "DepthViz_Projects"
    
    private init() {
        loadProjects()
    }
    
    func createProject(name: String, location: LocationMarker?) -> Project {
        let newProject = Project(name: name, location: location)
        projects.insert(newProject, at: 0)
        saveProjects()
        return newProject
    }
    
    func updateProject(_ project: Project) {
        if let index = projects.firstIndex(where: { $0.id == project.id }) {
            projects[index] = project
            projects[index].updatedAt = Date()
            saveProjects()
        }
    }
    
    func deleteProject(_ project: Project) {
        projects.removeAll { $0.id == project.id }
        // TODO: Delete associated files
        saveProjects()
    }
    
    func getProject(byId id: UUID) -> Project? {
        return projects.first(where: { $0.id == id })
    }

    func getProject(byName name: String) -> Project? {
        return projects.first(where: { $0.name == name })
    }
    
    private func saveProjects() {
        if let encoded = try? JSONEncoder().encode(projects) {
            UserDefaults.standard.set(encoded, forKey: storageKey)
        }
    }
    
    private func loadProjects() {
        if let data = UserDefaults.standard.data(forKey: storageKey),
           let decoded = try? JSONDecoder().decode([Project].self, from: data) {
            projects = decoded
        }
    }
}
