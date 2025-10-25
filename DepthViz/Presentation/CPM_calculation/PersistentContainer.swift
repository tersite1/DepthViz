//
//  PersistentContainer.swift
//  ConstructionEagleEye
//
//  Created by HYUNAHKO on 6/9/24.
//

// PersistentContainer.swift
import CoreData

class PersistentContainer {
    static let shared = PersistentContainer()
    
    let container: NSPersistentContainer
    
    private init() {
        container = NSPersistentContainer(name: "CPM_Graphic")
        container.loadPersistentStores { description, error in
            if let error = error {
                fatalError("Unable to load persistent stores: \(error)")
            }
        }
    }
    
    func saveContext() {
        let context = container.viewContext
        if context.hasChanges {
            do {
                try context.save()
            } catch {
                fatalError("Unresolved error \(error)")
            }
        }
    }
}

