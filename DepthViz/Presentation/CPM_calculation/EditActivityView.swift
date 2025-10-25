import SwiftData
import SwiftUI

struct EditActivityView: View {
    @Binding var navigationPath: NavigationPath
    @Bindable var activity: Activity
    @State private var selectedPredecessorId: Int?
    @State private var selectedSuccessorId: Int?
    @Query var activities: [Activity]
    @State private var isEditing = false  // State to track edit mode

    var body: some View {
        Form {
            Section(header: Text("General Information")) {
                HStack {
                    Text("ID:")
                    Spacer()
                    TextField("ID", value: $activity.id, formatter: NumberFormatter()).disabled(!isEditing)
                }
                
                HStack {
                    Text("Name:")
                    Spacer()
                    TextField("Name", text: $activity.name).disabled(!isEditing)
                }
                HStack {
                    Text("Duration:")
                    Spacer()
                    TextField("Duration", value: $activity.duration, formatter: NumberFormatter()).disabled(!isEditing)
                }
            }
            
            Section(header: Text("Predecessors")) {
                if activity.predecessors.isEmpty {
                    Text("No predecessors")
                } else {
                    ForEach(activity.predecessors.sorted(by: { $0.id < $1.id }), id: \.id) { predecessor in
                        Text(predecessor.name)
                    }
                    .onDelete(perform: isEditing ? removePredecessor : nil)
                }
                
                if isEditing {
                    Picker("Select New Predecessor", selection: $selectedPredecessorId) {
                        Text("None").tag(nil as Int?)
                        ForEach(activities.filter { $0.id != activity.id }, id: \.id) { activity in
                            Text("\(activity.name) (\(activity.id))").tag(activity.id as Int?)
                        }
                    }
                    .onChange(of: selectedPredecessorId) {
                        addPredecessor()
                    }
                }
            }

            Section(header: Text("Successors")) {
                if activity.successors.isEmpty {
                    Text("No successors")
                } else {
                    ForEach(activity.successors.sorted(by: { $0.id < $1.id }), id: \.id) { successor in
                        Text(successor.name)
                    }
                    .onDelete(perform: isEditing ? removeSuccessor : nil)
                }

                if isEditing {
                    Picker("Select New Successor", selection: $selectedSuccessorId) {
                        Text("None").tag(nil as Int?)
                        ForEach(activities.filter { $0.id != activity.id }, id: \.id) { activity in
                            Text("\(activity.name) (\(activity.id))").tag(activity.id as Int?)
                        }
                    }
                    .onChange(of: selectedSuccessorId) {
                        addSuccessor()
                    }
                }
            }
        }
        .navigationTitle("Edit Activity")
        .navigationBarItems(trailing: Button(isEditing ? "Done" : "Edit") {
            isEditing.toggle()
        })
        .navigationBarTitleDisplayMode(.inline)
    }

    private func addPredecessor() {
        guard let newId = selectedPredecessorId,
              let newPredecessor = activities.first(where: { $0.id == newId }),
              !activity.predecessors.contains(where: { $0.id == newId })
        else { return }
        
        activity.predecessors.append(newPredecessor)
        selectedPredecessorId = nil // Reset selection
    }

    private func removePredecessor(at offsets: IndexSet) {
        activity.predecessors.remove(atOffsets: offsets)
    }

    private func addSuccessor() {
        guard let newId = selectedSuccessorId,
              let newSuccessor = activities.first(where: { $0.id == newId }),
              !activity.successors.contains(where: { $0.id == newId })
        else { return }
        
        activity.successors.append(newSuccessor)
        selectedSuccessorId = nil // Reset selection
    }

    private func removeSuccessor(at offsets: IndexSet) {
        activity.successors.remove(atOffsets: offsets)
    }
}
