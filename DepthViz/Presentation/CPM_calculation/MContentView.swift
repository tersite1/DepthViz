
//
//  ContentView.swift
//  CPM_Graphic
//
//
import SwiftUI
import SwiftData

struct MContentView: View {
    @Environment(\.modelContext)  private var modelContext  // ModelContext 설정
    @State private var path = NavigationPath()
    @Query var activities: [Activity]
    @State private var projectResult: String?
    @State private var isShowingProjectResults = false
    @State private var startDateInput: String = "1"

    var body: some View {
        NavigationStack(path: $path) {
            ProjectView(startDateInput: $startDateInput)
                .navigationTitle("Activity List")
                .navigationDestination(for: Activity.self) { activity in
                    EditActivityView(navigationPath: $path, activity: activity)
                }
                .toolbar {
                    ToolbarItem(placement: .navigationBarLeading) {
                        Button {
                            calculateSchedule()
                        } label: {
                            Label("Schedule", systemImage: "calendar")
                        }
                    }

                    ToolbarItem(placement: .navigationBarTrailing) {
                        Button {
                            addActivity()
                        } label: {
                            Label("Add Activity", systemImage: "plus")
                        }
                    }
                }
                .navigationDestination(isPresented: $isShowingProjectResults) {
                    if let resultString = projectResult {
                        ProjectResultView(resultString: resultString)
                    }
                }
        }
    }

    func addActivity() {
        let newId = (activities.max(by: { $0.id < $1.id })?.id ?? 0) + 1
        let activity = Activity(id: newId, name: "", duration: 0)
        modelContext.insert(activity)
        path.append(activity)
    }

    func calculateSchedule() {
        guard let startDate = Int(startDateInput) else {
            print("Invalid start date input")
            return
        }

        let schedule = Schedule(startDate: startDate, schedule: activities)
        let project = Project(schedules: [schedule])
        project.scheduleCalculation()

        self.projectResult = project.result
        isShowingProjectResults = true
    }
}
