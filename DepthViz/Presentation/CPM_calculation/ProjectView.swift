//
//  ProjectView.swift
//  CPM_Graphic
//
//  Created by 김형관 on 4/2/24.
//

import SwiftData
import SwiftUI
struct ProjectView: View {
    
    @Environment(\.modelContext) var modelContext
    @Query (sort: [
        SortDescriptor(\Activity.id),
        SortDescriptor(\Activity.name)
    ]) var activities: [Activity]
    @Binding var startDateInput : String
    
    
    var body: some View {
        VStack {
            TextField("Start Date", text: $startDateInput)
                .keyboardType(.numberPad) // Ensures numeric input
                .padding()
            List {
                ForEach(activities) { activity in
                    NavigationLink(value: activity) {
                        Text("ID: \(activity.id), Name: \(activity.name)")
                    }
                }
                .onDelete(perform: deleteActivity)
            }
        }
    }
    
    func deleteActivity (at offsets: IndexSet) {
        for offset in offsets {
            let activity = activities[offset]
            modelContext.delete(activity)
        }
    }
}

struct ProjectResultView: View {
    let resultString: String
    @State private var isShowingGraphicalResults = false // State to manage presentation
    @StateObject var activityPositions = ActivityPositions() // Defined in the parent view
    
    var body: some View {
        
        ScrollView {
            Text(resultString)
                .padding()
            
            Button("Show Graphical View") {
                isShowingGraphicalResults = true
            }
            .sheet(isPresented: $isShowingGraphicalResults) {
                GraphicalResultView() // The view to show in a modal sheet
                    .environmentObject(activityPositions) // Provide the instance here
                
            }
        }
        .navigationTitle("Project Results")
        
    }
}

struct GraphicalResultView: View {
    @Query(sort: [
        SortDescriptor(\Activity.id),
        SortDescriptor(\Activity.name)
    ]) var activities: [Activity]
    @EnvironmentObject var activityPositions: ActivityPositions
    
    // Function to group and sort activities by `earlyStart`
    private func groupedAndSortedActivities() -> [[Activity]] {
        let grouped = Dictionary(grouping: activities) { $0.earlyStart }
        return grouped.sorted { $0.key < $1.key }.map { $0.value }
    }
    
    var body: some View {
        GeometryReader { geo in
            ScrollView {
                VStack () {
                    ForEach(groupedAndSortedActivities(), id: \.self) { activityGroup in
                        HStack(spacing: 10) {
                            ForEach(activityGroup, id: \.id) { activity in
                                ActivityBlockView(activity: activity)
                            }
                        }
                    }
                }
            }
            .overlay(
                ArrowOverlay()
                    .environmentObject(activityPositions)
            )
        }
    }
}


struct ActivityBlockView: View {
    var activity: Activity
    @EnvironmentObject var activityPositions: ActivityPositions
    
    var body: some View {
        Rectangle()
            .fill(activity.totalFloat == 0 ? Color.red : Color.blue)
            .frame(width: 90, height: 120)
        
            .overlay(
                VStack {
                    Text("\(activity.name)   Du: \(activity.duration)")
                    
                    HStack {
                        Text("ES: \(activity.earlyStart)")
                        Text("EF: \(activity.earlyFinish)")
                    }
                    
                    HStack {
                        Text("LS: \(activity.lateStart)")
                        Text("LF: \(activity.lateFinish)")
                    }

                }
                    .foregroundColor(.white)
            )
            .overlay(
                GeometryReader { geometry in
                    
                    Color.clear.onAppear {
                        let frame = geometry.frame(in: .global)
                        let topCenter = CGPoint(x: frame.midX, y: frame.minY)
                        let bottomCenter = CGPoint(x: frame.midX, y: frame.maxY)
                        
                        activityPositions.updatePosition(for: activity.id, top: topCenter, bottom: bottomCenter)
                    }
                }
                
            )
            .padding(.vertical)

    }
}


class ActivityPositions: ObservableObject {
    // Maps Activity ID to its rectangle's top and bottom center positions
    @Published var positions: [Int: (top: CGPoint, bottom: CGPoint)] = [:]
    
    func updatePosition(for id: Int, top: CGPoint, bottom: CGPoint) {
        positions[id] = (top, bottom)
    }
    
    func position(for id: Int) -> (top: CGPoint, bottom: CGPoint)? {
        return positions[id]
    }
}

struct ArrowOverlay: View {
    @EnvironmentObject var activityPositions: ActivityPositions
    @Query(sort: [
        SortDescriptor(\Activity.id),
        SortDescriptor(\Activity.name)
    ]) var activities: [Activity]

    var body: some View {
        Canvas { context, size in
            for activity in activities {
                // 현재 activity의 화살표 시작 위치를 가져옴
                guard let currentActivityPosition = activityPositions.position(for: activity.id) else { continue }

                // 각 successor에 대해 화살표를 그림
                for successor in activity.successors {
                    guard let successorPosition = activityPositions.position(for: successor.id) else { continue }

                    // 화살표 시작 위치는 현재 activity의 하단 중앙
                    let start = CGPoint(x: currentActivityPosition.bottom.x, y: currentActivityPosition.bottom.y-70)
                    // 화살표 끝 위치는 successor의 상단 중앙
                    let end = CGPoint(x: successorPosition.top.x, y: successorPosition.top.y-70)

                    // 화살표를 그리는 함수 호출
                    drawArrow(from: start, to: end, in: context)
                }
            }
        }
    }

    func drawArrow(from start: CGPoint, to end: CGPoint, in context: GraphicsContext) {
        var path = Path()
        path.move(to: start)
        path.addLine(to: end)

        let arrowHeadLength: CGFloat = 15
        let arrowHeadAngle: CGFloat = .pi / 6  // 30 degrees

        let angle = atan2(end.y - start.y, end.x - start.x)

        let arrowPoint1 = CGPoint(
            x: end.x - arrowHeadLength * cos(angle + arrowHeadAngle),
            y: end.y - arrowHeadLength * sin(angle + arrowHeadAngle)
        )
        let arrowPoint2 = CGPoint(
            x: end.x - arrowHeadLength * cos(angle - arrowHeadAngle),
            y: end.y - arrowHeadLength * sin(angle - arrowHeadAngle)
        )

        context.stroke(path, with: .color(.black), lineWidth: 2)

        var arrowHead = Path()
        arrowHead.move(to: end)
        arrowHead.addLine(to: arrowPoint1)
        arrowHead.addLine(to: arrowPoint2)
        arrowHead.addLine(to: end)
        arrowHead.closeSubpath()

        context.fill(arrowHead, with: .color(.black))
    }
}
