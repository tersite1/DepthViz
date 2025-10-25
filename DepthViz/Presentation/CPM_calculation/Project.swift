//
//  Project.swift
//  CPM_Graphic
//
//  Created by 김형관 on 4/2/24.
//

class Project {
    init(result: String = String(), criticalPaths: [[Activity]] = [[Activity]](), criticalPath: [Activity] = [Activity](), schedules: [Schedule]) {
        self.result = result
        self.criticalPaths = criticalPaths
        self.criticalPath = criticalPath
        self.schedules = schedules
    }
    
    var result = String()
    var criticalPaths = [[Activity]]()
    var criticalPath = [Activity]()
    
    
    var schedules: [Schedule] = []
    private func forwardPass(activities: [Activity], startDay: Int) -> Int {
        var comparisonEarlyFinish = 0
        for activity in activities {
            if activity.actualFinish > 0 { continue }
            if activity.isFirst() { // Assuming `isFirst` now does not take parameters and directly uses `predecessors`
                activity.setEarlyTime(earlyStart: startDay, earlyFinish: startDay + activity.duration)
                comparisonEarlyFinish = startDay + activity.duration
            } else {
                var comparison = 0
                // Directly iterate over `predecessors` since it's no longer optional
                for predecessor in activity.predecessors {
                    if comparison < predecessor.earlyFinish {
                        comparison = predecessor.earlyFinish
                    }
                }
                activity.setEarlyTime(earlyStart: comparison, earlyFinish: comparison + activity.duration)
                if comparisonEarlyFinish < comparison + activity.duration {
                    comparisonEarlyFinish = comparison + activity.duration
                }
            }
        }
        return comparisonEarlyFinish
    }


    private func backwardPass(activities: [Activity], finishDay: Int) {
        for activity in activities.reversed() {
            if activity.actualFinish > 0 { continue }
            if activity.successors.isEmpty {
                // If there are no successors, set late times based on the finish day
                activity.setLateTime(lateStart: finishDay - activity.duration, lateFinish: finishDay)
            } else {
                // Find the minimum late start among all successors to determine this activity's late finish
                var comparison = Int.max
                for successor in activity.successors {
                    if comparison > successor.lateStart {
                        comparison = successor.lateStart
                    }
                }
                // Set the late start and finish times based on the earliest successor start
                activity.setLateTime(lateStart: comparison - activity.duration, lateFinish: comparison)
            }
        }
    }

    

    private func floatCalculation(activities: [Activity]) -> [Activity] {
        var criticalActivities: [Activity] = []
        for activity in activities {
            let startTotalFloat = activity.lateStart - activity.earlyStart
            let finishTotalFloat = activity.lateFinish - activity.earlyFinish
            activity.totalFloat = min(startTotalFloat, finishTotalFloat)
            if activity.totalFloat == 0 {
                activity.freeFloat = 0
            } else {
                var comparison = 100000 // A very large number used as initial comparison value
                // Directly check if successors are not empty; otherwise, set comparison to lateFinish
                if !activity.successors.isEmpty {
                    for successor in activity.successors {
                        if comparison > successor.earlyStart {
                            comparison = successor.earlyStart
                        }
                    }
                } else {
                    comparison = activity.lateFinish
                }
                activity.freeFloat = comparison - activity.earlyFinish
            }
        }
        criticalActivities = activities.filter { $0.totalFloat == 0 }
        let temp = criticalActivities.filter { $0.actualFinish == 0 }
        return temp
    }

    
    private func searchCP(activities: [Activity], activity: Activity, criticalActivities: [Activity]) {
        if activity.isFirst() { // Assuming isFirst has been updated to not require parameters
            criticalPath.removeAll()
            criticalPath.append(activity)
        }
        // Since successors is now a non-optional array, you can check isEmpty directly
        if activity.successors.isEmpty { return }
        
        for successor in activity.successors {
            if successor.totalFloat == 0 {
                criticalPath.append(successor)
                // Directly check if successors are empty
                if successor.successors.isEmpty {
                    criticalPaths.append(Array(criticalPath)) // Make a copy of the current critical path
                }
                searchCP(activities: activities, activity: successor, criticalActivities: criticalActivities)
                criticalPath.removeLast()
            }
        }
    }


    
    private func criticalPathFind(activities: [Activity], criticalActivities: [Activity]) {
        criticalPath.removeAll()
        criticalPaths.removeAll()
        for activity in criticalActivities {
            if activity.isFirst() { // Assuming isFirst is updated accordingly
                searchCP(activities: activities, activity: activity, criticalActivities: criticalActivities)
            }
        }
    }


    
    private func printProject(activities: [Activity], criticalActivities: [Activity]) {
        
        for activity in activities {
            
            if (activity.actualFinish != 0) {
                result.append("ID = \(activity.id): \(activity.name)\n" )
                result.append("Actual Start: \(activity.actualStart)\n")
                result.append("Actual Finish: \(activity.actualFinish)\n")
            } else if (activity.actualStart != 0) {
                result.append("ID = \(activity.id): \(activity.name)\n" )
                result.append("Actual Start: \(activity.actualStart)\n")
                result.append("Early Finish: \(activity.earlyFinish)\n")
                result.append("Late Finish: \(activity.lateFinish)\n")
                result.append("Total Float: \(activity.totalFloat)\n")
                result.append("Free Float: \(activity.freeFloat)\n")
            } else {
                result.append("ID = \(activity.id): \(activity.name)\n" )
                result.append("Early Start: \(activity.earlyStart)\n")
                result.append("Early Finish: \(activity.earlyFinish)\n")
                result.append("Late Start: \(activity.lateStart)\n")
                result.append("Late Finish: \(activity.lateFinish)\n")
                result.append("Total Float: \(activity.totalFloat)\n")
                result.append("Free Float: \(activity.freeFloat)\n")
            }
            
            
            result.append("\n")
        }

        result.append("Critical Paths \n")
        criticalPathFind(activities: activities, criticalActivities: criticalActivities)
//
//        for index in criticalPaths.indices {
//            criticalPaths[index].sort { $0.id < $1.id } // Sorting each critical path by activity.id
//        }
        criticalPaths.sort { $0.first?.id ?? Int.max < $1.first?.id ?? Int.max }

        for criticalPath in criticalPaths {
            for activity in criticalPath {
                result.append("\(activity.id) ")
            }
            result.append("\n")
        }
        result.append("\n")
    }
    
    public func scheduleCalculation () {
        for schedule in schedules {
            let finishDay = forwardPass(activities: schedule.schedule , startDay: schedule.startDate)
            backwardPass(activities: schedule.schedule, finishDay: finishDay)
            let criticalActivities = floatCalculation(activities: schedule.schedule)
            printProject(activities: schedule.schedule, criticalActivities: criticalActivities)
        }
    }
}
