//
//  Activity.swift
//  CPM_Graphic
//
//  Created by 김형관 on 4/2/24.
//
import Foundation
import SwiftData

@Model
class Activity {
    public var id: Int
    public var name: String
    public var duration: Int
    public var parentPred: [Activity] = []
    public var parentSucc: [Activity] = []

    @Relationship(deleteRule: .nullify, inverse: \Activity.parentPred)
    public var predecessors: [Activity] = []

    @Relationship(deleteRule: .nullify, inverse: \Activity.parentSucc)
    public var successors: [Activity] = []

    public var earlyStart: Int = 0
    public var earlyFinish: Int = 0
    public var lateStart: Int = 0
    public var lateFinish: Int = 0
    public var totalFloat: Int = 0
    public var freeFloat: Int = 0
    public var actualStart: Int = 0
    public var actualFinish: Int = 0

    required init() {
        self.id = 0
        self.name = ""
        self.duration = 0
    }

    init(id: Int, name: String, duration: Int, predecessors: [Activity] = [], successors: [Activity] = []) {
        self.id = id
        self.name = name
        self.duration = duration
        self.predecessors = predecessors
        self.successors = successors
    }

    init(id: Int, name: String, duration: Int) {
        self.id = id
        self.name = name
        self.duration = duration
    }

    public func setEarlyTime(earlyStart: Int, earlyFinish: Int) {
        self.earlyStart = earlyStart
        self.earlyFinish = earlyFinish
    }

    public func setLateTime(lateStart: Int, lateFinish: Int) {
        self.lateStart = lateStart
        self.lateFinish = lateFinish
    }

    public func isFirst() -> Bool {
        if predecessors.isEmpty {
            return true
        } else {
            let predArray = predecessors.map { $0.actualFinish }
            return predArray.allSatisfy { $0 > 0 }
        }
    }

    public func setRelation(predecessors: [Activity] = [], successors: [Activity] = []) {
        self.predecessors = predecessors
        self.successors = successors
    }
}
