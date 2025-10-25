//
//  Schedule.swift
//  CPM_Graphic
//
//  Created by 김형관 on 4/2/24.
//



class Schedule {
    var startDate: Int = 1
    var schedule = [Activity]()
    
    init() {
        
    }
    
    init (schedule: [Activity]) {
        self.schedule = schedule
    }
    
    init(startDate: Int, schedule: [Activity] ) {
        self.startDate = startDate
        self.schedule = schedule
    }
}
