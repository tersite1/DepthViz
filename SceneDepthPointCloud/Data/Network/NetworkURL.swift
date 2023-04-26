//
//  NetworkURL.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/03/27.
//  Copyright © 2023 Apple. All rights reserved.
//

import Foundation

struct NetworkURL {
    static let baseURL: String = Bundle.main.infoDictionary!["BASE_URL"] as! String
    
    enum Domain {
        static let server = baseURL + "/typers"
        static let kakao = "https://dapi.kakao.com"
    }
    
    enum Server {
        static let lidars: String = Domain.server + "/lidars"
    }
    
    enum Kakao {
        static let coordToAddress: String = Domain.kakao + "/v2/local/geo/coord2address.JSON"
        static let searchByCategory: String = Domain.kakao + "/v2/local/search/category.JSON"
    }
}
