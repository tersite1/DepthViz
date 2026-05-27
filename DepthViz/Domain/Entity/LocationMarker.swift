//
//  LocationMarker.swift
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//

import Foundation
import CoreLocation

struct LocationMarker: Identifiable, Codable {
    let id: UUID
    let name: String
    let description: String
    let latitude: Double
    let longitude: Double
    let address: String?
    let placeId: String?
    let imageUrls: [String]  // Google Maps 이미지 URL
    
    var coordinate: CLLocationCoordinate2D {
        CLLocationCoordinate2D(latitude: latitude, longitude: longitude)
    }
    
    enum CodingKeys: String, CodingKey {
        case id, name, description, latitude, longitude, address, placeId, imageUrls
    }
    
    init(id: UUID = UUID(), name: String, description: String, latitude: Double, longitude: Double, address: String? = nil, placeId: String? = nil, imageUrls: [String] = []) {
        self.id = id
        self.name = name
        self.description = description
        self.latitude = latitude
        self.longitude = longitude
        self.address = address
        self.placeId = placeId
        self.imageUrls = imageUrls
    }
    
    // Custom coding for CLLocationCoordinate2D is not needed if we store lat/long
}

extension LocationMarker {
    static let samples: [LocationMarker] = [
        LocationMarker(
            name: "서울역",
            description: "서울의 주요 기차역",
            latitude: 37.5546788,
            longitude: 126.9706786,
            address: "서울특별시 중구 세종대로 2",
            imageUrls: [
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSxmdng8IVHcqaxYpkGQCbHwGwmT4u4CVKrlVKBhXjFggu3M4gtyoxlRn2dA4ZPKtyl_ub866v3P4hvIjZxS2Bq3Yi9h0qxb4OcC1nCJhNUOwGNloWmhFnNIj43TN58kn6jKNnOx=w520-h350-n-k-no",
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSwopyoqgHOH45zQVJp9MxDVwFAEub65-rCYOGnusINB0Co22Ov-AtWQrzeZ05oVvlcbY1gujFAQEZ_x2Ba0DpPTf6_A7ZiVQU6ut8bJw3mpIQ75EF_h9_pJ9Fyz2uEYiawzuQY6rw=w260-h174-n-k-no"
            ]
        ),
        LocationMarker(
            name: "강남역",
            description: "강남의 중심지",
            latitude: 37.4979898,
            longitude: 127.0276316,
            address: "서울특별시 강남구 강남대로 396",
            imageUrls: [
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSyHY2dYZ8H4gunaw7qO8bGQpl8bG1niFTQ_xl-zi8wVNsnHyU9CbQnalH-wDDTDVihQ0P2__ZJJpJs8FHdViT-Q7BAkEuyW2HYhUQWcTYJHNYuC0KmO6VZvZTLBnMbflggqNMI=w520-h350-n-k-no",
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSwIQGI_zfQEM3uGcpG7JvGb97j3vpOSij0v3Pgz0cAmt6QFvCJuGt-wK3oTy5nTUBiN_BSI--ysvWvdvpL1FrW2mmt45TRY0YMzE9yOZZMDhHQnVIiQgk0plqRCtwgg9dZxhsA=w260-h174-n-k-no"
            ]
        ),
        LocationMarker(
            name: "N서울타워",
            description: "서울의 랜드마크",
            latitude: 37.5511694,
            longitude: 126.9882266,
            address: "서울특별시 용산구 남산공원길 105",
            imageUrls: [
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSztv04BFLdcgm0ovX_XXrnzxY4FhvpzP17an4ttpmTSJCohBPu-HCrgF6EuNlVvqoqyorX9I6FfSfz_BPqEapDNu_T4TasC4jrt-06m-WNvHlgwHjefMc22v4-cM0rjDGtqZ8PU=w244-h306-n-k-no-nu",
                "https://lh3.googleusercontent.com/gps-cs-s/AG0ilSwSZQ2SXwvb6br3nV-b3GSYqcCsK1hNfyokas4JLrGHm1PeZRzXyxtwYQxqS2Yode05sr_JuPq-98UQsi7WcAQP5PDjr8CjASw5CCMlVSCQHKAkXnhUb-LC3xm-vSt2P1zAVXaR=w135-h156-n-k-no"
            ]
        ),
        LocationMarker(
            name: "KAIST IT 융합빌딩",
            description: "한국과학기술원 IT 융합연구센터",
            latitude: 36.3717,
            longitude: 127.3617,
            address: "대전광역시 유성구 대학로 291",
            imageUrls: [
                "https://autoidlab.kaist.ac.kr/img/kaist_n1_03.jpg",
                "https://autoidlab.kaist.ac.kr/img/kaist_n1_03.jpg"
            ]
        ),
        LocationMarker(
            name: "연세대학교",
            description: "대한민국의 명문 사립대학",
            latitude: 37.5665,
            longitude: 126.9388,
            address: "서울특별시 서대문구 연세로 50",
            imageUrls: [
                "https://www.yonsei.ac.kr/ocx/data/file/sc_intro/2085395896_hLv8N7RD_1d1afe5b851c6c9ed13cce61b80db46bcc35e71d.jpg",
                "https://www.yonsei.ac.kr/ocx/data/file/sc_intro/2085395896_hLv8N7RD_1d1afe5b851c6c9ed13cce61b80db46bcc35e71d.jpg"
            ]
        )
    ]
}
