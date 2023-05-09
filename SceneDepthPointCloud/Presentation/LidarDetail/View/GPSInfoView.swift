//
//  GPSInfoView.swift
//  SceneDepthPointCloud
//
//  Created by Kang Minsang on 2023/05/09.
//  Copyright © 2023 Apple. All rights reserved.
//

import UIKit
import MapKit

final class GPSInfoView: UIView {
    private let sectionLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 16, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.text = "GPS"
        return label
    }()
    private let backgroundView: UIView = {
        let view = UIView()
        view.translatesAutoresizingMaskIntoConstraints = false
        view.layer.cornerRadius = 16
        view.layer.cornerCurve = .continuous
        view.backgroundColor = UIColor(named: "cellBackgroundColor")
        return view
    }()
    private let gpsIcon: UIImageView = {
        let imageView = UIImageView()
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.image = UIImage(named: "locationCircleIcon")
        imageView.contentMode = .scaleAspectFit
        NSLayoutConstraint.activate([
            imageView.widthAnchor.constraint(equalToConstant: 36),
            imageView.heightAnchor.constraint(equalToConstant: 36)
        ])
        return imageView
    }()
    private let floorLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 18, weight: .bold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let latitudeLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let longitudeLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.font = UIFont.systemFont(ofSize: 14, weight: .semibold)
        label.textColor = .white
        label.textAlignment = .left
        label.numberOfLines = 1
        return label
    }()
    private let tagView = LidarDetailTagView(tag: .apple)
    private let mapView: MKMapView = {
        let mapView = MKMapView()
        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.layer.cornerRadius = 8
        mapView.layer.cornerCurve = .continuous
        
        mapView.isScrollEnabled = false
        mapView.isZoomEnabled = false
        mapView.isPitchEnabled = false
        mapView.isRotateEnabled = false
        
        return mapView
    }()
    let centerPinIcon: UIImageView = {
        let imageView = UIImageView()
        imageView.translatesAutoresizingMaskIntoConstraints = false
        imageView.image = UIImage(named: "pin")
        imageView.contentMode = .scaleAspectFit
        
        NSLayoutConstraint.activate([
            imageView.widthAnchor.constraint(equalToConstant: 14),
            imageView.heightAnchor.constraint(equalToConstant: 14)
        ])
        
        return imageView
    }()
    
    convenience init() {
        self.init(frame: CGRect())
        self.configure()
    }
    
    private func configure() {
        self.translatesAutoresizingMaskIntoConstraints = false
        self.addSubview(self.sectionLabel)
        NSLayoutConstraint.activate([
            self.sectionLabel.topAnchor.constraint(equalTo: self.topAnchor),
            self.sectionLabel.leadingAnchor.constraint(equalTo: self.leadingAnchor, constant: 16)
        ])
        
        self.addSubview(self.backgroundView)
        NSLayoutConstraint.activate([
            self.backgroundView.topAnchor.constraint(equalTo: self.sectionLabel.bottomAnchor, constant: 4),
            self.backgroundView.leadingAnchor.constraint(equalTo: self.leadingAnchor),
            self.backgroundView.trailingAnchor.constraint(equalTo: self.trailingAnchor),
            self.backgroundView.heightAnchor.constraint(equalToConstant: 300),
            self.backgroundView.bottomAnchor.constraint(equalTo: self.bottomAnchor)
        ])
        
        self.backgroundView.addSubview(self.gpsIcon)
        NSLayoutConstraint.activate([
            self.gpsIcon.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.gpsIcon.leadingAnchor.constraint(equalTo: self.backgroundView.leadingAnchor, constant: 16)
        ])
        
        self.backgroundView.addSubview(self.floorLabel)
        NSLayoutConstraint.activate([
            self.floorLabel.topAnchor.constraint(equalTo: self.backgroundView.topAnchor, constant: 12),
            self.floorLabel.leadingAnchor.constraint(equalTo: self.gpsIcon.trailingAnchor, constant: 16),
            self.floorLabel.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16)
        ])
        
        let stackView = UIStackView(arrangedSubviews: [self.latitudeLabel, self.longitudeLabel])
        stackView.translatesAutoresizingMaskIntoConstraints = false
        stackView.axis = .horizontal
        stackView.spacing = 12
        stackView.alignment = .center
        
        self.backgroundView.addSubview(stackView)
        NSLayoutConstraint.activate([
            stackView.topAnchor.constraint(equalTo: self.floorLabel.bottomAnchor, constant: 4),
            stackView.leadingAnchor.constraint(equalTo: self.floorLabel.leadingAnchor)
        ])
        
        self.backgroundView.addSubview(self.tagView)
        NSLayoutConstraint.activate([
            self.tagView.centerYAnchor.constraint(equalTo: stackView.centerYAnchor),
            self.tagView.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16)
        ])
        
        self.backgroundView.addSubview(self.mapView)
        NSLayoutConstraint.activate([
            self.mapView.topAnchor.constraint(equalTo: stackView.bottomAnchor, constant: 12),
            self.mapView.leadingAnchor.constraint(equalTo: self.backgroundView.leadingAnchor, constant: 16),
            self.mapView.trailingAnchor.constraint(equalTo: self.backgroundView.trailingAnchor, constant: -16),
            self.mapView.bottomAnchor.constraint(equalTo: self.backgroundView.bottomAnchor, constant: -16)
        ])
        
        self.mapView.addSubview(self.centerPinIcon)
        NSLayoutConstraint.activate([
            self.centerPinIcon.centerXAnchor.constraint(equalTo: self.mapView.centerXAnchor),
            self.centerPinIcon.centerYAnchor.constraint(equalTo: self.mapView.centerYAnchor, constant: -7)
        ])
    }
    
    func configure(info: LidarDetailInfo) {
        let region = MKCoordinateRegion(center: CLLocationCoordinate2D(latitude: info.latitude, longitude: info.longitude), span: .init(latitudeDelta: 0.002, longitudeDelta: 0.002))
        self.mapView.setRegion(region, animated: false)
        
        let floor = info.floor >= 0 ? "F\(info.floor)" : "B\(-info.floor)"
        self.floorLabel.text = floor
        if let altitude = info.altitude {
            self.floorLabel.text = floor + " [ \(String(format: "%.1f m", altitude)) ]"
        }
        self.latitudeLabel.text = info.latitude.latitudeToDMS
        self.longitudeLabel.text = info.longitude.longitudeToDMS
    }
}
