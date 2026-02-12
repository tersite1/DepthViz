//
//  AddMarkerView.swift
//  DepthViz
//
//  새 마커 추가 화면 (SwiftUI)
//

import SwiftUI
import CoreLocation

struct AddMarkerView: View {
    @Environment(\.dismiss) private var dismiss
    
    @State private var name: String = ""
    @State private var description: String = ""
    @State private var latitude: String = ""
    @State private var longitude: String = ""
    @State private var address: String = ""
    
    @State private var showAlert = false
    @State private var alertMessage = ""
    
    let onSave: (LocationMarker) -> Void
    
    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Basic Information")) {
                    TextField("Name", text: $name)
                        .autocapitalization(.words)
                    
                    TextField("Description", text: $description)
                        .autocapitalization(.sentences)
                }
                
                Section(header: Text("Location")) {
                    HStack {
                        Text("Latitude")
                            .frame(width: 80, alignment: .leading)
                        TextField("37.5665", text: $latitude)
                            .keyboardType(.decimalPad)
                    }
                    
                    HStack {
                        Text("Longitude")
                            .frame(width: 80, alignment: .leading)
                        TextField("126.9780", text: $longitude)
                            .keyboardType(.decimalPad)
                    }
                    
                    TextField("Address (Optional)", text: $address)
                        .autocapitalization(.words)
                }
                
                Section(header: Text("Tips")) {
                    VStack(alignment: .leading, spacing: 8) {
                        Label("Long press on map to get coordinates", systemImage: "hand.tap.fill")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        
                        Label("Or use current location from GPS", systemImage: "location.fill")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
            }
            .navigationTitle("Add Marker")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarLeading) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Save") {
                        saveMarker()
                    }
                    .fontWeight(.semibold)
                    .disabled(!isValid)
                }
            }
            .alert("Error", isPresented: $showAlert) {
                Button("OK", role: .cancel) { }
            } message: {
                Text(alertMessage)
            }
        }
    }
    
    private var isValid: Bool {
        !name.isEmpty &&
        !description.isEmpty &&
        !latitude.isEmpty &&
        !longitude.isEmpty &&
        Double(latitude) != nil &&
        Double(longitude) != nil
    }
    
    private func saveMarker() {
        guard let lat = Double(latitude),
              let lon = Double(longitude) else {
            alertMessage = "Invalid coordinates. Please enter valid numbers."
            showAlert = true
            return
        }
        
        // 좌표 범위 검증
        guard lat >= -90 && lat <= 90 else {
            alertMessage = "Latitude must be between -90 and 90."
            showAlert = true
            return
        }
        
        guard lon >= -180 && lon <= 180 else {
            alertMessage = "Longitude must be between -180 and 180."
            showAlert = true
            return
        }
        
        let marker = LocationMarker(
            name: name,
            description: description,
            latitude: lat,
            longitude: lon,
            address: address.isEmpty ? nil : address
        )
        
        onSave(marker)
        dismiss()
    }
}

#Preview {
    AddMarkerView { marker in
        print("Saved: \(marker.name)")
    }
}

