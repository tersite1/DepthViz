import Combine

final class ScanInfoRowEventListener: ObservableObject {
    @Published var selectedLidarFileName: String? = nil
    @Published var selectedProject: String? = nil
}
