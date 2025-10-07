import UIKit

class FrameGalleryVC: UIViewController, UICollectionViewDelegate, UICollectionViewDataSource {
    private var frames: [AccumulatedFrameData.Metadata] = []
    private var images: [UIImage] = []
    private let cellReuseIdentifier = "FrameCell"
    
    private lazy var collectionView: UICollectionView = {
        let layout = UICollectionViewFlowLayout()
        layout.itemSize = CGSize(width: 100, height: 120)
        layout.sectionInset = UIEdgeInsets(top: 10, left: 10, bottom: 10, right: 10)
        let cv = UICollectionView(frame: .zero, collectionViewLayout: layout)
        cv.delegate = self
        cv.dataSource = self
        cv.register(FrameCell.self, forCellWithReuseIdentifier: cellReuseIdentifier)
        cv.backgroundColor = .systemBackground
        return cv
    }()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        view.addSubview(collectionView)
        collectionView.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            collectionView.topAnchor.constraint(equalTo: view.topAnchor),
            collectionView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            collectionView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            collectionView.trailingAnchor.constraint(equalTo: view.trailingAnchor)
        ])
        loadFrames()
    }
    
    private func loadFrames() {
        // Load all metadata and images from AccumulatedFrames directory
        let documentsURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        let framesDir = documentsURL.appendingPathComponent("AccumulatedFrames", isDirectory: true)
        guard let files = try? FileManager.default.contentsOfDirectory(at: framesDir, includingPropertiesForKeys: nil) else { return }
        let jsonFiles = files.filter { $0.pathExtension == "json" }
        for jsonURL in jsonFiles.sorted(by: { $0.lastPathComponent < $1.lastPathComponent }) {
            if let data = try? Data(contentsOf: jsonURL),
               let metadata = try? JSONDecoder().decode(AccumulatedFrameData.Metadata.self, from: data) {
                frames.append(metadata)
                let imageURL = framesDir.appendingPathComponent(metadata.imageFileName)
                if let imageData = try? Data(contentsOf: imageURL), let image = UIImage(data: imageData) {
                    images.append(image)
                } else {
                    images.append(UIImage())
                }
            }
        }
        collectionView.reloadData()
    }
    
    // MARK: UICollectionViewDataSource
    func collectionView(_ collectionView: UICollectionView, numberOfItemsInSection section: Int) -> Int {
        return frames.count
    }
    
    func collectionView(_ collectionView: UICollectionView, cellForItemAt indexPath: IndexPath) -> UICollectionViewCell {
        let cell = collectionView.dequeueReusableCell(withReuseIdentifier: cellReuseIdentifier, for: indexPath) as! FrameCell
        cell.imageView.image = images[indexPath.item]
        cell.label.text = "Frame \(frames[indexPath.item].index)"
        return cell
    }
    
    func collectionView(_ collectionView: UICollectionView, didSelectItemAt indexPath: IndexPath) {
        let detailVC = FrameDetailVC()
        detailVC.metadata = frames[indexPath.item]
        detailVC.image = images[indexPath.item]
        navigationController?.pushViewController(detailVC, animated: true)
    }
}

class FrameCell: UICollectionViewCell {
    let imageView = UIImageView()
    let label = UILabel()
    override init(frame: CGRect) {
        super.init(frame: frame)
        contentView.addSubview(imageView)
        contentView.addSubview(label)
        imageView.translatesAutoresizingMaskIntoConstraints = false
        label.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            imageView.topAnchor.constraint(equalTo: contentView.topAnchor),
            imageView.leadingAnchor.constraint(equalTo: contentView.leadingAnchor),
            imageView.trailingAnchor.constraint(equalTo: contentView.trailingAnchor),
            imageView.heightAnchor.constraint(equalToConstant: 100),
            label.topAnchor.constraint(equalTo: imageView.bottomAnchor, constant: 2),
            label.leadingAnchor.constraint(equalTo: contentView.leadingAnchor),
            label.trailingAnchor.constraint(equalTo: contentView.trailingAnchor),
            label.bottomAnchor.constraint(equalTo: contentView.bottomAnchor)
        ])
        label.textAlignment = .center
        label.font = UIFont.systemFont(ofSize: 12)
        imageView.contentMode = .scaleAspectFill
        imageView.clipsToBounds = true
    }
    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }
}
