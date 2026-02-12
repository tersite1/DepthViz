import UIKit

final class SplashViewController: UIViewController {
    private let titleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.text = "DepthViz"
        label.font = .systemFont(ofSize: 34, weight: .bold)
        label.textColor = .white
        label.textAlignment = .center
        return label
    }()

    private let subtitleLabel: UILabel = {
        let label = UILabel()
        label.translatesAutoresizingMaskIntoConstraints = false
        label.text = "Preparing scanner…"
        label.font = .systemFont(ofSize: 15, weight: .medium)
        label.textColor = UIColor.white.withAlphaComponent(0.75)
        label.textAlignment = .center
        return label
    }()

    private let progressView: UIProgressView = {
        let view = UIProgressView(progressViewStyle: .bar)
        view.translatesAutoresizingMaskIntoConstraints = false
        view.trackTintColor = UIColor.white.withAlphaComponent(0.15)
        view.progressTintColor = .systemPurple
        view.progress = 0
        return view
    }()

    private let spinner: UIActivityIndicatorView = {
        let view = UIActivityIndicatorView(style: .medium)
        view.translatesAutoresizingMaskIntoConstraints = false
        view.color = UIColor.white.withAlphaComponent(0.85)
        return view
    }()

    private var progressTimer: Timer?
    private var progress: Float = 0

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .black

        view.addSubview(titleLabel)
        view.addSubview(subtitleLabel)
        view.addSubview(progressView)
        view.addSubview(spinner)

        NSLayoutConstraint.activate([
            titleLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            titleLabel.centerYAnchor.constraint(equalTo: view.centerYAnchor, constant: -40),

            subtitleLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            subtitleLabel.topAnchor.constraint(equalTo: titleLabel.bottomAnchor, constant: 12),

            progressView.leadingAnchor.constraint(equalTo: view.leadingAnchor, constant: 44),
            progressView.trailingAnchor.constraint(equalTo: view.trailingAnchor, constant: -44),
            progressView.topAnchor.constraint(equalTo: subtitleLabel.bottomAnchor, constant: 20),

            spinner.centerXAnchor.constraint(equalTo: view.centerXAnchor),
            spinner.topAnchor.constraint(equalTo: progressView.bottomAnchor, constant: 18)
        ])

        spinner.startAnimating()
        startProgressAnimation()
    }

    override func viewDidDisappear(_ animated: Bool) {
        super.viewDidDisappear(animated)
        progressTimer?.invalidate()
        progressTimer = nil
    }

    private func startProgressAnimation() {
        progressTimer?.invalidate()
        progress = 0
        progressView.progress = 0

        progressTimer = Timer.scheduledTimer(withTimeInterval: 0.02, repeats: true) { [weak self] timer in
            guard let self else {
                timer.invalidate()
                return
            }
            self.progress = min(0.95, self.progress + 0.02)
            self.progressView.setProgress(self.progress, animated: true)
            if self.progress >= 0.95 {
                timer.invalidate()
            }
        }
    }

    func completeProgress(animated: Bool) {
        progressTimer?.invalidate()
        progressTimer = nil
        progress = 1
        progressView.setProgress(1, animated: animated)
    }
}
