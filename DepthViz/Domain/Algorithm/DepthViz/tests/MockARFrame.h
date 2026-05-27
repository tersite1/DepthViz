#ifndef MOCK_AR_FRAME_H
#define MOCK_AR_FRAME_H

#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace MockARKit {

struct MockLiDARData {
    std::vector<float> xyz;
    std::vector<uint8_t> confidence;
    int pointCount;
    double timestamp;
    
    MockLiDARData() : pointCount(0), timestamp(0.0) {}
};

class MockARFrame {
public:
    enum NoiseProfile {
        CLEAN,           
        LIGHT_NOISE,     
        MODERATE_NOISE,  
        HEAVY_NOISE      
    };
    
    enum ScanPattern {
        RANDOM,          
        GRID,            
        CIRCULAR,        
        WALL             
    };
    
    MockARFrame() 
        : pointsPerFrame_(5000),
          noiseProfile_(LIGHT_NOISE),
          scanPattern_(RANDOM),
          frameIndex_(0) {
        cameraTransform_ = Eigen::Matrix4f::Identity();
        rng_.seed(std::random_device{}());
    }
    
    void setPointsPerFrame(int count) {
        pointsPerFrame_ = count;
    }
    
    void setNoiseProfile(NoiseProfile profile) {
        noiseProfile_ = profile;
    }
    
    void setScanPattern(ScanPattern pattern) {
        scanPattern_ = pattern;
    }
    
    void setCameraTransform(const Eigen::Matrix4f& transform) {
        cameraTransform_ = transform;
    }
    
    void injectConfidenceDrop(float dropRate = 0.2f) {
        confidenceDropRate_ = dropRate;
    }
    
    void injectSystematicNoise(float noiseStd = 0.005f) {
        systematicNoiseStd_ = noiseStd;
    }
    
    MockLiDARData generateFrame() {
        MockLiDARData frame;
        frame.timestamp = getCurrentTimestamp();
        frame.pointCount = pointsPerFrame_;
        
        frame.xyz.resize(pointsPerFrame_ * 3);
        frame.confidence.resize(pointsPerFrame_);
        
        switch (scanPattern_) {
            case RANDOM:
                generateRandomPoints(frame);
                break;
            case GRID:
                generateGridPoints(frame);
                break;
            case CIRCULAR:
                generateCircularPoints(frame);
                break;
            case WALL:
                generateWallPoints(frame);
                break;
        }
        
        applyNoise(frame);
        injectMissingConfidence(frame);
        applySystematicError(frame);
        transformPointsToWorldFrame(frame);
        
        frameIndex_++;
        return frame;
    }
    
    MockLiDARData generateStaticScene(int duration_ms) {
        return generateFrame();
    }
    
    MockLiDARData generateMovingScene(float velocity_x, float velocity_y, float velocity_z) {
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        float dt = 0.033f;
        T.block<3, 1>(0, 3) = Eigen::Vector3f(
            velocity_x * dt,
            velocity_y * dt,
            velocity_z * dt
        );
        cameraTransform_ = T * cameraTransform_;
        
        return generateFrame();
    }
    
    int getFrameIndex() const { return frameIndex_; }
    Eigen::Matrix4f getCameraTransform() const { return cameraTransform_; }
    
private:
    int pointsPerFrame_;
    NoiseProfile noiseProfile_;
    ScanPattern scanPattern_;
    Eigen::Matrix4f cameraTransform_;
    int frameIndex_;
    float confidenceDropRate_;
    float systematicNoiseStd_;
    std::mt19937 rng_;
    std::normal_distribution<float> normalDist_;
    std::uniform_real_distribution<float> uniformDist_;
    
    void generateRandomPoints(MockLiDARData& frame) {
        std::uniform_real_distribution<float> xDist(0.3f, 3.0f);
        std::uniform_real_distribution<float> yzDist(-1.5f, 1.5f);
        std::uniform_int_distribution<int> confDist(0, 2);
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            frame.xyz[i * 3] = xDist(rng_);           
            frame.xyz[i * 3 + 1] = yzDist(rng_);     
            frame.xyz[i * 3 + 2] = yzDist(rng_);     
            frame.confidence[i] = confDist(rng_);
        }
    }
    
    void generateGridPoints(MockLiDARData& frame) {
        int gridSize = (int)std::sqrt(pointsPerFrame_);
        float cellSize = 3.0f / gridSize;
        
        for (int y = 0; y < gridSize && y * gridSize < pointsPerFrame_; y++) {
            for (int x = 0; x < gridSize && y * gridSize + x < pointsPerFrame_; x++) {
                int idx = y * gridSize + x;
                frame.xyz[idx * 3] = 0.5f + x * cellSize;
                frame.xyz[idx * 3 + 1] = -1.5f + y * cellSize;
                frame.xyz[idx * 3 + 2] = 0.0f;
                frame.confidence[idx] = (x + y) % 2 == 0 ? 2 : 1;
            }
        }
    }
    
    void generateCircularPoints(MockLiDARData& frame) {
        float radius = 1.0f;
        float centerX = 1.0f;
        float centerY = 0.0f;
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            float angle = (float)i / pointsPerFrame_ * 2.0f * M_PI;
            frame.xyz[i * 3] = centerX + radius * std::cos(angle);
            frame.xyz[i * 3 + 1] = centerY + radius * std::sin(angle);
            frame.xyz[i * 3 + 2] = 0.0f;
            frame.confidence[i] = (i % 3 == 0) ? 2 : 1;
        }
    }
    
    void generateWallPoints(MockLiDARData& frame) {
        float wallDistance = 1.5f;
        float wallHeight = 2.0f;
        float wallWidth = 3.0f;
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            int row = i / (int)std::sqrt(pointsPerFrame_);
            int col = i % (int)std::sqrt(pointsPerFrame_);
            
            float u = (float)col / std::sqrt(pointsPerFrame_);
            float v = (float)row / std::sqrt(pointsPerFrame_);
            
            frame.xyz[i * 3] = wallDistance;
            frame.xyz[i * 3 + 1] = -wallWidth/2.0f + u * wallWidth;
            frame.xyz[i * 3 + 2] = -wallHeight/2.0f + v * wallHeight;
            frame.confidence[i] = 2;
        }
    }
    
    void applyNoise(MockLiDARData& frame) {
        float noiseSigma = 0.0f;
        float outliersRate = 0.0f;
        
        switch (noiseProfile_) {
            case CLEAN:
                noiseSigma = 0.0f;
                outliersRate = 0.0f;
                break;
            case LIGHT_NOISE:
                noiseSigma = 0.003f;
                outliersRate = 0.01f;
                break;
            case MODERATE_NOISE:
                noiseSigma = 0.01f;
                outliersRate = 0.05f;
                break;
            case HEAVY_NOISE:
                noiseSigma = 0.02f;
                outliersRate = 0.10f;
                break;
        }
        
        std::normal_distribution<float> noiseDist(0.0f, noiseSigma);
        std::uniform_real_distribution<float> outliersUniform(0.0f, 1.0f);
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            if (outliersUniform(rng_) < outliersRate) {
                frame.xyz[i * 3] += uniformDist_(rng_, 
                    std::uniform_real_distribution<float>::param_type(-0.5f, 0.5f));
                frame.xyz[i * 3 + 1] += uniformDist_(rng_,
                    std::uniform_real_distribution<float>::param_type(-0.5f, 0.5f));
                frame.xyz[i * 3 + 2] += uniformDist_(rng_,
                    std::uniform_real_distribution<float>::param_type(-0.5f, 0.5f));
            } else {
                frame.xyz[i * 3] += noiseDist(rng_);
                frame.xyz[i * 3 + 1] += noiseDist(rng_);
                frame.xyz[i * 3 + 2] += noiseDist(rng_);
            }
        }
    }
    
    void injectMissingConfidence(MockLiDARData& frame) {
        if (confidenceDropRate_ <= 0.0f) return;
        
        std::uniform_real_distribution<float> dropUniform(0.0f, 1.0f);
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            if (dropUniform(rng_) < confidenceDropRate_) {
                frame.confidence[i] = 0;
            }
        }
    }
    
    void applySystematicError(MockLiDARData& frame) {
        if (systematicNoiseStd_ <= 0.0f) return;
        
        std::normal_distribution<float> errorDist(0.0f, systematicNoiseStd_);
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            frame.xyz[i * 3] += errorDist(rng_);
            frame.xyz[i * 3 + 1] += errorDist(rng_);
            frame.xyz[i * 3 + 2] += errorDist(rng_);
        }
    }
    
    void transformPointsToWorldFrame(MockLiDARData& frame) {
        Eigen::Matrix4f T = cameraTransform_;
        
        for (int i = 0; i < pointsPerFrame_; i++) {
            Eigen::Vector4f p(
                frame.xyz[i * 3],
                frame.xyz[i * 3 + 1],
                frame.xyz[i * 3 + 2],
                1.0f
            );
            
            Eigen::Vector4f p_world = T * p;
            
            frame.xyz[i * 3] = p_world(0);
            frame.xyz[i * 3 + 1] = p_world(1);
            frame.xyz[i * 3 + 2] = p_world(2);
        }
    }
    
    double getCurrentTimestamp() {
        static auto startTime = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(now - startTime).count();
    }
};

class MockARFrameSequence {
public:
    MockARFrameSequence(int sequenceLength)
        : sequenceLength_(sequenceLength), frameIndex_(0) {
        mockFrame_.setPointsPerFrame(5000);
    }
    
    void configureStaticScene() {
        mockFrame_.setScanPattern(MockARFrame::WALL);
        mockFrame_.setNoiseProfile(MockARFrame::LIGHT_NOISE);
    }
    
    void configureMovingScene(float velocity = 0.1f) {
        mockFrame_.setScanPattern(MockARFrame::RANDOM);
        movementVelocity_ = velocity;
    }
    
    void configureNoisyScene() {
        mockFrame_.setNoiseProfile(MockARFrame::HEAVY_NOISE);
        mockFrame_.injectConfidenceDrop(0.15f);
        mockFrame_.injectSystematicNoise(0.008f);
    }
    
    MockLiDARData getNextFrame() {
        if (movementVelocity_ > 0.0f) {
            return mockFrame_.generateMovingScene(movementVelocity_, 0.0f, 0.0f);
        } else {
            return mockFrame_.generateFrame();
        }
    }
    
    bool hasMoreFrames() const {
        return frameIndex_ < sequenceLength_;
    }
    
    int getSequenceProgress() const {
        return (frameIndex_ * 100) / sequenceLength_;
    }
    
private:
    MockARFrame mockFrame_;
    int sequenceLength_;
    int frameIndex_;
    float movementVelocity_;
};

} // namespace MockARKit

#endif // MOCK_AR_FRAME_H
