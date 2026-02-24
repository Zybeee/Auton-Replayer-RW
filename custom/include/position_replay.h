#pragma once

#include "vex.h"
#include <vector>
#include <string>

// Guard flag for playback vs driver control ownership
extern bool is_replaying;

// Packed to ensure consistent binary layout across compiler versions
#pragma pack(push, 1)
struct WaypointFrame {
    // Position data from RW-Drive odometry
    float x;           // X position in inches
    float y;           // Y position in inches
    float theta;       // Heading in degrees (compass style: 0=Fwd, CW+)

    // Timing
    uint64_t timestamp; // Time since recording started (microseconds)

    // Mechanism states (voltage representation: -12.0 to 12.0)
    float intakeVoltage;
    float outtakeVoltage;

    // Button states packed into bitflags
    // Bit 0: MidScoring (X) (toggle)
    // Bit 1: Descore (R2) (hold)
    // Bit 2: Unloader (L2) (toggle)
    uint8_t buttons;

    // Flag to indicate if mechanism action occurred at this waypoint
    bool hasAction;
};
#pragma pack(pop)

// Button bit positions
constexpr uint8_t BTN_MIDSCORING = 0;
constexpr uint8_t BTN_DESCORE = 1;
constexpr uint8_t BTN_UNLOADER = 2;

class PositionReplay {
private:
    std::vector<WaypointFrame> recording;
    uint64_t recordStartTime = 0;
    bool _isRecording = false;
    
    // Playback state
    bool _abortRequested = false;
    
    // Previous button states for edge detection during recording
    uint8_t prevButtons = 0;
    uint8_t lastPlaybackButtons = 0; // For edge detection during playback

    // Configuration
    uint32_t recordingIntervalMs = 25; // 40 samples/sec (25ms intervals)
    float lookaheadOffsetMs = 150.0f;  // Time-shifted lookahead in ms

    // Derivatives for stabilization
    float smoothedDistanceDerivative = 0;
    float smoothedHeadingDerivative = 0;
    float prevDistanceError = 0;
    float prevHeadingError = 0;
    uint32_t lastLoopTimeMs = 0;

    // File path for SD card storage
    std::string filePath = "pos_rec.bin"; 

    // Last record time for interval-based recording
    uint64_t lastRecordTime = 0;

    // Helper methods
    uint8_t packButtons();
    bool wasPressed(uint8_t current, uint8_t prev, uint8_t bit);
    size_t findFrameIndexAtTime(uint64_t elapsedMicros);

public:
    PositionReplay() = default;

    // ==================== Recording ====================
    void startRecording();
    void stopRecording(bool saveToSD = true);
    void recordFrame();

    // ==================== Playback ====================
    void playback(); 
    void abortPlayback();

    // ==================== Data Management ====================
    void clearRecording();
    bool saveToSD();
    bool loadFromSD();

    // ==================== Getters/Setters ====================
    size_t getFrameCount() const { return recording.size(); }
    static constexpr size_t MAX_FRAMES = 5000;
    uint32_t getDuration() const;

    bool isRecording() const { return _isRecording; }
    
    void setRecordingInterval(uint32_t ms) { recordingIntervalMs = ms; }
    void setLookaheadOffset(float ms) { lookaheadOffsetMs = ms; }
    void setFilePath(const std::string& path) { filePath = path; }

    bool isSDCardInserted() const;
};

// Global instance
extern PositionReplay positionReplay;
