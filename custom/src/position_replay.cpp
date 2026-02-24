#include "../custom/include/position_replay.h"
#include "../include/motor-control.h"
#include "../custom/include/robot-config.h"
#include <cmath>
#include <cstdio>

// Global instance
PositionReplay positionReplay;

// Guard flag for playback vs driver ownership
bool is_replaying = false;

// ==================== Helper Functions ====================

uint8_t PositionReplay::packButtons() {
    uint8_t buttons = 0;
    if (controller_1.ButtonX.pressing()) buttons |= (1 << BTN_MIDSCORING);
    if (controller_1.ButtonR2.pressing()) buttons |= (1 << BTN_DESCORE);
    if (controller_1.ButtonL2.pressing()) buttons |= (1 << BTN_UNLOADER);
    return buttons;
}

bool PositionReplay::wasPressed(uint8_t current, uint8_t prev, uint8_t bit) {
    return (current & (1 << bit)) && !(prev & (1 << bit));
}

bool PositionReplay::isSDCardInserted() const {
    return Brain.SDcard.isInserted();
}

size_t PositionReplay::findFrameIndexAtTime(uint64_t elapsedMicros) {
    if (recording.empty()) return 0;
    if (elapsedMicros >= recording.back().timestamp) return recording.size() - 1;

    // Binary search
    size_t lo = 0, hi = recording.size() - 1;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2;
        if (recording[mid].timestamp < elapsedMicros) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

// ==================== Recording ====================

void PositionReplay::startRecording() {
    if (is_replaying) {
        controller_1.rumble("---");
        return; // Don't record while playing back!
    }

    if (!isSDCardInserted()) {
        controller_1.Screen.clearLine(1);
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("NO SD CARD!");
        controller_1.rumble("---");
        wait(1000, msec);
    }

    recording.clear();
    try {
        recording.reserve(2000);
    } catch (...) {
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("MEM ERROR!");
        controller_1.rumble("---");
    }

    // Reset Odometry exactly to 0
    x_pos = 0;
    y_pos = 0;
    correct_angle = 0;
    inertial_sensor.setRotation(0, degrees);

    recordStartTime = Brain.Timer.systemHighResolution(); // micros
    lastRecordTime = 0;
    _isRecording = true;
    prevButtons = packButtons();

    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("RECORDING... ");
    controller_1.rumble("-");
}

void PositionReplay::stopRecording(bool save) {
    _isRecording = false;

    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("STOP  %d pts ", recording.size());
    controller_1.rumble(".");

    if (save) {
        if (!isSDCardInserted()) {
            controller_1.Screen.setCursor(2, 1);
            controller_1.Screen.print("NO SD CARD");
            controller_1.rumble("---");
        } else if (this->saveToSD()) {
            controller_1.Screen.setCursor(2, 1);
            controller_1.Screen.print("SAVED TO SD! ");
        } else {
            controller_1.Screen.setCursor(2, 1);
            controller_1.Screen.print("SAVE FAILED! ");
            controller_1.rumble("--");
        }
    }
}

void PositionReplay::recordFrame() {
    if (!_isRecording) return;

    uint64_t currentTime = Brain.Timer.systemHighResolution() - recordStartTime;
    uint64_t currentTimeMs = currentTime / 1000;

    if ((currentTime - lastRecordTime) / 1000 < recordingIntervalMs) {
        return;
    }
    lastRecordTime = currentTime;

    // Get current button state
    uint8_t currentButtons = packButtons();

    // Action check
    bool actionOccurred = false;
    if (wasPressed(currentButtons, prevButtons, BTN_MIDSCORING) || 
        wasPressed(currentButtons, prevButtons, BTN_UNLOADER) ||
        (currentButtons ^ prevButtons) & (1 << BTN_DESCORE)) {
        actionOccurred = true;
    }

    float iVolt = intake_motor.voltage(volt);
    float oVolt = outtake_motor.voltage(volt);

    if (std::abs(iVolt) > 1.0 || std::abs(oVolt) > 1.0) {
        actionOccurred = true;
    }

    WaypointFrame frame;
    frame.x = x_pos;
    frame.y = y_pos;
    frame.theta = getInertialHeading();
    frame.timestamp = currentTime;
    frame.intakeVoltage = iVolt;
    frame.outtakeVoltage = oVolt;
    frame.buttons = currentButtons;
    frame.hasAction = actionOccurred;

    prevButtons = currentButtons;

    if (recording.size() >= MAX_FRAMES) {
        stopRecording(true);
        return;
    }
    recording.push_back(frame);
}

// ==================== Playback ====================

void PositionReplay::abortPlayback() {
    _abortRequested = true;
}

void PositionReplay::playback() {
    if (_isRecording) {
        controller_1.rumble("---");
        return;
    }

    if (recording.empty()) {
        if (!loadFromSD()) {
            controller_1.Screen.setCursor(1, 1);
            controller_1.Screen.print("NO REC DATA! ");
            controller_1.rumble("-");
            return;
        }
    }

    _abortRequested = false;

    // Ownership: Take over driver loop
    is_replaying = true;

    // Make sure pistons start same as initialize
    midscoring_piston.set(false);
    descore_piston.set(true);
    unloader_piston.set(false);

    bool midScoring = false;
    bool descore = false; // descore flag represents boolean state for toggles, though it's hold
    bool unloader = false;

    // Reset odometry
    x_pos = 0;
    y_pos = 0;
    correct_angle = 0;
    inertial_sensor.setRotation(0, degrees);
    wait(50, msec);

    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("REPLAYING...");

    lastPlaybackButtons = recording.empty() ? 0 : recording[0].buttons;
    prevDistanceError = 0;
    prevHeadingError = 0;
    smoothedDistanceDerivative = 0;
    smoothedHeadingDerivative = 0;
    lastLoopTimeMs = Brain.Timer.system();

    uint64_t startTimeMicros = Brain.Timer.systemHighResolution();
    uint64_t totalDurationMicros = recording.back().timestamp;

    while (!_abortRequested && !(controller_1.ButtonUp.pressing() && controller_1.ButtonDown.pressing())) {
        uint64_t elapsedMicros = Brain.Timer.systemHighResolution() - startTimeMicros;
        if (elapsedMicros >= totalDurationMicros) break;

        // Apply time-shifted lookahead
        uint64_t targetTimeMicros = elapsedMicros + (lookaheadOffsetMs * 1000.0f);
        size_t idx = findFrameIndexAtTime(targetTimeMicros);
        const WaypointFrame &target = recording[idx];

        float currentHeading = getInertialHeading();
        float dx = target.x - x_pos;
        float dy = target.y - y_pos;
        float distance = std::sqrt(dx * dx + dy * dy);

        // Compass heading: 0 is +Y, 90 is +X (CW+)
        // atan2(dx, dy) where dx is x-axis (East), dy is y-axis (North)
        float targetHeading = std::atan2(dx, dy) * 180.0f / M_PI;

        float headingError = targetHeading - currentHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        // PID parameters configuration based on lemlib values
        float kP_forward = 5.0f;
        float kD_forward = 8.0f;
        float kP_turn = 1.7f;
        float kD_turn = 14.0f;

        uint32_t nowMs = Brain.Timer.system();
        float dt = (nowMs - lastLoopTimeMs);
        if (dt <= 0) dt = 1;

        float rawDistanceDeriv = (distance - prevDistanceError) / dt;
        float rawHeadingDeriv = (headingError - prevHeadingError) / dt;

        // Low-pass filter for derivative
        smoothedDistanceDerivative = 0.7f * smoothedDistanceDerivative + 0.3f * rawDistanceDeriv;
        smoothedHeadingDerivative = 0.7f * smoothedHeadingDerivative + 0.3f * rawHeadingDeriv;

        float forwardCmd = (distance * kP_forward) + (smoothedDistanceDerivative * kD_forward * 20.0f); // scaling for dt
        float turnCmd = (headingError * kP_turn) + (smoothedHeadingDerivative * kD_turn * 20.0f);

        prevDistanceError = distance;
        prevHeadingError = headingError;
        lastLoopTimeMs = nowMs;

        // Check if driving backward
        float headingRad = currentHeading * M_PI / 180.0f;
        float forwardDotProduct = dx * std::sin(headingRad) + dy * std::cos(headingRad);
        if (forwardDotProduct < 0) {
            forwardCmd = -forwardCmd;
        }

        // Clamp 
        if (forwardCmd > 12.0f) forwardCmd = 12.0f;
        if (forwardCmd < -12.0f) forwardCmd = -12.0f;
        if (turnCmd > 12.0f) turnCmd = 12.0f;
        if (turnCmd < -12.0f) turnCmd = -12.0f;

        // Very close -> match heading directly
        if (distance < 0.5f) {
            forwardCmd = 0;
            float thetaError = target.theta - currentHeading;
            while (thetaError > 180) thetaError -= 360;
            while (thetaError < -180) thetaError += 360;

            float rawThetaDeriv = (thetaError - prevHeadingError) / dt;
            smoothedHeadingDerivative = 0.7f * smoothedHeadingDerivative + 0.3f * rawThetaDeriv;
            turnCmd = (thetaError * kP_turn) + (smoothedHeadingDerivative * kD_turn * 20.0f);
            prevHeadingError = thetaError;

            if (turnCmd > 12.0f) turnCmd = 12.0f;
            if (turnCmd < -12.0f) turnCmd = -12.0f;
        }

        double left_power = forwardCmd + turnCmd;
        double right_power = forwardCmd - turnCmd;
        
        // Output to motors
        driveChassis(left_power, right_power);

        // Mechanisms
        intake_motor.spin(fwd, target.intakeVoltage, volt);
        outtake_motor.spin(fwd, target.outtakeVoltage, volt);

        // Buttons
        if (wasPressed(target.buttons, lastPlaybackButtons, BTN_MIDSCORING)) {
            midScoring = !midScoring;
            midscoring_piston.set(midScoring);
        }
        
        // Descore is held in user.cpp: descore_piston.set(!r2_current);
        bool r2_down = (target.buttons & (1 << BTN_DESCORE));
        descore_piston.set(!r2_down);

        if (wasPressed(target.buttons, lastPlaybackButtons, BTN_UNLOADER)) {
            unloader = !unloader;
            unloader_piston.set(unloader);
        }

        lastPlaybackButtons = target.buttons;

        wait(20, msec);
    }

    driveChassis(0, 0);
    intake_motor.stop(coast);
    outtake_motor.stop(coast);

    is_replaying = false; // Give back ownership
    
    if (Competition.isDisabled()) {
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("GAME DISABLED ");
    } else if (_abortRequested || (controller_1.ButtonUp.pressing() && controller_1.ButtonDown.pressing())) {
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("REPLAY ABORTED");
    } else {
        controller_1.Screen.setCursor(1, 1);
        controller_1.Screen.print("REPLAY DONE   ");
    }
}

// ==================== Data Management ====================

void PositionReplay::clearRecording() {
    recording.clear();
}

uint32_t PositionReplay::getDuration() const {
    if (recording.empty()) return 0;
    return recording.back().timestamp / 1000;
}

bool PositionReplay::saveToSD() {
    if (!isSDCardInserted()) return false;
    
    FILE *file = fopen(filePath.c_str(), "wb");
    if (!file) return false;

    uint32_t magic = 0x504F5352;
    uint32_t version = 1;
    uint32_t frameCount = recording.size();

    fwrite(&magic, sizeof(uint32_t), 1, file);
    fwrite(&version, sizeof(uint32_t), 1, file);
    fwrite(&frameCount, sizeof(uint32_t), 1, file);

    for (const auto &frame : recording) {
        fwrite(&frame, sizeof(WaypointFrame), 1, file);
    }
    
    fclose(file);
    return true;
}

bool PositionReplay::loadFromSD() {
    if (!isSDCardInserted()) return false;

    FILE *file = fopen(filePath.c_str(), "rb");
    if (!file) return false;

    uint32_t magic, version, frameCount;
    if (fread(&magic, sizeof(uint32_t), 1, file) != 1 ||
        fread(&version, sizeof(uint32_t), 1, file) != 1 ||
        fread(&frameCount, sizeof(uint32_t), 1, file) != 1) {
        fclose(file);
        return false;
    }

    if (magic != 0x504F5352) {
        fclose(file);
        return false;
    }

    if (frameCount > MAX_FRAMES) {
        fclose(file);
        return false;
    }

    recording.clear();
    recording.resize(frameCount);
    for (uint32_t i = 0; i < frameCount; i++) {
        if (fread(&recording[i], sizeof(WaypointFrame), 1, file) != 1) {
            fclose(file);
            recording.clear();
            return false;
        }
    }

    fclose(file);
    return true;
}
