#include "PIDAutoTune.h"
#include <cmath>

// use_unstuck_behaviour lives in the firmware globals
extern bool use_unstuck_behaviour;

namespace SmallRobots {

// Out-of-line definitions required for ODR with constexpr arrays (C++11/14)
constexpr float PIDAutoTune::WHEEL_I_CANDIDATES[];
constexpr float PIDAutoTune::POSE_I_CANDIDATES[];

// -------------------------------------------------------------------------

PIDAutoTune::PIDAutoTune(VelocityCorrector&      corrector,
                         DifferentialKinematics& kinematics,
                         DeadReckoning&          dead_reckoning,
                         Odometry&               odometry)
    : _corrector(corrector)
    , _kinematics(kinematics)
    , _dead_reckoning(dead_reckoning)
    , _odometry(odometry)
{}

// -------------------------------------------------------------------------
// Public
// -------------------------------------------------------------------------

void PIDAutoTune::begin() {
    Serial.println("[AutoTune] *** Starting PID autotune (square trajectory) ***");
    Serial.printf("[AutoTune] Phase 1: %d wheel-I candidates  Phase 2: %d pose-I candidates\n",
                  N_WHEEL, N_POSE);
    Serial.printf("[AutoTune] Segment: %lu ms  Pause: %lu ms  Settle: %lu ms\n",
                  (unsigned long)_segment_ms, (unsigned long)_pause_ms, (unsigned long)_settle_ms);

    // Disable stuck detector — its false positives are exactly what we're tuning away
    _saved_unstuck    = use_unstuck_behaviour;
    use_unstuck_behaviour = false;

    _wheel_idx        = 0;
    _pose_idx         = 0;
    _in_phase2        = false;
    _best_wheel_score = 1e9f;
    _best_pose_score  = 1e9f;
    _best_wheel_i     = 0.0f;
    _best_pose_i      = 0.0f;

    applyGains(WHEEL_I_CANDIDATES[0], 0.0f);

    _kinematics.stop();
    _state = AT_SETTLING;
    _ts    = millis();
}

void PIDAutoTune::run() {
    switch (_state) {

        // -----------------------------------------------------------
        case AT_SETTLING: {
            if (millis() - _ts < _settle_ms) return;
            // Zero both estimates so scoring starts from a clean origin
            _dead_reckoning.resetCurPose();
            _odometry.resetCurPose();
            _corrector.reset();
            startTrial();
            break;
        }

        // -----------------------------------------------------------
        case AT_SEGMENT: {
            accumulateScore();
            if (millis() - _ts >= _segment_ms) {
                _kinematics.stop();
                _segment_idx++;
                _state = AT_SEGMENT_PAUSE;
                _ts    = millis();
            }
            break;
        }

        // -----------------------------------------------------------
        case AT_SEGMENT_PAUSE: {
            if (millis() - _ts < _pause_ms) return;
            if (_segment_idx >= N_SEGMENTS) {
                _state = AT_TRIAL_DONE;
            } else {
                startSegment();
            }
            break;
        }

        // -----------------------------------------------------------
        case AT_TRIAL_DONE: {
            finishTrial();
            break;
        }

        // -----------------------------------------------------------
        case AT_PHASE2_START: {
            Serial.printf("[AutoTune] === Phase 2 start — wheel_i locked at %.3f ===\n",
                          _best_wheel_i);
            _in_phase2 = true;
            _pose_idx  = 0;
            applyGains(_best_wheel_i, POSE_I_CANDIDATES[0]);
            _kinematics.stop();
            _state = AT_SETTLING;
            _ts    = millis();
            break;
        }

        case AT_IDLE:
        case AT_DONE:
        default:
            break;
    }
}

// -------------------------------------------------------------------------
// Private helpers
// -------------------------------------------------------------------------

void PIDAutoTune::startTrial() {
    _score_acc   = 0.0f;
    _score_count = 0;
    _segment_idx = 0;
    startSegment();
}

void PIDAutoTune::startSegment() {
    // Square: even indices = straight forward, odd indices = rotate CW 90°
    if (_segment_idx % 2 == 0) {
        _kinematics.move(_trial_speed);
    } else {
        _kinematics.rotateCW(_rotate_speed);
    }
    _state = AT_SEGMENT;
    _ts    = millis();
}

void PIDAutoTune::accumulateScore() {
    Pose dr  = _dead_reckoning.getCurPose();
    Pose odo = _odometry.getCurPose();

    // Position drift between DR and odometry
    float dx   = dr.x - odo.x;
    float dy   = dr.y - odo.y;
    float dpos = sqrtf(dx*dx + dy*dy);  // mm

    // Heading drift, normalised to [-π, π]
    float da     = dr.angle - odo.angle;
    da           = atan2f(sinf(da), cosf(da));
    float dangle = fabsf(da);  // rad

    // Scale: 1 rad ≈ 100 mm so angle contributes comparably to position
    _score_acc   += dpos + dangle * 100.0f;
    _score_count += 1;
}

float PIDAutoTune::computeFinalScore() const {
    if (_score_count == 0) return 1e9f;
    return _score_acc / static_cast<float>(_score_count);
}

void PIDAutoTune::applyGains(float wheel_i, float pose_i) {
    _corrector.setWheelPIDGains(0.0f, wheel_i, 0.0f);
    _corrector.setPosePIDGains(0.0f, pose_i, 0.0f);
    _corrector.enableWheelCorrection(wheel_i > 0.0f);
    _corrector.enablePoseCorrection(pose_i   > 0.0f);
    _corrector.reset();
}

void PIDAutoTune::finishTrial() {
    float score = computeFinalScore();

    if (!_in_phase2) {
        // ---- Phase 1: wheel-I sweep ----
        float wi = WHEEL_I_CANDIDATES[_wheel_idx];
        Serial.printf("[AutoTune] P1 [%d/%d] wheel_i=%.3f  score=%.2f%s\n",
                      _wheel_idx + 1, N_WHEEL, wi, score,
                      (score < _best_wheel_score) ? "  <-- best" : "");

        if (score < _best_wheel_score) {
            _best_wheel_score = score;
            _best_wheel_i     = wi;
        }

        _wheel_idx++;
        if (_wheel_idx >= N_WHEEL) {
            Serial.printf("[AutoTune] Phase 1 complete. Best wheel_i=%.3f (score=%.2f)\n",
                          _best_wheel_i, _best_wheel_score);
            _state = AT_PHASE2_START;
            return;
        }
        applyGains(WHEEL_I_CANDIDATES[_wheel_idx], 0.0f);

    } else {
        // ---- Phase 2: pose-I sweep ----
        float pi = POSE_I_CANDIDATES[_pose_idx];
        Serial.printf("[AutoTune] P2 [%d/%d] pose_i=%.3f  score=%.2f%s\n",
                      _pose_idx + 1, N_POSE, pi, score,
                      (score < _best_pose_score) ? "  <-- best" : "");

        if (score < _best_pose_score) {
            _best_pose_score = score;
            _best_pose_i     = pi;
        }

        _pose_idx++;
        if (_pose_idx >= N_POSE) {
            Serial.printf("[AutoTune] *** Complete. Best wheel_i=%.3f  pose_i=%.3f ***\n",
                          _best_wheel_i, _best_pose_i);
            applyGains(_best_wheel_i, _best_pose_i);
            _kinematics.stop();
            use_unstuck_behaviour = _saved_unstuck;
            _state = AT_DONE;
            return;
        }
        applyGains(_best_wheel_i, POSE_I_CANDIDATES[_pose_idx]);
    }

    // Settle before next trial
    _kinematics.stop();
    _state = AT_SETTLING;
    _ts    = millis();
}

} // namespace SmallRobots
