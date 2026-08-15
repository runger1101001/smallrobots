#pragma once

#include <Arduino.h>
#include "motion/DifferentialKinematics.h"
#include "localisation/DeadReckoning.h"
#include "localisation/Odometry.h"
#include "localisation/VelocityCorrector.h"

namespace SmallRobots {

/**
 * PIDAutoTune
 *
 * Coordinate-descent sweep over wheel-I and pose-I gains using a timed
 * square trajectory (4× forward + rotate-CW). Minimises drift between
 * dead reckoning and odometry — the drift that triggers false stuck detections.
 *
 * Phase 1: sweeps wheel-I candidates with pose correction disabled.
 *          Score = average (position drift + 100×angle drift) per 10ms tick.
 * Phase 2: locks best wheel-I, sweeps pose-I candidates.
 *          Score = same metric.
 *
 * Call begin() once to start. Call run() from the main loop. When isDone(),
 * read getBestWheelI() / getBestPoseI() and save to NVS.
 *
 * The robot needs ~220×220 mm of clear floor. Total duration ≈ 2.5 min.
 * Serial prints every trial score so you can monitor progress.
 */
class PIDAutoTune {
public:
    PIDAutoTune(VelocityCorrector&      corrector,
                DifferentialKinematics& kinematics,
                DeadReckoning&          dead_reckoning,
                Odometry&               odometry);

    // ------------------------------------------------------------------
    // Configuration — call before begin()
    // ------------------------------------------------------------------

    /** Forward speed for straight segments (mm/s). Default 100. */
    void setTrialSpeed(float mm_s)       { _trial_speed  = mm_s; }

    /** Rotation speed for turn segments (mm/s). Default 80. */
    void setRotateSpeed(float mm_s)      { _rotate_speed = mm_s; }

    /** Duration of each forward/turn segment (ms). Default 1200. */
    void setSegmentDuration(uint32_t ms) { _segment_ms   = ms;   }

    /** Pause between segments (ms). Default 200. */
    void setPauseDuration(uint32_t ms)   { _pause_ms     = ms;   }

    /** Settle time before each trial starts (ms). Default 600. */
    void setSettleDuration(uint32_t ms)  { _settle_ms    = ms;   }

    // ------------------------------------------------------------------
    // Control
    // ------------------------------------------------------------------
    void begin();
    void run();

    bool isRunning() const { return _state != AT_IDLE && _state != AT_DONE; }
    bool isDone()    const { return _state == AT_DONE; }

    float getBestWheelI() const { return _best_wheel_i; }
    float getBestPoseI()  const { return _best_pose_i;  }

private:
    // Candidate gain values — extend as needed
    static constexpr float WHEEL_I_CANDIDATES[] = {0.0f, 0.05f, 0.1f, 0.2f, 0.3f, 0.5f, 0.8f};
    static constexpr float POSE_I_CANDIDATES[]  = {0.0f, 0.02f, 0.05f, 0.1f, 0.2f};
    static constexpr int   N_WHEEL    = 7;
    static constexpr int   N_POSE     = 5;
    // One trial = 4× (forward segment + rotate-CW segment) = 8 segments
    static constexpr int   N_SEGMENTS = 8;

    enum ATState {
        AT_IDLE,
        AT_SETTLING,        // motors stopped, poses reset, waiting before trial
        AT_SEGMENT,         // executing a square segment
        AT_SEGMENT_PAUSE,   // brief stop between segments
        AT_TRIAL_DONE,      // score recorded, advance to next candidate
        AT_PHASE2_START,    // finished phase 1, set up phase 2
        AT_DONE
    };

    void startTrial();
    void startSegment();
    void accumulateScore();
    float computeFinalScore() const;
    void applyGains(float wheel_i, float pose_i);
    void finishTrial();

    VelocityCorrector&      _corrector;
    DifferentialKinematics& _kinematics;
    DeadReckoning&          _dead_reckoning;
    Odometry&               _odometry;

    // Config
    float    _trial_speed  = 100.0f;
    float    _rotate_speed = 80.0f;
    uint32_t _segment_ms   = 1200;
    uint32_t _pause_ms     = 200;
    uint32_t _settle_ms    = 600;

    // State
    ATState       _state       = AT_IDLE;
    int           _segment_idx = 0;
    unsigned long _ts          = 0;

    // Phase 1 — wheel-I sweep
    int   _wheel_idx        = 0;
    float _best_wheel_i     = 0.0f;
    float _best_wheel_score = 1e9f;

    // Phase 2 — pose-I sweep
    bool  _in_phase2       = false;
    int   _pose_idx        = 0;
    float _best_pose_i     = 0.0f;
    float _best_pose_score = 1e9f;

    // Per-trial score
    float    _score_acc   = 0.0f;
    uint32_t _score_count = 0;

    // Saved state to restore after completion
    bool _saved_unstuck = false;
};

} // namespace SmallRobots
