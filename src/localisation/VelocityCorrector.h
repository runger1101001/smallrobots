#pragma once

#include <Arduino.h>
#include "motion/DifferentialKinematics.h"
#include "localisation/DeadReckoning.h"
#include "localisation/Odometry.h"
#include "utils/PID.h"

namespace SmallRobots {

/**
 * VelocityCorrector
 *
 * Closes an outer velocity loop between dead reckoning (commanded velocities)
 * and odometry (actual encoder velocities) using per-wheel PIDs and an optional
 * pose-level angular PID.
 *
 * Call run() at the same cadence as DeadReckoning and Odometry (default 10 ms).
 *
 * Corrections are sent via DifferentialKinematics::applyVelocityBias(), which
 * queues corrected wheel velocities to the motor driver without modifying the
 * dead reckoning set-velocity reference.
 *
 * Example usage:
 *   VelocityCorrector corrector(kinematics, deadReckoning, odometry);
 *   corrector.setWheelPIDGains(0.5f, 0.1f, 0.0f);
 *   corrector.enablePoseCorrection(true);
 *   corrector.setup();
 *   // in loop:
 *   corrector.run();
 */
class VelocityCorrector {
public:
    VelocityCorrector(DifferentialKinematics& kinematics,
                      DeadReckoning&          dead_reckoning,
                      Odometry&               odometry);
    ~VelocityCorrector() = default;

    void setup();
    void run();

    /**
     * Reset all PID integrators and derivative state.
     * Call this whenever either pose is snapped to an external reference
     * (e.g. camera ground truth, resetDeadReckoning) to prevent stale
     * integral windup from driving incorrect corrections.
     */
    void reset();

    // ------------------------------------------------------------------
    // Tuning API
    // ------------------------------------------------------------------

    /** Apply the same P/I/D gains to both wheel PIDs. */
    void setWheelPIDGains(float p, float i, float d);

    /** Per-wheel gain tuning (useful for asymmetric motors). */
    void setLeftWheelPIDGains(float p, float i, float d);
    void setRightWheelPIDGains(float p, float i, float d);

    /**
     * Pose-level angular PID.
     * Error = deadReckoning.angle − odometry.angle.
     * Correction is distributed symmetrically: left -= bias, right += bias.
     */
    void setPosePIDGains(float p, float i, float d);

    /** Temporarily enable / disable the per-wheel velocity correction layer.
     *  Does NOT update the init snapshot — use enableWheelCorrectionPID() for that. */
    void enableWheelCorrection(bool enable);

    /** Temporarily enable / disable the pose-level angular correction layer.
     *  Does NOT update the init snapshot — use enablePoseCorrectionPID() for that. */
    void enablePoseCorrection(bool enable);

    /** Set the per-wheel correction on/off as the baseline (also updates the init
     *  snapshot so restoreEnabled() returns to this value after e.g. unstucking). */
    void enableWheelCorrectionPID(bool enable);

    /** Set the pose correction on/off as the baseline (also updates the init
     *  snapshot so restoreEnabled() returns to this value after e.g. unstucking). */
    void enablePoseCorrectionPID(bool enable);

    /**
     * Restore wheel and pose correction to the enabled state that was active
     * when setup() was called. Use this after a temporary disable (e.g. unstucking).
     */
    void restoreEnabled();

    /**
     * Clamp the maximum per-wheel velocity correction in rad/s.
     * Default: 2.0 rad/s.
     */
    void setWheelOutputLimit(float limit_rad_s);

    /**
     * Clamp the maximum pose angular correction contribution in rad/s.
     * Default: 1.0 rad/s.
     */
    void setPoseOutputLimit(float limit_rad_s);

    /** Anti-windup integral clamp applied to both wheel PIDs. */
    void setWheelIntegralLimit(float limit);

    /** Per-wheel integral clamp (useful for asymmetric motors). */
    void setLeftWheelIntegralLimit(float limit);
    void setRightWheelIntegralLimit(float limit);

    /** Anti-windup integral clamp for the pose PID. */
    void setPoseIntegralLimit(float limit);

    // ------------------------------------------------------------------
    // Inspection
    // ------------------------------------------------------------------
    float getLastLeftCorrection()  const { return _last_left_corr; }
    float getLastRightCorrection() const { return _last_right_corr; }
    float getLastPoseCorrection()  const { return _last_pose_corr; }

    PID left_pid;
    PID right_pid;
    PID pose_pid;

private:
    DifferentialKinematics& _kinematics;
    DeadReckoning&          _dead_reckoning;
    Odometry&               _odometry;

    unsigned long _last_time          = 0;
    uint32_t      _update_ms          = 10;
    bool          _wheel_enabled      = false;
    bool          _pose_enabled       = false;
    bool          _wheel_enabled_init = false;   // snapshot taken in setup()
    bool          _pose_enabled_init  = false;  // snapshot taken in setup()

    float _last_left_corr  = 0.0f;
    float _last_right_corr = 0.0f;
    float _last_pose_corr  = 0.0f;
};

} // namespace SmallRobots
