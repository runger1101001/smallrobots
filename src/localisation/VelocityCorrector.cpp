#include "VelocityCorrector.h"
#include <cmath>

namespace SmallRobots {

VelocityCorrector::VelocityCorrector(DifferentialKinematics& kinematics,
                                     DeadReckoning&          dead_reckoning,
                                     Odometry&               odometry)
    : _kinematics(kinematics)
    , _dead_reckoning(dead_reckoning)
    , _odometry(odometry)
{

    // All gains default to 0 = no correction (passthrough). Set them in main code.
    left_pid.P  = 0.0f;  left_pid.I  = 0.0f;  left_pid.D  = 0.0f;
    left_pid.integral_limit = 3.0f;  left_pid.output_limit = 2.0f;

    right_pid.P = 0.0f;  right_pid.I = 0.0f;  right_pid.D = 0.0f;
    right_pid.integral_limit = 3.0f; right_pid.output_limit = 2.0f;

    pose_pid.P  = 0.0f;  pose_pid.I  = 0.0f;  pose_pid.D  = 0.0f;
    pose_pid.integral_limit = 1.0f;  pose_pid.output_limit = 1.0f;
}

void VelocityCorrector::setup() {
    // Snapshot the enabled state so restoreEnabled() can return to it later.
    _wheel_enabled_init = _wheel_enabled;
    _pose_enabled_init  = _pose_enabled;
    left_pid.reset();
    right_pid.reset();
    pose_pid.reset();
    _last_time = micros();
}

void VelocityCorrector::reset() {
    left_pid.reset();
    right_pid.reset();
    pose_pid.reset();
    _last_left_corr  = 0.0f;
    _last_right_corr = 0.0f;
    _last_pose_corr  = 0.0f;
    _last_time = micros(); // restart the dt timer so first run() after reset is clean
}

void VelocityCorrector::run() {
    unsigned long now = micros();
    if (now - _last_time < _update_ms * 1000UL) return;

    const float dt = static_cast<float>(now - _last_time) / 1000000.0f;
    _last_time = now;

    // Get commanded (DR) and actual (encoder) wheel velocities
    MotorsVelocity set    = _kinematics.getMotorsSetVelocity();
    MotorsVelocity actual = _kinematics.getMotorsVelocity();

    // Per-wheel correction: error = commanded - actual
    if (_wheel_enabled) {
        _last_left_corr  = left_pid.compute(set.left,  actual.left,  dt);
        _last_right_corr = right_pid.compute(set.right, actual.right, dt);
    } else {
        _last_left_corr  = 0.0f;
        _last_right_corr = 0.0f;
    }

    // Pose-level angular correction (optional)
    // Error = angle DR intended − angle odometry measured
    // A positive error means the robot rotated less than commanded:
    //   → slow down left wheel / speed up right wheel to nudge CCW
    _last_pose_corr = 0.0f;
    if (_pose_enabled) {
        float angle_err = _dead_reckoning.getCurPose().angle
                        - _odometry.getCurPose().angle;
        // Normalise to [-π, π] to handle wrap-around
        angle_err = atan2f(sinf(angle_err), cosf(angle_err));
        _last_pose_corr = pose_pid.compute(0.0f, -angle_err, dt);
    }

    // Compose final corrected velocities
    float left_out  = set.left  + _last_left_corr  - _last_pose_corr;
    float right_out = set.right + _last_right_corr  + _last_pose_corr;

    // Only push a correction when the motors are commanded to move
    if (_kinematics.getMotorsEnabled()) {
        _kinematics.applyVelocityBias(left_out, right_out);
    }
}

// ------------------------------------------------------------------
// Tuning API
// ------------------------------------------------------------------

void VelocityCorrector::setWheelPIDGains(float p, float i, float d) {
    left_pid.setGains(p, i, d);
    right_pid.setGains(p, i, d);
}

void VelocityCorrector::setLeftWheelPIDGains(float p, float i, float d) {
    left_pid.setGains(p, i, d);
}

void VelocityCorrector::setRightWheelPIDGains(float p, float i, float d) {
    right_pid.setGains(p, i, d);
}

void VelocityCorrector::setPosePIDGains(float p, float i, float d) {
    pose_pid.setGains(p, i, d);
}

void VelocityCorrector::enableWheelCorrection(bool enable) {
    _wheel_enabled = enable;
    if (!enable) {
        left_pid.reset();
        right_pid.reset();
        _last_left_corr  = 0.0f;
        _last_right_corr = 0.0f;
    }
}

void VelocityCorrector::enablePoseCorrection(bool enable) {
    _pose_enabled = enable;
    if (!enable) {
        pose_pid.reset();
        _last_pose_corr = 0.0f;
    }
}

void VelocityCorrector::enableWheelCorrectionPID(bool enable) {
    _wheel_enabled_init = enable;
    _wheel_enabled = enable;
    if (!enable) {
        left_pid.reset();
        right_pid.reset();
        _last_left_corr  = 0.0f;
        _last_right_corr = 0.0f;
    }
}

void VelocityCorrector::enablePoseCorrectionPID(bool enable) {
    _pose_enabled_init = enable;
    _pose_enabled = enable;
    if (!enable) {
        pose_pid.reset();
        _last_pose_corr = 0.0f;
    }
}

void VelocityCorrector::restoreEnabled() {
    enableWheelCorrection(_wheel_enabled_init);
    enablePoseCorrection(_pose_enabled_init);
}

void VelocityCorrector::setWheelOutputLimit(float limit_rad_s) {
    left_pid.output_limit  = limit_rad_s;
    right_pid.output_limit = limit_rad_s;
}

void VelocityCorrector::setPoseOutputLimit(float limit_rad_s) {
    pose_pid.output_limit = limit_rad_s;
}

void VelocityCorrector::setWheelIntegralLimit(float limit) {
    left_pid.integral_limit  = limit;
    right_pid.integral_limit = limit;
}

void VelocityCorrector::setLeftWheelIntegralLimit(float limit) {
    left_pid.integral_limit = limit;
}

void VelocityCorrector::setRightWheelIntegralLimit(float limit) {
    right_pid.integral_limit = limit;
}

void VelocityCorrector::setPoseIntegralLimit(float limit) {
    pose_pid.integral_limit = limit;
}

} // namespace SmallRobots
