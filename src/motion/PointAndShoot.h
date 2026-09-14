#pragma once

#include "Vector.h"
#include "structs.h"
#include "DifferentialKinematics.h"

namespace SmallRobots {

enum PointAndShootState {
    PAS_START_ROTATING,  // Initial state: start rotating
    PAS_ROTATING,          // Step 1: Rotate to face desired direction
    PAS_START_MOVING,      // Step 2: Transition state to start moving
    PAS_MOVING,            // Step 3: Keep moving straight
    PAS_SMOOTH_MOVING,     // Smooth curved movement (SMOOTHED_LINE_FOLLOWING mode)
    PAS_START_ROTATE_AT_TARGET, // (Optional) Start rotating to final target angle
    PAS_ROTATE_AT_TARGET,  // (Optional) Rotate to final target angle
    PAS_START_ODOMETRY_ROTATE, // Raw odometry-based rotation
    PAS_ODOMETRY_ROTATING,     // Rotating using only odometry data
    PAS_START_ODOMETRY_MOVE,   // Raw odometry-based movement
    PAS_ODOMETRY_MOVING,       // Moving using only odometry data
    PAS_STOP,           // Stopped
    PAS_IDLE
};

enum class PointAndShootMode {
    INFINITE,  // Rotate to heading, move forever
    TARGET,     // Rotate to heading, move to target pose, rotate to target angle
    SMOOTHED_LINE_FOLLOWING, // Continuously update heading with smoothing while moving
    VELOCITY_TRACKING,      // Direct velocity tracking from external controller (camera/simulation)
    DISTANCE                // Move straight for a fixed distance in current heading, then stop
};

class PointAndShoot {
public:
    PointAndShoot(DifferentialKinematics& _kinematics);
    ~PointAndShoot();
    
    // INFINITE Mode
    // Set desired velocity - will be executed in two steps
    void setDesiredVelocity(float vx, float vy,float speed = -1.0f);
    
    // SMOOTHED_LINE_FOLLOWING Mode
    // Set desired velocity with smooth heading updates (call when receiving new server data)
    void setDesiredVelocitySmoothed(float vx, float vy, float speed = -1.0f,
                                    float smoothing_factor = 0.5f, 
                                    float significant_heading_change_rad = 0.02f);
    
    // VELOCITY_TRACKING Mode
    // Direct velocity tracking for external controller (camera/simulation)
    // Smoothly curves toward desired heading while moving - no stop-rotate-move cycle
    // For large heading changes (> rotate_in_place_threshold), rotates in place first
    void setTrackedVelocity(float vx, float vy, float speed = -1.0f,
                            float max_angular_rate = 15.0f);
    
    // Set the heading error threshold above which the robot rotates in place
    // instead of curving (default: 90 degrees)
    void setRotateInPlaceThreshold(float threshold_rad) { rotate_in_place_threshold = threshold_rad; }

    // NOT USED CURRENTLY    
    // Smooth heading update with low-pass filtering (best for line following)
    // Gradually blends new heading with current heading to avoid jerky changes
    void updateDesiredHeadingSmoothed(float vx, float vy, float smoothing_factor = 0.5f, float significant_heading_change_rad = 0.02f);
    
    // TARGET Mode
    void setTarget(const Pose& target, const Pose& current_pose, float speed = -1.0f);

    // Rotate in place to an absolute heading angle, then stop
    // turnPref: 0 = shortest path, 1 = force CCW, -1 = force CW
    void setHeading(float angle_rad, float speed = -1.0f, int8_t turnPref = 0);

    // Move straight for a fixed distance in the current heading, then stop
    void setMoveDistance(float distance_mm, float speed = -1.0f);

    // ODOMETRY_RAW Modes: Use only local odometry (encoder/IMU), no external tracking
    // Rotate by a specific angle using raw odometry data - purely local, no external tracking dependency
    // angle_deg: rotation angle in degrees (positive = CCW, negative = CW)
    void rotateByDegrees(float angle_deg, float speed = -1.0f);
    
    // Move by a specific distance using raw odometry data - purely local, no external tracking dependency
    // distance_mm: distance to travel in mm (positive = forward, negative = backward)
    void moveBy(float distance_mm, float speed = -1.0f);

    // Execute the current step
    void run(const Pose& current_pose);
    
    // Configure parameters
    void setHeadingTolerance(float tolerance_rad);
    void setCurvatureFactor(float factor=50.0f){ // Tunable: smaller = tighter curves, larger = gentler curves
        curvature_factor = factor;
    }

    void setRobotVelocity(float _vRobot);
    void setRobotVelocityAndActivate(float _vRobot);

    // Get current state
    PointAndShootState getState() const { return state; }
   
    
    // Stop movement
    void stop();
    
protected:
    Pose curPose;
private:
    DifferentialKinematics& kinematics;
    
    PointAndShootState state = PAS_IDLE;
    PointAndShootMode mode;
    Pose target_pose;
    
    float desired_heading = 0.0f;
    float heading_tolerance = 0.2f; // Radians (~2.86 degrees)
    float slow_down_angle = 1.5f; // Radians - below this angle, slow down rotation, typically 3–5× the heading_tolerance
    float min_rotation_speed = 50.0f; // Minimum rotation speed 
    float dist_tolerance = 5.0f;     // mm

    float robotSpeed = DEFAULT_ROBOT_SPEED;  // Speed value used for both rotate and move
    float rotationSpeed = DEFAULT_ROBOT_SPEED;  // Current rotation speed
    
    int rotationDirection;  // 1 for CCW, -1 for CW
    float last_rotation_speed = -1.0f;  // -1 forces a command on first cycle
    Vector start_moving_pos;  // Position when we started moving towards target
    float target_distance;      // Distance to travel to reach target
    float multiplierRotationSpeed = 1.0f;          // make rotation faster than move speed
    
    
    // Smoothing variables for line following
    float smoothed_desired_heading = 0.0f;  // Filtered heading value
    float smoothing_factor = 0.25f;         // Low-pass filter factor for smooth line following
    float significant_heading_change_rad = 0.15f;  // Threshold before restarting rotation
    float curvature_factor = 200.0f;  // Tunable: smaller = tighter curves, larger = gentler curves

    // Velocity tracking state
    float tracked_vx = 0.0f;
    float tracked_vy = 0.0f;
    float max_angular_rate = 15.0f;       // rad/s - limits how fast robot can turn while moving
    float rotate_in_place_threshold = M_PI / 2.0f;  // 90° - rotate in place above this

    float min_move_speed = 15.0f;         // mm/s - minimum forward speed during slowdown
    float move_slow_down_distance = 50.0f; // mm - braking zone before distance target
    float moveDirection = 1.0f;            // 1 = forward, -1 = backward
    int8_t turnDirectionPref = 0;          // 0 = shortest, 1 = force CCW, -1 = force CW
    
    // Odometry-based movement state (raw, no external tracking)
    float target_odometry_angle = 0.0f;    // Target angle for rotateByDegrees (in odometry frame)
    float starting_odometry_angle = 0.0f;  // Starting angle when rotateByDegrees was called
    float target_odometry_distance = 0.0f; // Target distance for moveBy
    float starting_odometry_distance = 0.0f; // Starting distance when moveBy was called

    // Private step functions
    void stepStartRotating(const Pose& current_pose);
    void stepRotateToHeading(const Pose& current_pose);
    void stepStartMovingTarget(const Pose& current_pose);
    void stepStartMovingDistance(const Pose& current_pose);
    void stepMovingSmoothCurve(const Pose& current_pose);  // SMOOTHED_LINE_FOLLOWING helper
    void stepVelocityTracking(const Pose& current_pose);   // VELOCITY_TRACKING helper
    void stepStartMovingInfiniteStraight();
    void stepCheckTargetReached(const Pose& current_pose);
    void stepStartRotateAtTarget(const Pose& current_pose);
    void stepRotateAtTarget(const Pose& current_pose);
    void stepStartOdometryRotate(const Pose& current_pose);
    void stepOdometryRotating(const Pose& current_pose);
    void stepStartOdometryMove(const Pose& current_pose);
    void stepOdometryMoving(const Pose& current_pose);
    void stepStop(const Pose& current_pose);
};

} // namespace SmallRobots