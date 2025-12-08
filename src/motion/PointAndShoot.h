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
    PAS_START_ROTATE_AT_TARGET, // (Optional) Start rotating to final target angle
    PAS_ROTATE_AT_TARGET,  // (Optional) Rotate to final target angle
    PAS_STOP,           // Stopped
    PAS_IDLE
};

enum class PointAndShootMode {
    INFINITE,  // Rotate to heading, move forever
    TARGET     // Rotate to heading, move to target pose, rotate to target angle
};

class PointAndShoot {
public:
    PointAndShoot(DifferentialKinematics& _kinematics);
    ~PointAndShoot();
    
    // INFINITE Mode
    // Set desired velocity - will be executed in two steps
    void setDesiredVelocity(float vx, float vy,float speed = -1.0f);
    // TARGET Mode
    void setTarget(const Pose& target, const Pose& current_pose, float speed = -1.0f);

    // Execute the current step
    void run(const Pose& current_pose);
    
    // Configure parameters
    void setHeadingTolerance(float tolerance_rad);

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
    float heading_tolerance = 0.02f; // Radians (~1.15 degrees)

    float robotSpeed = DEFAULT_ROBOT_SPEED;  // Speed value used for both rotate and move
       
    int rotationDirection;  // 1 for CCW, -1 for CW
    Vector start_moving_pos;  // Position when we started moving towards target
    float target_distance;      // Distance to travel to reach target

    // Private step functions
    void stepStartRotating(const Pose& current_pose);
    void stepRotateToHeading(const Pose& current_pose);
    void stepStartMoving(const Pose& current_pose);
    void stepCheckTargetReached(const Pose& current_pose);
    void stepStartRotateAtTarget(const Pose& current_pose);
    void stepRotateAtTarget(const Pose& current_pose);
    void stepStop(const Pose& current_pose);
};

} // namespace SmallRobots