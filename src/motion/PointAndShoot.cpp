#include "PointAndShoot.h"
#include <cmath>

namespace SmallRobots {

PointAndShoot::PointAndShoot(DifferentialKinematics& _kinematics) 
    : kinematics(_kinematics), state(PAS_ROTATING)
{
}

PointAndShoot::~PointAndShoot()
{
}

void PointAndShoot::run(const Pose& current_pose) {
    switch (state) {
        case PAS_START_ROTATING:
            stepStartRotating(current_pose);
            break;
            
        case PAS_ROTATING:
            stepRotateToHeading(current_pose);
            break;
            
        case PAS_START_MOVING:
            stepStartMoving(current_pose);
            break;
            
        case PAS_MOVING:
            if (mode == PointAndShootMode::TARGET) {
                stepCheckTargetReached(current_pose);
            }
            break;
            
        case PAS_START_ROTATE_AT_TARGET:
            stepStartRotateAtTarget(current_pose);
            break;
            
        case PAS_ROTATE_AT_TARGET:
            stepRotateAtTarget(current_pose);
            break;
            
        case PAS_STOP:
            stepStop(current_pose);
            break;
            
        case PAS_IDLE:
            // Do nothing
            break;
    }
}

// INFINITE Mode: Set desired velocity
void PointAndShoot::setDesiredVelocity(float vx, float vy, float speed) {
    mode = PointAndShootMode::INFINITE;
    desired_heading = atan2(vy, vx);
    
    // Optionally update speed, otherwise keep existing speedValue
    if (speed >= 0.0f) {
        robotSpeed = speed;
    }
    
    state = PAS_START_ROTATING;
}

// TARGET Mode: Set target pose
void PointAndShoot::setTarget(const Pose& target, const Pose& current_pose, float speed) {
    mode = PointAndShootMode::TARGET;
    target_pose = target;
    
    // Calculate heading vector from current pose to target
    float vx = target.x - current_pose.x;
    float vy = target.y - current_pose.y;
    desired_heading = atan2(vy, vx);
    
    // Optionally update speed, otherwise keep existing speedValue
    if (speed >= 0.0f) {
        robotSpeed = speed;
    }
    
    state = PAS_START_ROTATING;
}

// Step 1: Calculate target angle and send rotate command with direction
void PointAndShoot::stepStartRotating(const Pose& current_pose) {
    float angle_error = desired_heading - current_pose.angle;
    angle_error = atan2(sin(angle_error), cos(angle_error));
    
    // Choose rotation direction based on shortest angular distance
    if (angle_error > 0) {
        rotationDirection = 1;  // Counter-clockwise
        kinematics.rotateCCW(robotSpeed);  // Counter-clockwise
    } else {
        rotationDirection = -1;  // Clockwise
        kinematics.rotateCW(robotSpeed);   // Clockwise
    }
    state = PAS_ROTATING;
}


// Step 2: Check if arrived at target heading
void PointAndShoot::stepRotateToHeading(const Pose& current_pose) {
    float angle_error = desired_heading - current_pose.angle;
    
    // Normalize angle error to [-π, π]
    angle_error = atan2(sin(angle_error), cos(angle_error));
    
    // Check if we've arrived, considering rotation direction
    if (fabs(angle_error) < heading_tolerance) {
        state = PAS_START_MOVING;
    }
    // Verify we're still rotating in the correct direction
    else if ((rotationDirection > 0 && angle_error < 0) || 
             (rotationDirection < 0 && angle_error > 0)) {
        // Overshot the target, transition to next state
        state = PAS_START_MOVING;
    }
}

// Step 3: Send move straight command once
void PointAndShoot::stepStartMoving(const Pose& current_pose) {
    kinematics.move(robotSpeed, 1e6f);  // Very large radius = straight line
    state = PAS_MOVING;
}

// Step 4: Check if target reached (only in TARGET mode)
void PointAndShoot::stepCheckTargetReached(const Pose& current_pose) {
    float dx = target_pose.x - current_pose.x;
    float dy = target_pose.y - current_pose.y;
    float distance = sqrt(dx * dx + dy * dy);
    
    if (distance < 0.01f) {  // Reached target location
        state = PAS_START_ROTATE_AT_TARGET;
    }
}

// Step 5: Send rotate to target angle command with direction
void PointAndShoot::stepStartRotateAtTarget(const Pose& current_pose) {
    float angle_error = target_pose.angle - current_pose.angle;
    angle_error = atan2(sin(angle_error), cos(angle_error));
    
    // Choose rotation direction based on shortest angular distance
    if (angle_error > 0) {
        rotationDirection = 1;  // Counter-clockwise
        kinematics.rotateCCW(robotSpeed);  // Counter-clockwise
    } else {
        rotationDirection = -1;  // Clockwise
        kinematics.rotateCW(robotSpeed);   // Clockwise
    }
    state = PAS_ROTATE_AT_TARGET;
}

// Step 6: Check if arrived at target angle
void PointAndShoot::stepRotateAtTarget(const Pose& current_pose) {
    float angle_error = target_pose.angle - current_pose.angle;
    
    // Normalize angle error to [-π, π]
    angle_error = atan2(sin(angle_error), cos(angle_error));
    
    // Check if we've arrived, considering rotation direction
    if (fabs(angle_error) < heading_tolerance) {
        state = PAS_STOP;
    }
    // Verify we're still rotating in the correct direction
    else if ((rotationDirection > 0 && angle_error < 0) || 
             (rotationDirection < 0 && angle_error > 0)) {
        // Overshot the target, transition to stop
        state = PAS_STOP;
    }
}


// Step 7: Stop and go idle
void PointAndShoot::stepStop(const Pose& current_pose) {
    kinematics.stop();
    state = PAS_IDLE;
}

void PointAndShoot::stop() {
    kinematics.stop();
    state = PAS_IDLE;
}

void PointAndShoot::setHeadingTolerance(float tolerance_rad) {
    heading_tolerance = tolerance_rad;
}


void PointAndShoot::setRobotVelocity( float _vRobot){ //in mm/s
    if(_vRobot >=0) robotSpeed = _vRobot; //-1 resets to the last set speed
};
void PointAndShoot::setRobotVelocityAndActivate(float _vRobot){ //in mm/s and send move command with current radius
        //make sure that the sign of new velocity is the same as in current movement
    if(_vRobot >=0){ 
        float speed = kinematics.getCurRobotSpeed();
        if (speed >= 0)robotSpeed = abs( _vRobot );
        else robotSpeed = - abs(_vRobot);
        robotSpeed =_vRobot;
        float radius = kinematics.getCurRobotRadius(); 
        if (state != PAS_IDLE && state != PAS_STOP){
            kinematics.move(robotSpeed, radius);
        }
    }

};


} // namespace SmallRobots