#include "PointAndShoot.h"
#include <cmath>

namespace SmallRobots {

PointAndShoot::PointAndShoot(DifferentialKinematics& _kinematics) 
    : kinematics(_kinematics), state(PAS_IDLE), mode(PointAndShootMode::INFINITE)
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
         if (mode == PointAndShootMode::TARGET) {
                stepStartMovingTarget(current_pose);
            } else if (mode == PointAndShootMode::INFINITE) {
                stepStartMovingInfiniteStraight();
            }
            break;
            
        case PAS_MOVING:
            if (mode == PointAndShootMode::TARGET) {
                stepCheckTargetReached(current_pose);
            }
            // INFINITE mode: keep moving, no special logic needed
            break;
            
        case PAS_SMOOTH_MOVING:
            // Continuously curve toward desired heading while moving
            stepMovingSmoothCurve(current_pose);
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
    float new_heading = atan2(vy, vx);
    
    // Only update state if heading has changed significantly
    float heading_diff = new_heading - desired_heading;
    heading_diff = atan2(sin(heading_diff), cos(heading_diff));
    
    if (fabs(heading_diff) > heading_tolerance) {
        // Significant heading change - restart rotation
        desired_heading = new_heading;
        state = PAS_START_ROTATING;
    } else if (state == PAS_IDLE || state == PAS_STOP) {
        // Currently idle/stopped - start moving in current direction
        desired_heading = new_heading;
        state = PAS_START_ROTATING;
    }
    // else: small heading change while already moving - ignore to avoid jerkiness
    
    // Update speed if provided
    if (speed >= 0.0f) {
        robotSpeed = speed;
    }
}

// SMOOTHED_LINE_FOLLOWING Mode: Set desired velocity with smooth heading updates
// Call this every time you receive new direction data from server (e.g., every 100ms)
// This mode curves the robot toward the desired heading while moving
void PointAndShoot::setDesiredVelocitySmoothed(float vx, float vy, float speed, 
                                               float smoothing_factor, 
                                               float significant_heading_change_rad
                                               ) {
    mode = PointAndShootMode::SMOOTHED_LINE_FOLLOWING;
    
    // Store smoothing parameters for use in run()
    this->smoothing_factor = smoothing_factor;
    this->significant_heading_change_rad = significant_heading_change_rad;
    // Update the new heading target
    float new_heading = atan2(vy, vx);
    
     // Check if this is a significant heading change
    float heading_diff = new_heading - smoothed_desired_heading;
    heading_diff = atan2(sin(heading_diff), cos(heading_diff));
    
    // Only apply smoothing if heading change is significant
    if (fabs(heading_diff) > significant_heading_change_rad) {
        // Low-pass filter: blend new heading with current smoothed heading
        smoothed_desired_heading = smoothed_desired_heading + heading_diff * smoothing_factor;
    }
    // else: ignore small heading fluctuations to avoid jerkiness
    
    // Normalize smoothed heading to [-π, π]
    smoothed_desired_heading = atan2(sin(smoothed_desired_heading), cos(smoothed_desired_heading));
    
    // Update desired heading to the smoothed value
    desired_heading = smoothed_desired_heading;
    
    
    // Update speed if provided
    if (speed >= 0.0f) {
        robotSpeed = speed;
    }
    
    // Start smooth movement if idle/stopped
    //if (state == PAS_IDLE || state == PAS_STOP) {
        state = PAS_SMOOTH_MOVING;  // Go directly to smooth moving
        kinematics.move(robotSpeed, 1e6f);  // Start moving straight
    //}
    // If already in smooth movement, just update desired_heading (will be used in next run() call)
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
    
    if (fabs(angle_error) < heading_tolerance) {
        state = PAS_START_MOVING;
        return;
    }
    
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
// TARGET Mode: Calculate target distance and store starting position
void PointAndShoot::stepStartMovingTarget(const Pose& current_pose) {
    // Calculate distance to target and store it
    float dx = target_pose.x - current_pose.x;
    float dy = target_pose.y - current_pose.y;
    target_distance = sqrt(dx * dx + dy * dy);

    if (target_distance < dist_tolerance) {  // Already at target position
        state = PAS_START_ROTATE_AT_TARGET;
        return;
    }  
    
    // Store starting position
    start_moving_pos = Vector(current_pose.x, current_pose.y);
    
    kinematics.move(robotSpeed, 1e6f);  // Very large radius = straight line
    state = PAS_MOVING;
}
// INFINITE Mode: Send move straight command once
void PointAndShoot::stepStartMovingInfiniteStraight() {
    kinematics.move(robotSpeed, 1e6f);  // Very large radius = straight line
    state = PAS_MOVING;
}

// SMOOTHED_LINE_FOLLOWING: Continuously curve toward desired heading while moving
void PointAndShoot::stepMovingSmoothCurve(const Pose& current_pose) {
    // Calculate the angular difference between current heading and desired heading
    float heading_error = desired_heading - current_pose.angle;
    
    // Normalize to [-π, π]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    // If heading is already correct, transition to straight movement
    if (fabs(heading_error) < heading_tolerance) {
        state = PAS_START_MOVING;  // Transition to normal movement
        return;
    }
    
    // Calculate radius to achieve smooth curve toward desired heading
    // Smaller radius = tighter curve = faster heading change
    // The radius is calculated such that the robot curves toward the target heading
    
    // Using simple proportional control: radius based on heading error
    // curvature_factor is a tunable parameter through setCurvatureFactor()
    // heading_error ranges from -π to π, we want positive radius
    // Tunable: smaller = tighter curves, larger = gentler curves
    float radius = curvature_factor / fabs(heading_error);
    
    // Clamp radius to reasonable bounds
    if (radius < 0.0f) radius = 0.0f;      // Minimum radius for stability
    if (radius > 1e6f) radius = 1e6f;        // Maximum radius (nearly straight)
    
    // Apply correct sign based on turn direction
    if (heading_error < 0) {
        radius = -radius;  // Negative radius = turn right/clockwise
    }
    
    // Continuously update movement with current radius
    kinematics.move(robotSpeed, radius);
}

// Step 4: Check if target reached (only in TARGET mode)
void PointAndShoot::stepCheckTargetReached(const Pose& current_pose) {
    // Calculate travelled distance from start of movement
    float dx = current_pose.x - start_moving_pos.x;
    float dy = current_pose.y - start_moving_pos.y;
    float travelled_distance = sqrt(dx * dx + dy * dy);
    
    // Stop when travelled distance >= target distance
    if (travelled_distance >= target_distance) {
        state = PAS_START_ROTATE_AT_TARGET;
    }
}

// Step 5: Send rotate to target angle command with direction
void PointAndShoot::stepStartRotateAtTarget(const Pose& current_pose) {
    float angle_error = target_pose.angle - current_pose.angle;
    angle_error = atan2(sin(angle_error), cos(angle_error));
    
    if (fabs(angle_error) < heading_tolerance) {
        state = PAS_STOP;
        return;
    }
    
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