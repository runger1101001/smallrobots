#pragma once

#include <stdint.h>
#include <limits>
#include <Arduino.h>
#include "./Vector.h"
#include "./DifferentialKinematics.h"
#include "./DifferentialPathPlanner.h"
#include "../localisation/Odometry.h"
#include "PointAndShoot.h"
#include "DubinController.h"


namespace SmallRobots {

    enum MotionMode {
        DUBINS_PATH,      // Original path planning mode
        POINT_AND_SHOOT   // Two-step: rotate first, then move
    };


    
    class MotionController {

        protected:

            Pose curPose;
            Pose targetPose;
            float vRobot = DEFAULT_ROBOT_SPEED; //mm/s
        public:

                    
                    
            MotionController(DifferentialKinematics& drive, Odometry& odommetryCtrl);
            ~MotionController();

            void setup();
            void run();

            void setDesiredVelocityPointAndShoot(float vx, float vy, float speed = -1.0f);
            void setTargetPointAndShoot(const Pose& target, float speed = -1.0f);
            void setTargetDubinsPath(const Pose& target, float speed = -1.0f);

            void stop();
            void enableMotors();

            //for both modes
            void setRobotVelocity(float _vRobot = DEFAULT_ROBOT_SPEED); //in mm/s

            //for dubins path mode
            void setDubinsPathRadius(float _radius);

            //for point and shoot mode
            // has to be updated when desired robot velocity is set over osc
            // bypasses path planning with dubin paths
            void setPointAndShootParams(float tolerance_rad, float speed_rad_s, float max_vel_mm_s){
                point_and_shoot.setHeadingTolerance(tolerance_rad);
                point_and_shoot.setRobotVelocity(max_vel_mm_s);
            } //TODO change


            DifferentialKinematics& kinematics;
            Odometry& odometryCtrl;

        private:
            MotionMode currentMode = DUBINS_PATH;

            PointAndShoot point_and_shoot;
            DubinController dubin_controller;

    };

}; //end namespace SmallRobots



