#pragma once

#include <Arduino.h>
#include "./motion/DifferentialKinematics.h"

namespace SmallRobots {

    class Odometry{
        public:
            Odometry(DifferentialKinematics& _kinematics);
            
            ~Odometry();
            void updatePose();
            void resetLastTime();
            Pose getCurPose();
            void resetCurPose(); //to start pose 0,0,0
            void setCurPose(float x, float y, float angle);

        protected:
            DifferentialKinematics& kinematics;
            //Start Pose: location x,y and heading angle
            //Local coordinate system at the moment
            //angle = 0 heading in y direction
            Pose curPose = Pose(); 
            
            int lastTime=0, deltaT=0; //delat T, read in micros
            
    };

    extern Pose odometryPose;
}; // namespace SmallRobots