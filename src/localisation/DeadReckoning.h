#pragma once

#include <Arduino.h>
#include "./motion/DifferentialKinematics.h"

namespace SmallRobots {

    class DeadReckoning{
        public:
        DeadReckoning(DifferentialKinematics& _kinematics);
            
            ~DeadReckoning();
            void setup();
            void run();
            void updatePose(unsigned long _deltaT);
            void setLastTime();
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
            uint32_t update_ms = 10;
            
    };

    extern Pose deadReckoningPose;
}; // namespace SmallRobots