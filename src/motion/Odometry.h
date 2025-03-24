#pragma once

#include "Arduino.h"
#include "DifferentialKinematics.h"


namespace SmallRobots {
class Odometry {
        public:
            Odometry(DifferentialKinematics& _kinematics);
            
            ~Odometry();
            void updatePose(String curDirName);
            void resetLastTime();
            int updateDeltaT();
            Pose getCurPose();

        protected:
            DifferentialKinematics& kinematics;
            //Start Pose: location x,y and heading angle
            //Local coordinate system at the moment
            //angle = 0 heading in y direction
            Pose curPose = Pose(); 
            
            
            int lastTime=0, deltaT=0; //delat T, read in millis,later converted to seconds to get m/s as unit ???
            
    };




}; // namespace SmallRobots