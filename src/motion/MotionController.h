#pragma once

#include <stdint.h>
#include <limits>
#include "Arduino.h"
#include "../config/globalStructs.h"
#include "DifferentialKinematics.h"
#include "DifferentialPathPlanner.h"

#define DEFAULT_ROBOT_SPEED 100 //mm/s
#define DEFAULT_PATH_RADIUS 50 //mm

namespace SmallRobots {


    class MotionController {

        protected:

            std::vector<Pose> path;
            int curPathIndex = 0;

            Pose curPose;
            Pose targetPose;

            


            Vector ICC; //Instantaneous Center of Curvature
            float R = MINRADIUS; //dist between robot's pose and its ICC

            float vRobot = DEFAULT_ROBOT_SPEED; //mm/s

            float vR =0.00;
            float vL =0.00;

            float targetAngle=0;

            float arriveDistance = 20; //precission of arriving behaviour
            float arriveAngleDistance = 0;

            float curDistance = 0;
            float lastDistance = 10000000000;

            
            Vector curV;

        public:

                    
                    
            MotionController(DifferentialKinematics& drive); //, DifferentialPathPlanner& pathPlanner);
            ~MotionController();

            int pathBehaviour = LOOP ; //PATHBEHAVIOURS

        

            void addPoseToPath(Pose p);
            void setPoseToReplacePath(Pose p);
            void addPoseListToPath(std::vector<Pose> poses);
            void setPoseListToReplacePath(std::vector<Pose> poses);

            void deletePath();
            void setLoopPath();
            void setPausePath();
            void setContinuePath();
            void setEndPath();
            void setRestartPath();


            void setTarget(); //get next pose in path, calculate dubin path from current pose and target pose
            void setWheelVelocitiesSeg1(); //ARC left or right with minRad
            void setWheelVelocitiesSeg2(); //STRAIGHT or left or right ARC
            void setWheelVelocitiesSeg3(); //ARC left or right
            void stop();
            
            void stopMoving();
            void enableMotors();

            void setRobotVelocity(float _vRobot = DEFAULT_ROBOT_SPEED); //in mm/s
            void setPathRadius(float _radius = DEFAULT_PATH_RADIUS); //in mm , absolute value
            void setCurPose(Pose pose);

            bool loopPath(); //returns true if there is more than one pose in the path
            bool checkIfArrived();

            DifferentialKinematics& kinematics;
            DifferentialPathPlanner pathPlanner = DifferentialPathPlanner();
            
            String curDirName = "N";
    };

}; //end namespace SmallRobots 



