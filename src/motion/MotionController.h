#pragma once

#include <stdint.h>
#include <limits>
#include <Arduino.h>
#include "./Vector.h"
#include "./DifferentialKinematics.h"
#include "./DifferentialPathPlanner.h"
#include "../localisation/Odometry.h"

#define DEFAULT_ROBOT_SPEED 100 //mm/s
#define DEFAULT_PATH_RADIUS 50 //mm

namespace SmallRobots {

    enum PATHBEHAVIOURS {
        PAUSE,
        LOOP,
        END,
        CONTINUE,
        RESTART,
    };


    
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

            int subPathIndex =-1; //to go through dubin path segments
            bool pathStateChanged = false;
            int pathState =0;
        public:

                    
                    
            MotionController(DifferentialKinematics& drive, Odometry& odommetryCtrl); //, DifferentialPathPlanner& pathPlanner);
            ~MotionController();

            int pathBehaviour = END ; //PATHBEHAVIOURS

            void setup();
            void run();

            void activateNewTarget();
            void addPoseToPath(Pose p);
            void addPoseToPathAndGoThereFirst(Pose p);
            void setPoseToReplacePath(Pose p);
            void addPoseListToPath(std::vector<Pose> poses);
            void addPoseListToPathAndGoThereFirst(std::vector<Pose> poses);
            void setPoseListToReplacePath(std::vector<Pose> poses);

            void deletePath();
            void setLoopPath();
            void setPausePath();
            void setContinuePath();
            void setEndPath();
            void setRestartPath();


            void calculateDubinForNextPoseInPath(); //get next pose in path, calculate dubin path from current pose and target pose
            void setWheelVelocitiesSeg1(); //ARC left or right with minRad
            void setWheelVelocitiesSeg2(); //STRAIGHT or left or right ARC
            void setWheelVelocitiesSeg3(); //ARC left or right
            void stop();
            void enableMotors();

            void setRobotVelocity(float _vRobot = DEFAULT_ROBOT_SPEED); //in mm/s
            void setPathRadius(float _radius = DEFAULT_PATH_RADIUS); //in mm , absolute value
            void setPathBevahiourType(int type);

            bool loopPath(); //returns true if there is more than one pose in the path
            bool checkIfArrived();

            DifferentialKinematics& kinematics;
            Odometry& odometryCtrl;
            DifferentialPathPlanner pathPlanner = DifferentialPathPlanner();
            
            String curDirName = "N";
    };

}; //end namespace SmallRobots 



