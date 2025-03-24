
#pragma once

#include <stdint.h>
#include <limits>
#include "Arduino.h"
#include "./config/globalStructs.h"

#include <SimpleFOC.h>
#include "encoders/sc60228/MagneticSensorSC60228.h"

#define ZOOIDBOT_WHEELBASE 52.6  //mm org: 0.55 dm
#define ZOOIDBOT_WHEEL_D   30.6  //mm org: 0.31 dm 

#define RADIUS_STREIGHT (std::numeric_limits<float>::infinity())
#define MINRADIUS 50.0 //kinematics do not work when radius is bigger than half_wheel_base... why?

extern MagneticSensorSC60228 sensorL;
extern MagneticSensorSC60228 sensorR;
extern BLDCMotor motorL;
extern BLDCMotor motorR;


namespace SmallRobots {


    class DifferentialKinematics {
        public:
            DifferentialKinematics(float _wheel_base= ZOOIDBOT_WHEELBASE, float wheel_diameter = ZOOIDBOT_WHEEL_D);
            ~DifferentialKinematics();

            //move
            //straight: only speed, posive: forward, negative: backward
            //rotate: speed and radius = 0: speed positive ->rotate ccw, speed negative -> rotate cw
            //left arc forward / ccw: speed and radius positive
            //right arc forward /cw : speed and radius negative
            //left arc backward / ccw: speed negative and radius positive
            //right arc backward /cw : speed positive and radius negative

            void move(float speed, float radius = RADIUS_STREIGHT); 
            void turnLeftForward(float speed, float radius);
            void turnRightForward(float speed, float radius);
            void turnLeftBackward(float speed, float radius);
            void turnRightBackward(float speed, float radius);

            void rotate(float speed);

            void setSpeed(float left, float right);
            void stop();
            void enable();
            void disable();

            Pose wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose,String curDirName);

            MotorsPosition getMotorsPosition();
            MotorsVelocity getMotorsVelocity();

            float wheel_base;
            float half_wheel_base;
            float wheel_radius;
            float wheel_circumference;
            float default_speed;

            void setCurRobotSpeed(float _curRobotSpeed);
            float getCurRobotSpeed();

            void setCurRobotRadius(float _curRobotRadius);
            float getCurRobotRadius();

            float globalCoordinateSystemOffsetAngle = M_PI/2.0;

        private:
            Pose pose;
            float deltaTseconds;
            float curRobotSpeed, curRobotRadius; //updated by move, so in case speed gets updated, the move function can be called with the last set values

            //ODOMETRY MOTOR
            Pose curPose = Pose();            
            int lastTime=0, deltaT=0; //delat T, read in millis,later converted to seconds to get m/s as unit ???
    };



}; // namespace SmallRobots
