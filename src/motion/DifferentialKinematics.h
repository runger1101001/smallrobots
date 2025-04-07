
#pragma once

#include <stdint.h>
#include <limits>
#include <Arduino.h>
#include "./Vector.h"
#include "control/SmallRobotEventBus.h"


#define RADIUS_STREIGHT (std::numeric_limits<float>::infinity())
#define MINRADIUS 50.0 //kinematics do not work when radius is bigger than half_wheel_base... why? TODO SET AUTOMATICALLY FROM ZOOIDDRIVE

namespace SmallRobots {

    struct MotorsPosition {
        int32_t left_turns;
        float left;
        int32_t right_turns;
        float right;
    };

    struct MotorsVelocity {
        float left;
        float right;
    };

    typedef struct Pose {
        float x =0;
        float y =0;
        float angle=0;
    } Pose;



    class DifferentialKinematics {
        public:
            DifferentialKinematics(float _wheel_base, float wheel_diameter);
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

            virtual void rotate(float speed);

            virtual void setSpeed(float left, float right)=0;
            virtual void stop()=0;
            virtual void start()=0;

            virtual MotorsPosition getMotorsPosition()=0;
            virtual MotorsVelocity getMotorsVelocity()=0;
            virtual MotorsVelocity getMotorsSetVelocity()=0;
            virtual unsigned long getMotorsSetVelocityTime()=0;
            virtual bool getMotorsEnabled()=0;
            Pose getDeltaPose(unsigned long deltaT,Pose lastPose, String type);// type: "odometry" or "deadreckoning" otherwise original pose is returned

            float wheel_base;
            float half_wheel_base;
            float wheel_radius;
            float wheel_circumference;
            float default_speed;

            void setCurRobotSpeed(float _curRobotSpeed);
            float getCurRobotSpeed();

            void setCurRobotRadius(float _curRobotRadius);
            float getCurRobotRadius();

            float globalCoordinateSystemOffsetAngle = M_PI/2.0; //TODO should be set by config?

        private:
            Pose pose;

            float curRobotSpeed, curRobotRadius; //updated by move, so in case speed gets updated, the move function can be called with the last set values

    };



}; // namespace SmallRobots
