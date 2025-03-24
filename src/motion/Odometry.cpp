#include "Odometry.h"

namespace SmallRobots {

    Odometry::Odometry(DifferentialKinematics& _kinematics): kinematics(_kinematics)
    {
        lastTime = micros();
    };
    Odometry::~Odometry()
    {};

    void Odometry::updatePose(String curDirName)
    {
       // Serial.println("Odometry::updatePose()");
    
        MotorsVelocity vel = kinematics.getMotorsVelocity();

        //Serial.println ("vel.left: " + (String) vel.left + ", vel.right: "+ (String) vel.right);
        updateDeltaT();

        curPose = kinematics.wheelVelToNextPose(vel.left,vel.right, deltaT, curPose, curDirName);

    };

    void Odometry::resetLastTime(){
        lastTime = millis();
        //Serial.println(lastTime);
    };
    int Odometry::updateDeltaT(){
        deltaT = millis() - lastTime; //later converted to seconds
        lastTime = millis();
        //Serial.println("lastTime: " + (String) lastTime);
        //Serial.println("deltaT: " + (String) deltaT);
        return deltaT;
    };
    
    Pose Odometry::getCurPose()
    {
        return curPose;
    };


}; // namespace SmallRobots