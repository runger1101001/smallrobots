#include "./Odometry.h"

namespace SmallRobots {

    Pose odometryPose;

    Odometry::Odometry(DifferentialKinematics& _kinematics): kinematics(_kinematics)
    {
        resetLastTime();
    };
    Odometry::~Odometry()
    {};

    void Odometry::updatePose()
    {
        unsigned long now = micros();
        deltaT = now - lastTime; 
        lastTime = now;
        
        Pose deltaPose = kinematics.getDeltaPose(deltaT,curPose);
        curPose.x += deltaPose.x;
        curPose.y += deltaPose.y;
        curPose.angle += deltaPose.angle;

    };

    void Odometry::resetLastTime(){
        lastTime = micros();
    };

    
    Pose Odometry::getCurPose()
    {
        return curPose;
    };

    void Odometry::resetCurPose(){
        curPose.x = 0;
        curPose.y = 0;
        curPose.angle = 0;

    }; 
    void Odometry::setCurPose(float x, float y, float angle){
        curPose.x = x;
        curPose.y = y;
        curPose.angle = angle;
    };


}; // namespace SmallRobots