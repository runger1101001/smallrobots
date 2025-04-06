#include "./Odometry.h"

namespace SmallRobots {

    Pose odometryPose;

    Odometry::Odometry(DifferentialKinematics& _kinematics): kinematics(_kinematics)
    {
        
    };
    Odometry::~Odometry()
    {};

    void Odometry::setup(){
        lastTime = micros();
    };

    void Odometry::run(){

        unsigned long now = micros();
        if (now - lastTime > update_ms*1000){
            deltaT = now - lastTime; 
            lastTime = now;

            updatePose(deltaT);
            odometryPose = getCurPose();
            event_bus.emit("new_odometry_pose");
        }
    };

    void Odometry::updatePose(unsigned long _deltaT)
    {
        Pose deltaPose = kinematics.getDeltaPose(_deltaT,curPose, "odometry");
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