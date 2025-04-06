#include "./DeadReckoning.h"

namespace SmallRobots {

    Pose deadReckoningPose;
   
    DeadReckoning::DeadReckoning(DifferentialKinematics& _kinematics): kinematics(_kinematics)
    {
    };
    DeadReckoning::~DeadReckoning()
    {};


    void DeadReckoning::setup(){
        lastTime = micros();
    };

    void DeadReckoning::run(){

        unsigned long now = micros();
        if (now - lastTime > update_ms*1000){
            deltaT = now - lastTime; 
            lastTime = now;

            updatePose(deltaT);
            deadReckoningPose = getCurPose();
        }
    };


    void DeadReckoning::updatePose(unsigned long _deltaT)
    {
        if (kinematics.getMotorsEnabled()){
            Pose deltaPose = kinematics.getDeltaPose(_deltaT,curPose, "deadreckoning");
            curPose.x += deltaPose.x;
            curPose.y += deltaPose.y;
            curPose.angle += deltaPose.angle;
        }
    };


    void DeadReckoning::setLastTime(){
  
        lastTime = kinematics.getMotorsSetVelocityTime();
    };

    
    Pose DeadReckoning::getCurPose()
    {
        return curPose;
    };

    void DeadReckoning::resetCurPose(){
        curPose.x = 0;
        curPose.y = 0;
        curPose.angle = 0;

    }; 
    void DeadReckoning::setCurPose(float x, float y, float angle){
        curPose.x = x;
        curPose.y = y;
        curPose.angle = angle;
    };


}; // namespace SmallRobots