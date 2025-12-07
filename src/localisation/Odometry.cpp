#include "./Odometry.h"

namespace SmallRobots {

    Pose odometryPose;
    Pose odometryDeltaPose;
    unsigned long odometryDeltaT;

    Odometry::Odometry(DifferentialKinematics& _kinematics, EventBus<String>& event_bus): kinematics(_kinematics), event_bus(event_bus)
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
            deltaT = now - lastTime;  // Update global deltaT
            lastTime = now;

            updatePose(deltaT);
            odometryPose = getCurPose(); // Update global pose
            odometryDeltaPose = getDeltaPose();  // Update global deltaPose
            odometryDeltaT = getDeltaT();  // Update global deltaT
            //event_bus.emit(String("new_odometry_pose"));
        }
    };

    void Odometry::updatePose(unsigned long _deltaT)
    {
        deltaPose = kinematics.getDeltaPose(_deltaT,curPose, "odometry");
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