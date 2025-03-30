#include "./DifferentialKinematics.h"

namespace SmallRobots {



    DifferentialKinematics::DifferentialKinematics(float _wheel_base, float wheel_diameter) {
        default_speed = 0.5;
        half_wheel_base = _wheel_base/2.0f;
        wheel_base = _wheel_base;
        wheel_radius = wheel_diameter/2.0f;
        wheel_circumference = 2*wheel_radius * M_PI;
    };

    DifferentialKinematics::~DifferentialKinematics() {
    };

    
   

    void DifferentialKinematics::move(float speed, float radius) { //negative radius + negative speed when leftturn = counterclockwise
        
        //speed in mm/s for straight
        //speed in mm/s for movement on arc
        setCurRobotSpeed(speed);
        setCurRobotRadius(radius);
     
        if (radius == RADIUS_STREIGHT) {
            float wheelSpeed = speed /wheel_circumference* 2*PI;
            setSpeed(wheelSpeed, -wheelSpeed);
        }
        else if (radius == 0.0) {
            float wheelSpeed = speed / half_wheel_base;
            setSpeed(-wheelSpeed, -wheelSpeed); 
        }
        else {

            float angularSpeed = speed / abs(radius);
            if (abs(radius) <= wheel_base) angularSpeed = speed / wheel_base;
            

            //R is ICC to the center of the robot
            // // LEFT ARC (+radius) FORWARD (+speed) or BACKWARD (-speed)
            float left =   (radius - half_wheel_base) * angularSpeed /wheel_radius;
            float right = -(radius + half_wheel_base) * angularSpeed /wheel_radius;

            //RIGHT ARC (-radius) FORWARD (-speed) or BACKWARD (+speed)
            if(radius <0 ) {
                radius = abs(radius);
                left =   (radius + half_wheel_base) * angularSpeed /wheel_radius;
                right = -(radius - half_wheel_base) * angularSpeed /wheel_radius;
            }

            //WORKS TOO
            //R is ICC to the outside wheel of the robot
            // // LEFT ARC (+radius) FORWARD (+speed) or BACKWARD (-speed)
            // float left =   (radius) * angularSpeed /wheel_radius;
            // float right = -(radius + 2* half_wheel_base) * angularSpeed /wheel_radius;
            
            // //RIGHT ARC (-radius) FORWARD (-speed) or BACKWARD (+speed)
            // if(radius <0 ) {
            //     radius = abs(radius);
            //     left =   (radius + 2* half_wheel_base) * angularSpeed /wheel_radius;
            //     right = -(radius ) * angularSpeed /wheel_radius;
            // }
          
            // Serial.println("left: " + (String) left + " , right: " + (String) right);
            setSpeed(left, right);

    
        }
    };
    void DifferentialKinematics::turnLeftForward(float speed, float radius){//cannot be used to go into another direction than specified,to guarantee the direction, values are turned absolute
        move(abs(speed),abs(radius));
    };
    void DifferentialKinematics::turnRightForward(float speed, float radius){
        move(abs(speed),-abs(radius));
    };

    void DifferentialKinematics::turnLeftBackward(float speed, float radius){
        move(-abs(speed),abs(radius));
    };
    void DifferentialKinematics::turnRightBackward(float speed, float radius){
        move(-abs(speed),-abs(radius));
    };

    void DifferentialKinematics::rotate(float speed) {
        move(speed, 0.0f);
    };


    Pose DifferentialKinematics::getDeltaPose(unsigned long deltaT, Pose lastPose){ //delataT in micros as SimpleFoc reads velocity /micros ?!

        MotorsVelocity vel = getMotorsVelocity();
        float vR = -vel.right * wheel_radius;  //negative because of the way motors are mounted
        float vL = vL * wheel_radius; //at shaft --> at wheel, wheel tangential velocities
       
        float deltaSR = vR*deltaT;
        float deltaSL = vL*deltaT;

        Pose deltaPose;
        deltaPose.x = (deltaSR+deltaSL)/2.0  * cos( lastPose.angle ) ;
        deltaPose.y = (deltaSR+deltaSL)/2.0  * sin( lastPose.angle  ) ;
        deltaPose.angle = (deltaSR-deltaSL)/(2*half_wheel_base);
    
        return deltaPose;
    }
   
    void DifferentialKinematics::setCurRobotSpeed(float _curRobotSpeed){
        curRobotSpeed = _curRobotSpeed;
    };
    float DifferentialKinematics::getCurRobotSpeed(){
        return curRobotSpeed;
    };

    void DifferentialKinematics::setCurRobotRadius(float _curRobotRadius){
        curRobotRadius =_curRobotRadius;
    };
    float DifferentialKinematics::getCurRobotRadius(){
        return curRobotRadius;
    };

  
    
}; // namespace SmallRobots