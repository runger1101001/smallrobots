#include "DifferentialKinematics.h"

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


    Pose DifferentialKinematics::wheelVelToNextPose (float vL, float vR, int deltaT, Pose lastPose, String curDirName){
        
        //Serial.println("vL: " + (String) vL + " , vR: " + (String) vR);

        vR = -vR; //the way motors are mounted

        vL = vL * wheel_radius; //at shaft --> at wheel, wheel tangential velocities
        vR = vR * wheel_radius;

        // float aV = (vR-vL)/wheel_base;//angular velocity of robot
        // float R = half_wheel_base *(vR + vL)/(vR - vL); //R = distance from ICR to the center of the robot

        //Serial.println ("R: " + (String) R + "mm");
        //Serial.println ("angular velocity: " + (String) aV + "unit?");

        // float V = (vR + vL)/2.0 ; //instantaneous velocity V of the point midway between the robot's wheels

        //Serial.println ("instantaneous velocity midway between wheels: " + (String) V + "unit?");


        deltaTseconds = deltaT / 1000.0f;
        //Serial.println("deltaTseconds: " +  (String) deltaTseconds);

        // float theta = lastPose.angle;
        // float L = half_wheel_base;

        float deltaSR = vR*deltaTseconds;
        float deltaSL = vL*deltaTseconds;

        // float deltaTheta = (deltaSR-deltaSL)/(2*L);
        // float deltaS = (deltaSR+deltaSL)/2.0;

        // float deltaX = deltaS * cos(theta + deltaTheta/2.0);
        // float deltaY = deltaS * sin(theta + deltaTheta/2.0);
    

        //R is distance to outside wheel
        //WORKS
        // pose.x = lastPose.x +  (deltaSR+deltaSL)/2.0 * cos(lastPose.angle + (deltaSR-deltaSL)/(4*half_wheel_base)) ;
        // pose.y = lastPose.y +  (deltaSR+deltaSL)/2.0 * sin(lastPose.angle + (deltaSR-deltaSL)/(4*half_wheel_base)) ;
        // pose.angle = lastPose.angle + (deltaSR-deltaSL)/(2*half_wheel_base);
         
        //Serial.println ("pose : " + (String) pose.x+ ", " +(String) pose.y+ ", " + (String) degrees(pose.angle)) ;
       

        //R is distance to robot center
        pose.x = lastPose.x + (deltaSR+deltaSL)/2.0  * cos( lastPose.angle ) ;
        pose.y = lastPose.y + (deltaSR+deltaSL)/2.0  * sin( lastPose.angle  ) ;
        pose.angle = lastPose.angle + (deltaSR-deltaSL)/(2*half_wheel_base);

        Serial.println ("pose : " + (String) pose.x+ ", " +(String) pose.y+ ", " + (String) degrees(pose.angle)) ;
  
        return pose;
    };

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