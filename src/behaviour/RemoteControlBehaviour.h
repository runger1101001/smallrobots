
#pragma once

#include <OSCMessage.h>
#include "./Behaviours.h"
#include "../motion/DifferentialKinematics.h"
//#include "FastLED.h"
#include "motion/MotionStateMachine.h"

// extern CRGB externalRGB;
extern u_int8_t externalR, externalG, externalB;


namespace SmallRobots {

    

    class RemoteControlBehaviour : public Behaviour {
        public:
            
            RemoteControlBehaviour(MotionController& _ctrl ) : Behaviour(100), ctrl (_ctrl)
            {
            //RemoteControlBehaviour( DifferentialPathPlanner& drive_pp ) : Behaviour(100)
            //{
            //RemoteControlBehaviour(DifferentialKinematics& drive, MotionStateMachine& pns) : Behaviour(100), kinematics(drive),motionStateMachine(pns) {

                osc_control.addCommand("move", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = msg.getFloat(0);
                    if (msg.size() > 1) {
                        radius = msg.getFloat(1);
                        this->ctrl.kinematics.move(speed, radius);
                    }
                    else {
                        radius = RADIUS_STREIGHT;
                        this->ctrl.kinematics.move(speed);
                    }
                    event_bus.emit("startOdometry");
                });
                osc_control.addCommand("rotate", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = msg.getFloat(0);
                    this->ctrl.kinematics.rotate(speed);
                });
                osc_control.addCommand("stop", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = 0.0f;
                    radius = RADIUS_STREIGHT;
                    this->ctrl.kinematics.stop();
                    event_bus.emit("pause");
                });

                osc_control.addCommand("enableMotors", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    this->ctrl.kinematics.enable();
                });

                osc_control.addCommand("disableMotors", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    this->ctrl.kinematics.disable();
                });

                osc_control.addCommand("resetOdometry", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    this->ctrl.setCurPose(Pose(0,0,0));
                    this->ctrl.deletePath();
                });

                 osc_control.addCommand("set_rgb", [this](OSCMessage& msg) {
                    
                   // externalRGB = CRGB( u_int8_t (msg.getInt(0)), u_int8_t ( msg.getInt(1)), u_int8_t ( msg.getInt(2)) );
                    externalR =  u_int8_t (msg.getInt(0));
                    externalG =  u_int8_t (msg.getInt(1));
                    externalB =  u_int8_t (msg.getInt(2));
              
                });

                osc_control.addCommand("setSpeed", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = msg.getFloat(0);
                    ctrl.setRobotVelocity(speed);   
                });

                osc_control.addCommand("setPathRadius", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    float R = msg.getFloat(0);
                    ctrl.setPathRadius(R);   
                });
       
                osc_control.addCommand("addPoseToPath", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    int state = msg.getInt(0);
                    x = msg.getFloat(1);
                    y = msg.getFloat(2);
                    angle = msg.getFloat(3);
                    
                    if (msg.size() > 4) {
                        speed = msg.getFloat(4);
                        ctrl.setRobotVelocity(speed);   
                    }
                    Pose pose = {x,y,angle};
                    this->ctrl.addPoseToPath(pose);
                    if (state == 1) event_bus.emit("set_new_pose");  
                });

                osc_control.addCommand("replacePathByPose", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    x = msg.getFloat(0);
                    y = msg.getFloat(1);
                    angle = msg.getFloat(2);
                    if (msg.size() > 3) {
                        speed = msg.getFloat(4);
                        ctrl.setRobotVelocity(speed);   
                    }
                    Pose pose = {x,y,angle};
                    this->ctrl.setPoseToReplacePath(pose);
                    event_bus.emit("set_new_pose");  
                    
                });

                osc_control.addCommand("addPoseListToPath", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    std::vector<Pose> newPath;
                    int nbOfPoses = msg.getInt(0);
                    int state = msg.getInt(1);
                    for (int i= 0; i<nbOfPoses; i+=3)
                    {
                        x = msg.getFloat(i+2);
                        y = msg.getFloat(i+3);
                        angle = msg.getFloat(i+4);
                                        
                        Pose pose = {x,y,angle};
                        newPath.push_back(pose);

                    }
                        this->ctrl.addPoseListToPath(newPath);
                    if (state == 1) event_bus.emit("set_new_pose");  
                });
                
            };
            virtual Behaviour* run() {
                unsigned long now = millis();
                if (idleTime > 0 && now - lastCommand > idleTime && speed != 0) {
                    speed = 0;
                    radius = RADIUS_STREIGHT;
                    //kinematics.stop();
                }
                return this;
            };
            virtual const char* getName() override { return name.c_str(); };

            float getSpeed() { return speed; };
            float getRadius() { return radius; };

            unsigned long idleTime = 1000;
        protected:
            String name = "RemoteControlBehaviour";
            unsigned long lastCommand = 0;
            //DifferentialKinematics& kinematics;
            //DifferentialPathPlanner& pathplanner;
            MotionController& ctrl;
           

            float speed = 0;
            float radius = RADIUS_STREIGHT;

            //MotionStateMachine& motionStateMachine;

            //Pose: location x,y and global angle for orientation;
            float x = 0, y = 0, angle = 0;
    };


};  // namespace SmallRobots

