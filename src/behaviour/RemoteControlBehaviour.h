
#pragma once

#include <OSCMessage.h>
#include "./Behaviours.h"
#include "../motion/DifferentialKinematics.h"

namespace SmallRobots {


    class RemoteControlBehaviour : public Behaviour {
        public:
            RemoteControlBehaviour(DifferentialKinematics& drive) : Behaviour(100), kinematics(drive) {
                osc_control.addCommand("/move", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = msg.getFloat(0);
                    if (msg.size() > 1) {
                        radius = msg.getFloat(1);
                        kinematics.move(speed, radius);
                    }
                    else {
                        radius = RADIUS_STREIGHT;
                        kinematics.move(speed);
                    }
                });
                osc_control.addCommand("/rotate", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    float speed = msg.getFloat(0);
                    radius = RADIUS_STREIGHT;
                    kinematics.rotate(speed);
                });
                osc_control.addCommand("/stop", [this](OSCMessage& msg) {
                    lastCommand = millis();
                    speed = 0;
                    radius = RADIUS_STREIGHT;
                    kinematics.stop();
                });
            };
            virtual Behaviour* run() {
                unsigned long now = millis();
                if (now - lastCommand > idleTime && speed != 0) {
                    speed = 0;
                    radius = RADIUS_STREIGHT;
                    kinematics.stop();
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
            DifferentialKinematics& kinematics;

            float speed = 0;
            float radius = RADIUS_STREIGHT;
    };

};  // namespace SmallRobots
