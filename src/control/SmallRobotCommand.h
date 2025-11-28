#pragma once

#include <functional>
#include <Arduino.h>
#include <OSCMessage.h>

namespace SmallRobots {

    /**
     * SmallRobotCommand
     * 
     * Represents a command that can be invoked via OSC messages
     * Used by both SmallRobotControl (UDP) and SmallRobotControlSlipSerial (SLIP Serial)
     */
    class SmallRobotCommand {
    public:
        String name;
        std::function<void(OSCMessage&)> callback;
    };

}
