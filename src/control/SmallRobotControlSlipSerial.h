#pragma once

#include <map>
#include <functional>
#include <Arduino.h>
#include <OSCMessage.h>
#include <SLIPEncodedSerial.h>
#include "SmallRobotCommand.h"

namespace SmallRobots {

    /**
     * SmallRobotControlSlipSerial
     * 
     * OSC control interface for SLIP Serial communication
     * Works with SLIPEncodedSerial for sending and receiving OSC messages
     * 
     * Can be used on both sender (C6) and receiver (Pico) sides
     * 
     * Usage:
     *   SLIPEncodedSerial slip_serial(Serial);
     *   SmallRobotControlSlipSerial osc_control(slip_serial);
     *   osc_control.init();
     *   osc_control.addCommand("/move", [](OSCMessage& msg) { ... });
     *   
     *   In loop:
     *   osc_control.run();  // receive and process messages
     *   
     *   To send:
     *   OSCMessage msg("/move");
     *   msg.add(velocity).add(radius);
     *   osc_control.send(msg);
     */
    class SmallRobotControlSlipSerial {
    public:
        static const int OSC_BUFFER_SIZE = 256;

        SmallRobotControlSlipSerial(SLIPEncodedSerial& slip_serial) 
            : _slip_serial(slip_serial) {}

        void init();
        
        void addCommand(String name, std::function<void(OSCMessage&)> callback);

        void run();
        
        void send(OSCMessage& msg);
        
        void onPacket(OSCMessage& msg);

        bool debug = false;

    private:
        SLIPEncodedSerial& _slip_serial;
        uint8_t _osc_buffer[OSC_BUFFER_SIZE];
        std::map<String, SmallRobotCommand> _commands;
    };

}
