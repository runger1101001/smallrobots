#include "./SmallRobotControlSlipSerial.h"
#include "SmallRobotDebug.h"

namespace SmallRobots {

    void SmallRobotControlSlipSerial::init() {
        // Initialization hook for any setup needed
        if (debug && smallrobot_debug_print != nullptr) {
            smallrobot_debug_print->println("SmallRobotControlSlipSerial initialized");
        }
    }

    void SmallRobotControlSlipSerial::addCommand(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        _commands[name] = cmd;
        
        if (debug && smallrobot_debug_print != nullptr) {
            smallrobot_debug_print->println("OSC command registered: " + name);
        }
    }

    void SmallRobotControlSlipSerial::run() {
        // Check if data is available from the SLIP serial connection
        if (_slip_serial.available()) {
            size_t packet_size = _slip_serial.readBytes(_osc_buffer, OSC_BUFFER_SIZE);
            if (packet_size > 0 && _slip_serial.endofPacket()) {
                OSCMessage msg;
                // Fill the message byte by byte from the buffer
                for (size_t i = 0; i < packet_size; i++) {
                    msg.fill(_osc_buffer[i]);
                }
                if (!msg.hasError()) {
                    onPacket(msg);
                } else {
                    if (debug && smallrobot_debug_print != nullptr) {
                        smallrobot_debug_print->println("OSC message error");
                    }
                }
            }
        }
    }

    void SmallRobotControlSlipSerial::send(OSCMessage& msg) {
        _slip_serial.beginPacket();
        msg.send(_slip_serial);
        _slip_serial.endPacket();
        
        if (debug && smallrobot_debug_print != nullptr) {
            smallrobot_debug_print->printf("OSC message sent to %s\n", msg.getAddress());
        }
    }

    void SmallRobotControlSlipSerial::onPacket(OSCMessage& msg) {
        String address = msg.getAddress();
        
        // Look for exact match in registered commands
        if (_commands.find(address) != _commands.end()) {
            if (debug && smallrobot_debug_print != nullptr) {
                smallrobot_debug_print->println("Invoking OSC command: " + address);
            }
            _commands[address].callback(msg);
        } else {
            if (debug && smallrobot_debug_print != nullptr) {
                smallrobot_debug_print->println("Unknown OSC command: " + address);
            }
        }
    }

}
