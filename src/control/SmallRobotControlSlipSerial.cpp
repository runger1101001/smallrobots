#include "./SmallRobotControlSlipSerial.h"
#include "SmallRobotDebug.h"

namespace SmallRobots {

    void SmallRobotControlSlipSerial::init() {
        // Initialization hook for any setup needed
    }

    void SmallRobotControlSlipSerial::addCommand(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        _commands[name] = cmd;
    }

    void SmallRobotControlSlipSerial::run() {
        // Check for timeout on incomplete SLIP packet
        if (_in_slip_packet && millis() - _packet_start_time > SLIP_PACKET_TIMEOUT) {
            _in_slip_packet = false;
        }
        
        // Check if data is available
        if (_slip_serial.available()) {
            if (!_in_slip_packet) {
                _in_slip_packet = true;
                _packet_start_time = millis();
            }
        }
    }

    void SmallRobotControlSlipSerial::send(OSCMessage& msg) {
        _slip_serial.beginPacket();
        msg.send(_slip_serial);
        _slip_serial.endPacket();
    }

    void SmallRobotControlSlipSerial::onPacket(OSCMessage& msg) {
        String address = msg.getAddress();
        
        // Look for exact match in registered commands
        if (_commands.find(address) != _commands.end()) {
            _commands[address].callback(msg);
        }
    }

}

