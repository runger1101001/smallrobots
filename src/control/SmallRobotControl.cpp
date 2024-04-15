
#include "./SmallRobotControl.h"
#include "./SmallRobotConfig.h"
#include "./SmallRobotEventBus.h"
#include <OSCMessage.h>
#include "services/isp/PicoUpdateService.h"

namespace SmallRobots {


    // global instance
    SmallRobotControl osc_control;



    void SmallRobotControl::start() {
        // initialize OSC server -- currently this is in the WiFI class, but it should probably move here
    };


    void SmallRobotControl::onPacket(AsyncUDPPacket packet){
        // parse OSC message
        OSCMessage msg;
        msg.fill(packet.data(), packet.length());
        if (!msg.hasError()) {
            // process OSC message
            int pos = msg.match("/*");
            if (pos>1) {
                String tagstr = String(&msg.getAddress()[1],pos-1);
                int more;
                bool all_match = true;
                do {
                    String tag;
                    more = tagstr.lastIndexOf('#');
                    if (more>0) {
                        tag = tagstr.substring(more);
                        tagstr = tagstr.substring(0,more);
                    }
                    else {
                        tag = tagstr;
                    }
                    // process tag
                    if (!robot_config.tags[tag]) {
                        all_match = false;
                        break;
                    }
                } while(more > 0);

                if (all_match) {
                    if (debug) Serial.println("Incoming OSC message tags: "+tagstr);
                    // process message
                    int pos2 = msg.match("/params", pos);
                    if (pos2>=pos) {
                        String param = String(&msg.getAddress()[pos2+6]);
                        SmallRobotParameter& p = robot_config[param];
                        if (p != SmallRobotConfig::UNKNOWN_PARAM) {
                            if (debug) Serial.println("OSC param: "+param);
                            if (p.type==SmallRobotParameterType::T_FLOAT) // TODO move this to Param class, don't expose type here
                                p = msg.getFloat(0);
                            else if (p.type==SmallRobotParameterType::T_INT)
                                p = msg.getInt(0);
                            else if (p.type==SmallRobotParameterType::T_STRING) {
                                char buff[32];
                                if (msg.getString(0, buff, 32)){
                                    p = String(buff);
                                }
                            }
                            event_bus.emit("p_"+param);
                        }
                        else
                            if (debug) Serial.println("Unknown OSC param: "+param);
                        return; // we're done with the param message
                    }
                    // process behaviour message
                    pos2 = msg.match("/behaviour", pos);
                    if (pos2>=pos) {
                        String behaviour = String(&msg.getAddress()[pos2+10]);
                        if (debug) Serial.println("OSC behaviour: "+behaviour);
                        // TODO process behaviour
                        return; // we're done with the behaviour message
                    }
                    // process control message
                    pos2 = msg.match("/command", pos);
                    if (pos2>=pos) {
                        String command = String(&msg.getAddress()[pos2+8]);
                        // process command
                        if (commands.find(command) != commands.end()) { 
                            if (debug) Serial.println("Invoking OSC command: "+command);
                            commands[command].callback(msg);
                        }
                        else
                            if (debug) Serial.println("Unknown OSC command: "+command);
                        return; // we're done with the control message
                    }
                    if (debug) Serial.println("Unknown OSC address: "+String(msg.getAddress()));
                } // if (all_match)
                else {
                    if (debug) Serial.println("OSC message tags N/A: "+tagstr);
                }

            } // if (pos>1)
        } // if (!hasError())
        else {
            // log error
            Serial.println("OSC message error: "+msg.getError());
        }
    };



    void SmallRobotControl::addCommand(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        commands[name] = cmd;
    };

};

