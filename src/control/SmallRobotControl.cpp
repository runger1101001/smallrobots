#include "./SmallRobotControl.h"
#include "../config/SmallRobotConfig.h"
#include "./SmallRobotEventBus.h"
#include "SmallRobotDebug.h"
#include <OSCMessage.h>

namespace SmallRobots {


    // global instance
    SmallRobotControl osc_control;

    void SmallRobotControl::init() {
        addCommand("tag", [](OSCMessage& tag){
            char buff[32];
            tag.getString(0, buff, 32);
            robot_config.tags += buff;
        });

        addCommand("untag", [](OSCMessage& tag){
            char buff[32];
            tag.getString(0, buff, 32);
            if (String("#all")==buff)
                return;
            #if defined(SMALLROBOT_SPECIES)
            if ((String("#")+SMALLROBOT_SPECIES)==buff)
                return;
            #endif
            robot_config.tags -= buff;
        });
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
                tagstr = String(&msg.getAddress()[1],pos-1);

                if (all_match) {
                    if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Incoming OSC message tags: "+tagstr);
                    // process message
                    int pos2 = msg.match("/params", pos);
                    if (pos2>1) {
                        String param = String(&msg.getAddress()[pos2+pos+1]);
                        SmallRobotParameter& p = robot_config[param];
                        if (!(p == SmallRobotConfig::UNKNOWN_PARAM)) {
                            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("OSC param: "+param);
                            if (p.type==SmallRobotParameterType::T_FLOAT) // TODO check that param and message types match
                                p = msg.getFloat(0);
                            else if (p.type==SmallRobotParameterType::T_INT)
                                p = msg.getInt(0);
                            else if (p.type==SmallRobotParameterType::T_STRING) {
                                char buff[32];
                                if (msg.getString(0, buff, 32)){
                                    p = String(buff);
                                }
                            }
                            else if (p.type==SmallRobotParameterType::T_BOOL) {
                                p = msg.getBoolean(0);
                            }
                            else {
                                if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown param type: "+param);
                            }
                            // TODO emit events event_bus.emit("p_"+param);
                        }
                        else
                            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown OSC param: "+param);
                        return; // we're done with the param message
                    }
                    // process behaviour message
                    pos2 = msg.match("/behaviour", pos);
                    if (pos2>1) {
                        String behaviour = String(&msg.getAddress()[pos2+pos]);
                        if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("OSC behaviour: "+behaviour);
                        // TODO process behaviour
                        return; // we're done with the behaviour message
                    }
                    // process data message
                    pos2 = msg.match("/data", pos);
                    if (pos2>1) {
                        String dataname = String(&msg.getAddress()[pos2+pos+1]);
                        // process data message

                        // First try exact match
                        if (data.find(dataname) != data.end()) { 
                            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC data: "+dataname);
                            data[dataname].callback(msg);
                        }
                        // Then try wildcard match (e.g., "fwd/*")
                        else {
                            bool found = false;
                            for (auto& entry : data) {
                                String pattern = entry.first;
                                if (pattern.endsWith("/*")) {
                                    String prefix = pattern.substring(0, pattern.length() - 2);
                                    if (dataname.startsWith(prefix + "/")) {
                                        if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC data (wildcard): "+dataname);
                                        entry.second.callback(msg);
                                        found = true;
                                        break;
                                    }
                                }
                            }
                            if (!found && debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown OSC data: "+dataname);
                        }
                        return; // we're done with the data message
                    }
                     // process sound message
                    pos2 = msg.match("/snd", pos);
                    if (pos2>1) {
                        String snd_command = String(&msg.getAddress()[pos2+pos+1]);
                        // process sound message
                        
                        // First try exact match
                        if (sound_commands.find(snd_command) != sound_commands.end()) { 
                            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC sound: "+snd_command);
                            sound_commands[snd_command].callback(msg);
                        }
                        // Then try wildcard match (e.g., "fwd/*")
                        else {
                            bool found = false;
                            for (auto& entry : sound_commands) {
                                String pattern = entry.first;
                                if (pattern.endsWith("/*")) {
                                    String prefix = pattern.substring(0, pattern.length() - 2);
                                    if (snd_command.startsWith(prefix + "/")) {
                                        if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC sound (wildcard): "+snd_command);
                                        entry.second.callback(msg);
                                        found = true;
                                        break;
                                    }
                                }
                            }
                            if (!found && debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown OSC sound: "+snd_command);
                        }
                        return; // we're done with the sound message
                    }
                    // process control message
                    pos2 = msg.match("/command", pos);
                    if (pos2>1) {
                        String command = String(&(msg.getAddress()[pos2+pos+1]));
                        // process command

                        // First try exact match
                        if (commands.find(command) != commands.end()) { 
                            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC command: "+command);
                            commands[command].callback(msg);
                        }
                        // Then try wildcard match (e.g., "fwd/*")
                        else {
                            bool found = false;
                            for (auto& entry : commands) {
                                String pattern = entry.first;
                                if (pattern.endsWith("/*")) {
                                    String prefix = pattern.substring(0, pattern.length() - 2);
                                    if (command.startsWith(prefix + "/")) {
                                        if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Invoking OSC command (wildcard): "+command);
                                        entry.second.callback(msg);
                                        found = true;
                                        break;
                                    }
                                }
                            }
                            if (!found && debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown OSC command: "+command);
                        }
                        return; // we're done with the control message
                    }
                    if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unknown OSC address: "+String(msg.getAddress()));
                } // if (all_match)
                else {
                    if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("OSC message tags N/A: "+tagstr);
                }

            } // if (pos>1)
        } // if (!hasError())
        else {
            // log error
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("OSC message error: "+msg.getError());
        }
    };



    void SmallRobotControl::addCommand(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        commands[name] = cmd;
    };

    void SmallRobotControl::addData(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        data[name] = cmd;
    };

    void SmallRobotControl::addSoundCommand(String name, std::function<void(OSCMessage&)> callback) {
        SmallRobotCommand cmd;
        cmd.name = name;
        cmd.callback = callback;
        sound_commands[name] = cmd;
    };

};

