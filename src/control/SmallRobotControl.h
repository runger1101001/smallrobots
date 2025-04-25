
#pragma once


#include <AsyncUDP.h>


/**
 * OSC control interface for the robot...
 * 
 * Processes OSC messages of the following form:
 * 
 * /zooid/params/max_speed 100.0
 * /tag1.tag2.tag3/params/myparam 1.0
 * /all/command/reboot
 * /zooid.tag99/command/mycommand 1.0 6 99 "hello"
 * etc...
 * 
 * TODO You can shorten the OSC messages a bit:
 * 
 * /zooid/p/max_speed 100.0
 * 
 */

#include <map>
#include <functional>
#include <Arduino.h>
#include <OSCMessage.h>

namespace SmallRobots {


  class SmallRobotCommand {
    public:
      String name;
      std::function<void(OSCMessage&)> callback;
  };



  class SmallRobotControl {
  public:
    void init();
    void addCommand(String name, std::function<void(OSCMessage&)> callback);

    void onPacket(AsyncUDPPacket packet);

    bool debug = false;
  private:
    std::map<String, SmallRobotCommand> commands;
  };


  extern SmallRobotControl osc_control;

};
