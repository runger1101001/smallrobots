
#pragma once


#include <AsyncUDP.h>


/**
 * OSC control interface for the robot...
 * 
 * Processes OSC messages of the following form:
 * 
 * /#zooid/params/max_speed 100.0
 * /#tag1#tag2#tag3/params/myparam 1.0
 * /#all/control/reboot
 * /#zooid#tag99/control/mycommand 1.0 6 99 "hello"
 * etc...
 * 
 * You can shorten the OSC messages a bit:
 * 
 * /#zooid/p/max_speed 100.0
 * 
 * Future improvement? Map to integers:
 * /#2/#24/p/17 100.0
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
  friend class SmallRobotEsp32Wifi;
  public:

    void start();
    void addCommand(String name, std::function<void(OSCMessage&)> callback);

    bool debug = false;
  private:
    void onPacket(AsyncUDPPacket packet);
    std::map<String, SmallRobotCommand> commands;
  };


  extern SmallRobots::SmallRobotControl osc_control;

};
