
# SmallRobotControl

`class SmallRobotControl` provides a simple mechanism for controlling the robot from external sources. It's main features are:

- centrally registering and dispatching commands for your robot
- ability to set the robot's parameters (see [Config](../config/))
- matching incoming messages based on tags applied to the robot
- providing commands to modify the robot's tags

Commands are intended to provide a way for external components to "invoke" or "trigger" functionality on the robot. They are different from parameters in that commands cause something to happen and are then forgotten, unlike parameters which can be set, and then retain their value in memory for later use.

Commands are not generally needed within the firmware on the robot, since other parts of the firmware can just call methods directly. Commands instead provide access to firmware functions to the outside.

Parameters are described in the [Config](../config/). The SmallRobotControl processes incoming "params" messages and dispatches them to the Config to modify parameters.

Tags and tagging of robots is the system used to the address the robots when working with multiple robots in a swarm or team of robots. The method of matching incoming messages to the tags is described in more detail below.

A global instance of SmallRobotControl is defined as `robot_control`. This global singleton instance can be used anywhere you include the SmallRobotControl.h header.

## OSC

Messages are sent to the robot as OSC (open sound control) messages. OSC is a kind of standard, and provides a message formatting and parsing infrastructure, with client libraries for many languages. It can use UDP over the network for transporting messages and handling framing, and suggests the use of SLIP for framing when sending messages across streams (Serial, TCP).

The SmallRobots networking code (for example for ESP32) configures UDP and/or Multicast sockets to listen for OSC messages, and forwards these to the `robot_control`. Be aware that these messages are processed asynchronously.

## Commands

Add your commands to the global instance, providing a callback (for example as a lambda function) which implements your command. Consider that the commands may execute asynchronously to other parts of your code.

Use it like this:

```c++
#include "control/SmallRobotControl.h"

// example using a lambda function as command handler

// add a command, supplying the callback function as a lambda
robot_control.addCommand("my-command", "Tf", [](OSCMessage& cmd){
    Serial.println("'my-command' was invoked.");
});


// example using a class with a command handler method

// the class declaration should go in a header file
class MyClass {
    public:
      void myCmdHandler(OSCMessage& cmd);
};

MyClass myObject;

// add a command, supplying the callback function using bind
robot_control.addCommand("my-other-cmd", "ii", std::bind(&MyClass::myCmdHandler, &myObject, std::placeholders::_1));

```

## Tags

