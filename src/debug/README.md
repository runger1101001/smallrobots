# SmallRobots Debug

This folder contains simple classes to help with debugging.

They're not intended to replace telemetry or control, but just to provide debug messages. Typically debug messages are passed over Serial or SWD, but for a mobile robot, debug messages over WiFi might be more practical.

SmallRobotsDebug provides the following functionality:

- add multiple debug `Print*` streams. `Print*` is implemented by Serial, but also USB, SlipSerial, etc...
- enable and disable them individually
- output is flushed after newline characters
- other flushes are suppressed

