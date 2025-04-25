
# EventBus

The event bus provides an important tool in the SmallRobots architecture. It is the foundation for implementing components in a decoupled, independent way. The abstraction is fairly simple:

- components can emit events (at any time)
- components can register to listen on (receive) events
- multiple components can listen to the same events, and events are delivered to all listeners
- different models can be used for event delivery (synchronous, asynchronous, see below)
- events are fixed size and copied to each recipient (no problems with deallocation, see below)

Multiple event buses can be created, and they can support different event types.

## Usage

A simple example, supporting a single event type. Note how pass by value semantics means the event is automatically destructed when it leaves the scope of the receiver method.

```c++
#include <control/SmallRobotEventBus.h>

// some kind of event type, keep it small
class MyEvent;

// define event bus type
typedef template EventBus<MyEvent> MyEventBus;

// create an event bus of your type
MyEventBus event_bus;

void some_setup_function() {
    // register an event listener as a lambda function
    event_bus.on([](MyEvent e){
        Serial.print("Received an event: ");
        Serial.println(e.toString()); // lets pretend it has a toString method
    });
}


void some_other_function(){
    event_bus.emit(MyEvent(9, 6.3f));
}
```

## Event types

Events are passed by value, and therefore copied to the different listeners that receive them. This implies that events should be kept small in size, and copying the types must be possible.

Users of the event bus can consider different models for representing the transported events. A simple model would certainly be to have a common event class used by all components using the bus. Such a class would most likely contain information to determine the type of event and any values/parameters associated with the event. It would also allow simple event buses where the events carry only a type, or all events have the same type and carry only a value.

The event bus also supports using multiple types on the same event bus, providing all types are compatible with the requirements (copyable, not too large). It provides type-safe semantics for such cases:

```c++
// events for the battery
enum BatteryEvent : uint8_t { full, low, empty, charging };
// events for temperature sensor
enum TemperatureEvent : uint8_t { overtemp, normal, undertemp };

// define an event bus which handles multiple event types
typedef template EventBus<BatteryEvent,TemperatureEvent> MainEventBus;

// create the event bus instance
MainEventBus main_bus;

// register some listeners
void register_listeners() {
    // register a listener for battery events
    main_bus.on<BatteryEvent>([](BatteryEvent e){
        // ...
    });
    // register a listener for temperature events
    main_bus.on<TemperatureEvent>([](TemperatureEvent e){
        // ...
    });
}


// emit events
void temperature_monitor() {
    // handle temperature sensor...
    if (temp > threshold)
        main_bus.emit<TemperatureEvent>(TemperatureEvent::overtemp);
}
```

