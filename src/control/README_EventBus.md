
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
typedef template TEventBus<ImmediateDispatcher, MyEvent> MyEventBus;

// create an event bus of your type
MyEventBus event_bus;

void some_setup_function() {
    // register an event listener as a lambda function
    event_bus.on([](MyEvent e){
        Serial.print("Received an event: ");
        Serial.println(e.toString()); // lets pretend it has a toString method
    });
}

// send an event to the bus
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
typedef template TEventBus<QueuedDispatcher, BatteryEvent, TemperatureEvent> MainEventBus;

// create the event bus instance
MainEventBus main_bus;

// register some listeners
void register_listeners() {
    // register a listener for battery events
    main_bus.on([](BatteryEvent e){
        // ...
    });
    // register a listener for only overtemp temperature events
    main_bus.on(TemperatureEvent::overtemp, [](TemperatureEvent e){
        // ...
    });
    // pass the event-bus to a component that only deals with temperatures
    temperature_indicator.useEventBus(main_bus.as<TemperatureEvent>());
}

// emit events
void temperature_monitor() {
    int temp = readTemperature(); // function that reads the sensor, just for example
    if (temp > THRESHOLD) // THRESHOLD is some constant, just for example
        main_bus.emit(TemperatureEvent::overtemp); // this is how we send an event
}


// main loop runs events
void loop() {
    // ...
    main_bus.run(); // process any events that have arrived since the last time we ran
}

```

## Event Dispatch

The event bus abstraction specifies that events which are emitted by some part of the code, are forwarded and eventually processed by the listnerers, in another part of the code. The manner in which events propagate from emitter to listener is not specified, except to say:
- all registered listeners receive every event for which they registered
- each listener receives the event in the form of its own seperate copy

In particular the contract does not specify:
- the order in which events are processed
- the order in which listeners are called with each event
- whether they are processed immediately, or later

This is controlled by the *dispatch* process, which is implemented seperately to the event bus itself. Different dispatchers exist, so events can be dispatched in several ways, and you can also supply your own dispatcher implementation. The following dispatchers are pre-defined:

- **ImmediateDispatcher** \
  Events are processed immediately when emitted. Since the processing of events can emit more events, which are in turn immediately processed, this will lead to a different order of events from the other dispatchers. It can also mean you can generate an infinite loop of handlers/emits, so be careful when using this dispatcher.
- **QueuedDispatcher** \
  Events are placed into a queue, and processed during the call to TEventBus.run(). Call run() from your main loop to process the events that have been emitted since the last time you called run. New events emitted will be placed at the end of the queue. Events emitted during the event processing will be processed on the subsequent call to run(), not the current one. This behaviour is much closer to what most users expect from an EventBus than the immediate dispatch. QueuedDispatcher is not thread-safe, and so only suitable for use in single-threaded environments.
- **LockedQueuedDispatcher** \
  Usable with FreeRTOS. Extends the QueuedDispatcher with locking for thread-safe use.
- **RTOSQueueDispatcher** \
  Uses a FreeRTOS Queue (rather than std::queue) to queue the events. Warning! Only suitable for event types which can be copied with memcpy.
- **RTOSTaskDispatcher** \
  Dispatches each event on its own new FreeRTOS task. This will lead to a different (and hard to predict) order for the processing of events. Within each event's task the listeners for that event are called sequentially.

Some notes:
- Generally, we expect events to be processed in the order they are generated. RTOSTaskDispatcher doesn't guarantee this, while ImmediateDispatcher processes the events/handlers in a depth-first way, wnich is unexpected. QueuedDispatcher, RTOSQueueDispatcher and LockedQueuedDispatcher process events in the expected order.
- ImmediateDispatcher processes emitted events immediately, RTOSTaskDispatcher processes them later, whenever their tasks are scheduled by FreeRTOS. The other dispatchers process the events when you call the TEventBus.run() method.
- Listeners are processed in order of their registration. So you can influence the order in which listeners are called by changing their order of registration.
- Note: the listener registration process (e.g. calling EventBus.on(...) ) is not thread-safe. (TODO should we provide a thread-safe version for use on ESP32?)

