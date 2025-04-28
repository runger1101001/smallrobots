
#include <control/SmallRobotEventBus.h>

using namespace SmallRobots;

enum EventType1 : uint8_t { t1_1, t1_2, t1_3, t1_4 };

enum EventType2 : uint8_t { t2_1, t2_2, t2_3, t2_4 };

class CustomEvent {
public:
    EventType1 type;
    int value1;
};

TEventBus<ImmediateDispatcher, EventType1> eventBus1;

TEventBus<QueuedDispatcher, EventType2> eventBus2;

TEventBus<QueuedDispatcher, EventType1, EventType2, CustomEvent> eventBus3;

void setup() {
    Serial.begin(115200);
    delay(1000);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Event Bus Test");

    eventBus1.on([](EventType1 event) {
        Serial.print("Bus 1: ");
        Serial.println(static_cast<int>(event));
    });
    eventBus2.on([](EventType2 event) {
        Serial.print("Bus 2: ");
        Serial.println(static_cast<int>(event));
    });
    eventBus3.on([](CustomEvent event) {
        Serial.print("Bus 3 Custom Event: ");
        Serial.print(static_cast<int>(event.type));
        Serial.print(", Value: ");
        Serial.println(event.value1);
    });
    eventBus3.on([](EventType1 event) {
        Serial.print("Bus 3 Type 1: ");
        Serial.println(static_cast<int>(event));
        eventBus1.emit(event);
    });
    eventBus3.on([](EventType2 event) {
        Serial.print("Bus 3 Type 2: ");
        Serial.println(static_cast<int>(event));
        eventBus2.emit(event);
    });

    Serial.println("Emitting events...");
    // emit some events
    eventBus3.emit(EventType1::t1_1);
    eventBus3.emit(EventType2::t2_1);
    eventBus3.emit(CustomEvent{EventType1::t1_2, 42});
    eventBus2.emit(EventType2::t2_2);
    eventBus1.emit(EventType1::t1_2);
    eventBus3.emit(EventType1::t1_3);
    Serial.println("Setup complete.");
}

void loop() {
    // Check if there are any events in the queue
    eventBus1.run();
    eventBus2.run();
    eventBus3.run();
    // expect output:
    // Event Bus Test
    // Emitting events...
    // Bus 1: 1                             - output already in setup
    // Setup complete.
    // Bus 2: 1                             - first output in main loop
    // Bus 3 Type 1: 0                      - bus 3 event of type 1
    // Bus 1: 0                             - forwarded to bus 1 and immediately output
    // Bus 3 Type 2: 0                      - bus 3 event of type 2
    // Bus 3 Custom Event: 1, Value: 42     - bus 3 custom event
    // Bus 3 Type 1: 2                      - last event generated in setup
    // Bus 1: 2                             - forwarded to bus 1 and immediately output
    // Bus 2: 0                             - finally bus 2 event that was forwarded from bus 3
}