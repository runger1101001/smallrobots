
#include "./SmallRobotEventBus.h"

namespace SmallRobots {


    EventBus event_bus;


    void EventBus::emit(String event) {
        // TODO disconnect emit from callbacks via a queue?
        // TODO send events to state machines automatically?
        if (callbacks.find(event) != callbacks.end()) {
            for (auto &callback : callbacks[event]) {
                callback(event);
            }
        }
        if (callbacks.find("*") != callbacks.end()) {
            for (auto &callback : callbacks["*"]) {
                callback(event);
            }
        }
    };

    void EventBus::on(String event, std::function<void(String)> callback) {
        if (callbacks.find(event) == callbacks.end()) {
            callbacks[event] = std::vector<std::function<void(String)>>();
        }
        callbacks[event].push_back(callback);
    };

};
