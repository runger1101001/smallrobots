
#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>


namespace SmallRobots {

    class EventBus {

    public:
        EventBus() = default;
        virtual ~EventBus() = default;
    

        // TODO change event type to a class that can also hold the parameters
        void emit(String event);
        void on(String event, std::function<void(String)> callback);

    private:
        std::map<String, std::vector<std::function<void(String)>>> callbacks;
    };


    extern SmallRobots::EventBus event_bus;

};

