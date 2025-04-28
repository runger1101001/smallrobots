
#pragma once

#include "../../control/SmallRobotEventBus.h"

#if defined(ESP32)

namespace SmallRobots {

    template<typename... Ts>
    class RTOSQueueDispatcher {
    public:
        RTOSQueueDispatcher(TEventBus<RTOSQueueDispatcher, Ts...>& eventbus) {
            q = xQueueCreate(10, sizeof(std::variant<Ts...>));
        };
        ~RTOSQueueDispatcher() {
            vQueueDelete(q);
        };
        void queue(std::variant<Ts...>& event) {
            xQueueSend(q, &event, 0);  
        };
        int size() {
            return uxQueueMessagesWaiting(q);
        };
        std::variant<Ts...> next() {
            std::variant<Ts...> event;
            if (xQueueReceive(q, &event, 0) == pdTRUE) {
                return event;
            }
            throw std::runtime_error("No events in queue");
        };
    protected:
        QueueHandle_t q;
    };

};

#endif // ESP32