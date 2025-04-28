
#pragma once

#include "../../control/SmallRobotEventBus.h"
#include "./Lock.h"

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


    template<typename... Ts>
    class RTOSTaskDispatcher {
    public:
        RTOSTaskDispatcher(TEventBus<RTOSTaskDispatcher, Ts...>& eventbus) : eventbus(eventbus) {
            // TODO make core and stack-size configurable
        };
        ~RTOSTaskDispatcher() {
        };
        void queue(std::variant<Ts...>& event) {
            std::tuple<std::variant<Ts...>, RTOSTaskDispatcher*> *tup = new std::tuple<>(event, this);
            xTaskCreateUniversal(
                [](void* arg) {
                    std::tuple<std::variant<Ts...>, RTOSTaskDispatcher*> *tup = static_cast<std::tuple<std::variant<Ts...>, RTOSTaskDispatcher*>>(arg);
                    tup[1]->eventbus.dispatchToListeners(tup[0]);
                    delete tup;
                    vTaskDelete(NULL);
                },
                "EventDispatcherTask",
                1024,
                this,
                1,
                NULL,
                -1
            );
        };
        int size() {
            return 0;
        };
        std::variant<Ts...> next() {
            throw std::runtime_error("No events in queue");
        };
    protected:
        TEventBus<RTOSTaskDispatcher, Ts...>& eventbus;
    };

    const char LOCKEDQUEUEDISPATCHER_LOCKNAME[] = "LockedQueuedDispatcher";
    typedef Lock<LOCKEDQUEUEDISPATCHER_LOCKNAME> LockedQueueDispatcherLock;

    template<typename... Ts>
    class LockedQueuedDispatcher {
        public:
            LockedQueuedDispatcher(TEventBus<LockedQueuedDispatcher, Ts...>& eventbus) {};
            void queue(std::variant<Ts...> event) {
                LockedQueueDispatcherLock lock;
                q.push_back(event);
            };
            int size() {
                LockedQueueDispatcherLock lock;
                return q.size();
            };
            std::variant<Ts...> next() {
                LockedQueueDispatcherLock lock;
                auto event = q.front();
                q.erase(q.begin());
                return event;
            };
        protected:
            std::vector<std::variant<Ts...>> q;
    };


};

#endif // ESP32