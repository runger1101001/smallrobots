
#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>
#include <tuple>
#include <variant>
#include <stdexcept>
#include <iostream>
#include <source_location>

namespace SmallRobots {

    template<typename T>
    class EventBus { // TODO move me to a seperate header file so it can be included separately
        public:
            virtual void emit(T&& event) = 0;
            virtual void on(std::function<void(T)> callback) = 0;
            virtual void on(T val, std::function<void(T)> callback) = 0;
    };

    template<typename T, typename... Ts>
    class EventBusBase : public EventBus<T> {
        public:
            void emit(T&& event) override {
                queue(event);
            };
            void on(std::function<void(T)> callback) override {
                callbacks.push_back(callback);
            };
            void on(T val, std::function<void(T)> callback) override {
                callbacks.push_back([&](T event) {
                    Serial.println("Comparing value");
                    std::cout << __PRETTY_FUNCTION__ << std::endl;
                    Serial.println(val);
                    Serial.println(event);
                    if (val == event) {
                        Serial.println("Value matches");
                        callback(event);
                    }
                });
            };
        protected:
            virtual void queue(std::variant<Ts...> event) = 0;
            std::vector<std::function<void(T)>> callbacks;
            void dispatch(T& event) {
                Serial.println("Dispatching event 2");
                for (auto& callback : callbacks) {
                    callback(event);
                }
            };
    };




    template<template<typename...> class D, typename... Ts>
    class TEventBus : public EventBusBase<Ts, Ts...>... {
    public:
        TEventBus() = default;
        virtual ~TEventBus() = default;

        template<typename T>
        EventBus<T>& as() {
            return *static_cast<EventBus<T>*>(this);
        };
    
        using EventBusBase<Ts, Ts...>::dispatch...;
        using EventBusBase<Ts, Ts...>::emit...;
        using EventBusBase<Ts, Ts...>::on...;

        void run() {
            int num = dispatcher.size();
            while (num-- > 0) {
                std::variant<Ts...> event = dispatcher.next();
                dispatchToListeners(event);
            }
        };
        void dispatchToListeners(std::variant<Ts...>& event) {
            Serial.println("Dispatching event");
            std::cout << "Size of variant: " << sizeof(event) << " str: " << sizeof(String) << std::endl;
            std::visit([this](auto&& arg) {
                std::cout << __PRETTY_FUNCTION__ << std::endl;
                dispatch(arg);
            }, event);
        };
    protected:
        D<Ts...> dispatcher{*this};
        void queue(std::variant<Ts...> event) override {
            Serial.println("Queueing event");
            dispatcher.queue(event);
        };
    };



    template<typename... Ts>
    class ImmediateDispatcher {
        public:
            ImmediateDispatcher(TEventBus<ImmediateDispatcher, Ts...>& eventbus) : eventbus(eventbus) {};
            void queue(std::variant<Ts...>& event) {
                eventbus.dispatchToListeners(event);
            };
            int size() {
                return 0;
            };
            std::variant<Ts...> next() {
                throw std::runtime_error("No events in queue");
            };
        protected:
            TEventBus<ImmediateDispatcher, Ts...>& eventbus;
    };


    template<typename... Ts>
    class QueuedDispatcher {
        public:
            QueuedDispatcher(TEventBus<QueuedDispatcher, Ts...>& eventbus) {};
            void queue(std::variant<Ts...>& event) {
                q.push_back(event);
            };
            int size() {
                return q.size();
            };
            std::variant<Ts...> next() {
                auto event = q.front();
                q.erase(q.begin());
                return event;
            };
        protected:
            std::vector<std::variant<Ts...>> q;
    };




    typedef TEventBus<ImmediateDispatcher, String> SimpleEventBus;
    
};

