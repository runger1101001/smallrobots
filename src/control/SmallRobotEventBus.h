
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
            virtual void emit(T event) = 0;
            virtual void on(std::function<void(T)> callback) = 0;
            virtual void on(T val, std::function<void(T)> callback) = 0;
    };

    template<typename T, template<typename...> class D, typename... Ts>
    class EventBusWrapper;


    template<template<typename...> class D, typename... Ts>
    class TEventBus {
    public:
        TEventBus() = default;
        virtual ~TEventBus() = default;

        template<typename T>
        EventBus<T>& as() {
            return std::get<EventBusWrapper<T, D, Ts...>>(wrappers);
        };

        template<typename T>
        void on(T val, std::function<void(T)> callback) {
            std::get<std::vector<std::function<void(T)>>>(callbacks).push_back([val, callback](T event){
                if (val == event) {
                    callback(event);
                }
            });
        };

        template<typename T>
        void on(std::function<void(T)> callback) {
            std::get<std::vector<std::function<void(T)>>>(callbacks).push_back(callback);
        };

        template<typename T>
        void emit(T event) {
            dispatcher.queue(event);
        };

        void run() {
            int num = dispatcher.size();
            while (num-- > 0) {
                std::variant<Ts...> event = dispatcher.next();
                dispatchToListeners(event);
            }
        };

        // void dispatchToListeners(std::variant<Ts...>& event) {
        //     Serial.println("Dispatching event");
        //     std::visit([&](auto&& arg) {
        //         dispatch(arg);
        //     }, event);
        // };



    protected:
        D<Ts...> dispatcher{*this};
        std::tuple<std::vector<std::function<void(Ts)>>...> callbacks = { std::vector<std::function<void(Ts)>>{}... };
        std::tuple<EventBusWrapper<Ts, D, Ts...>...> wrappers = { EventBusWrapper<Ts, D, Ts...>(*this)... };
        
        void dispatchToListeners(std::variant<Ts...>& event) {
            std::visit([&](auto&& arg) {
                dispatch(arg);
            }, event);
        };

        template<typename T>
        void dispatch(T& event) {
            std::vector<std::function<void(T)>>& cbs = std::get<std::vector<std::function<void(T)>>>(callbacks);
            for (std::function<void(T)>& callback : cbs) {
                callback(event);
            }
        };
    };




    template<typename T, template<typename...> class D, typename... Ts>
    class EventBusWrapper : public EventBus<T> {
        public:
            EventBusWrapper(TEventBus<D, Ts...>& eventbus) : eventbus(eventbus) {};
            void emit(T event) override {
                eventbus.emit(event);
            };
            void on(std::function<void(T)> callback) override {
                eventbus.on(callback);
            };
            void on(T val, std::function<void(T)> callback) override {
                eventbus.on(val, callback);
            };
        protected:
            TEventBus<D, Ts...>& eventbus;
    };





    template<typename... Ts>
    class ImmediateDispatcher {
        public:
            ImmediateDispatcher(TEventBus<ImmediateDispatcher, Ts...>& eventbus) : eventbus(eventbus) {};
            void queue(std::variant<Ts...> event) {
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
            void queue(std::variant<Ts...> event) {
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




    typedef TEventBus<QueuedDispatcher, String> SimpleEventBus;
    
};

