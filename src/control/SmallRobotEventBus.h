
#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>
#include <tuple>
#include <variant>

namespace SmallRobots {

    // template<typename... Ts>
    // struct NrArguments {
    //     static const int value = sizeof...(Ts);
    // };

    template<typename... Ts>
    class EventBus {
        using _first = std::tuple_element_t<0, std::tuple<Ts...>>;
    public:
        EventBus() = default;
        virtual ~EventBus() = default;
    
        void emit(_first event) requires (sizeof...(Ts) == 1);

        template<typename T>
        void emit(T event) requires (sizeof...(Ts) > 1);

        void on(std::function<void(_first)> callback) requires (sizeof...(Ts) == 1);
        void on(_first val, std::function<void(_first)> callback) requires (sizeof...(Ts) == 1);

        void on(std::function<void(std::variant<Ts...>)> callback) requires (sizeof...(Ts) > 1);        
        template<typename T>
        void on(std::function<void(T)> callback) requires (sizeof...(Ts) > 1);
        template<typename T>
        void on(T val, std::function<void(T)> callback) requires (sizeof...(Ts) > 1);

    protected:
        std::map<String, std::vector<std::function<void(String)>>> callbacks;

        void dispatch(std::variant<Ts...> event) {
            auto it = callbacks.find(typeid(event).name());
            if (it != callbacks.end()) {
                for (auto& callback : it->second) {
                    callback(std::get<0>(event));
                }
            }
        }
    };


};

