
#pragma once

#include <Arduino.h>
#include <map>
#include <set>

#include "../debug/SmallRobotsDebug.h"



namespace SmallRobots {


    typedef enum : uint8_t {
        T_UNKNOWN,
        T_FLOAT,
        T_INT,
        T_STRING,
        T_BOOL
    } SmallRobotParameterType;
    

    class SmallRobotParameter { // TODO this should be a template class
    friend class SmallRobotConfig;
    public:
        SmallRobotParameter(const char* name, float value, std::function<void(float)> onchange = nullptr) {
            this->name = name;
            this->type = SmallRobotParameterType::T_FLOAT;
            f = value;
            new (&this->onchange_f) std::function<void(float)>(std::move(onchange));
        };
        SmallRobotParameter(const char* name, int value, std::function<void(int)> onchange = nullptr) {
            this->name = name;
            this->type = SmallRobotParameterType::T_INT;
            i = value;
            new (&this->onchange_i) std::function<void(int)>(std::move(onchange));
        };
        SmallRobotParameter(const char* name, String value, std::function<void(String*)> onchange = nullptr) {
            this->name = name;
            this->type = SmallRobotParameterType::T_STRING;
            s = new String(value);
            new (&this->onchange_s) std::function<void(String*)>(std::move(onchange));
        };
        SmallRobotParameter(const char* name, bool value, std::function<void(bool)> onchange = nullptr) {
            this->name = name;
            this->type = SmallRobotParameterType::T_BOOL;
            b = value;
            new (&this->onchange_b) std::function<void(bool)>(std::move(onchange));
        };
        ~SmallRobotParameter() {
            switch (type) {
                case T_FLOAT:  onchange_f.~function(); break;
                case T_INT:    onchange_i.~function(); break;
                case T_BOOL:   onchange_b.~function(); break;
                case T_STRING: onchange_s.~function(); if (s != nullptr) delete s; break;
                default: break;
            }
        };

        operator float() const {
            return f;
        };

        operator String() const {
            return *s;
        };

        operator int() const {
            return i;
        };

        operator bool() const {
            return b;
        };


        SmallRobotParameter& operator=(float value) {
            if (debug) smallrobots_debug.printf("Setting float param %s to %f\n", name, value);
            if (onchange_f && value != f)
                onchange_f(value);
            f = value;
            if (onAnyParamChanged) onAnyParamChanged(name);
            return *this;
        };


        SmallRobotParameter& operator=(int32_t value) {
            if (debug) smallrobots_debug.printf("Setting int param %s to %d\n",name , value);
            if (onchange_i && value != i)
                onchange_i(value);
            i = value;
            if (onAnyParamChanged) onAnyParamChanged(name);
            return *this;
        };


        SmallRobotParameter& operator=(String value) {
            if (debug) smallrobots_debug.printf("Setting string param %s to %s\n", name, value.c_str());
            bool changed = false;
            if (onchange_s && value != *s)
                changed = true;
            *s = value;
            if (onchange_s && changed)
                onchange_s(s);
            if (onAnyParamChanged) onAnyParamChanged(name);
            return *this;
        };


        SmallRobotParameter& operator=(bool value) {
            if (debug) smallrobots_debug.printf("Setting bool param %s to %d\n", name, value);
            if (onchange_b && value != b)
                onchange_b(value);
            b = value;
            if (onAnyParamChanged) onAnyParamChanged(name);
            return *this;
        };


        bool operator== (const SmallRobotParameter& other) {
            return this == &other;
        };

        bool operator!= (const SmallRobotParameter& other) {
            return this != &other;
        };

        SmallRobotParameterType type;
        const char* name;

        // TODO change to template class
        union {
            std::function<void(float)> onchange_f = nullptr;
            std::function<void(int)> onchange_i;
            std::function<void(String*)> onchange_s;
            std::function<void(bool)> onchange_b;
        };
        union {
            float f;
            int32_t i;
            String* s;
            bool b;
        };

        inline static bool debug = false;
        inline static std::function<void(const char*)> onAnyParamChanged = nullptr;
    private:

        SmallRobotParameter(const char* name, SmallRobotParameterType type = SmallRobotParameterType::T_UNKNOWN) {
            this->name = name;
            this->type = type;
            if (type==SmallRobotParameterType::T_STRING) {
                s = new String();
            }
            this->onchange_f = nullptr;
        };
    };




    class SmallRobotTags {
    public:

        SmallRobotTags& operator+=(const char* tag) {
            tags.insert(String(tag));
            if (onTagsChanged) onTagsChanged();
            return *this;
        };

        SmallRobotTags& operator+=(String& tag) {
            tags.insert(tag);
            if (onTagsChanged) onTagsChanged();
            return *this;
        };

        SmallRobotTags& operator-=(const char* tag) {
            tags.erase(String(tag));
            if (onTagsChanged) onTagsChanged();
            return *this;
        };

        // adds a tag that survives clear() (e.g. #all, #<species>, #<mac address>)
        SmallRobotTags& keep(String tag) {
            permanent.insert(tag);
            tags.insert(tag);
            if (onTagsChanged) onTagsChanged();
            return *this;
        };

        bool isPermanent(String tag) {
            return permanent.find(tag) != permanent.end();
        };

        // removes all tags except the permanent ones
        void clear() {
            for (auto it = tags.begin(); it != tags.end(); ) {
                if (permanent.find(*it) != permanent.end())
                    ++it;
                else
                    it = tags.erase(it);
            }
            if (onTagsChanged) onTagsChanged();
        };

        inline static std::function<void()> onTagsChanged = nullptr;

        bool operator[](String tag) {
            return tags.find(tag) != tags.end();
        };

        void forEach(std::function<void(String)> func) {
            for (auto i = tags.begin(); i != tags.end(); ++i) {
                func(*i);
            }
        };

    private:
        std::set<String> tags;
        std::set<String> permanent;

    };



    class SmallRobotConfig {
    public:
        SmallRobotConfig();
        ~SmallRobotConfig();

        SmallRobotParameter& add(const char* name, float value, std::function<void(float)> onchange = nullptr);
        SmallRobotParameter& add(const char* name, int value, std::function<void(int)> onchange = nullptr);
        SmallRobotParameter& add(const char* name, String value, std::function<void(String*)> onchange = nullptr);
        SmallRobotParameter& add(const char* name, bool value, std::function<void(bool)> onchange = nullptr);

        SmallRobotParameter& operator[](const char* key);
        SmallRobotParameter& operator[](String& key);

        void forEach(std::function<void(const char*, SmallRobotParameter&)> func) {
            for (auto i = params.begin(); i != params.end(); ++i) {
                func(i->second->name, *i->second);
            }
        };

        SmallRobotTags tags;
        bool debug = false;
    
        static SmallRobotParameter UNKNOWN_PARAM;

    private:
        std::map<String, SmallRobotParameter*> params;
    };


    extern SmallRobots::SmallRobotConfig robot_config;

};

