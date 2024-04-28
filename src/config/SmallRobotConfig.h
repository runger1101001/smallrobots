
#pragma once

#include <Arduino.h>
#include <map>
#include <set>

#include "SmallRobotDebug.h"



namespace SmallRobots {


    typedef enum : uint8_t {
        T_UNKNOWN,
        T_FLOAT,
        T_INT,
        T_STRING
    } SmallRobotParameterType;
    

    class SmallRobotParameter { // TODO this should be a template class
    friend class SmallRobotConfig;
    public:
        SmallRobotParameter(String name, float value) {
            this->name = name;
            this->type = SmallRobotParameterType::T_FLOAT;
            f = value;
        };
        SmallRobotParameter(String name, int value) {
            this->name = name;
            this->type = SmallRobotParameterType::T_INT;
            i = value;
        };
        SmallRobotParameter(String name, String value) {
            this->name = name;
            this->type = SmallRobotParameterType::T_STRING;
            s = new String(value);
        };
        ~SmallRobotParameter() {
            if (type==SmallRobotParameterType::T_STRING && s!=nullptr) {
                delete s;
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


        SmallRobotParameter& operator=(float value) {
            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Setting float param "+name+" to "+String(value));
            f = value;
            return *this;
        };


        SmallRobotParameter& operator=(int32_t value) {
            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Setting int param "+name+" to "+String(value));
            i = value;
            return *this;
        };


        SmallRobotParameter& operator=(String value) {
            if (debug && smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Setting string param "+name+" to "+value);
            *s = value;
            return *this;
        };


        bool operator== (const SmallRobotParameter& other) {
            return this == &other;
        };

        SmallRobotParameterType type;
        String name;

        union {
            float f;
            int32_t i;
            String* s;
        };

        inline static bool debug = false;
    private:

        SmallRobotParameter(String name, SmallRobotParameterType type = SmallRobotParameterType::T_UNKNOWN) {
            this->name = name;
            this->type = type;
            if (type==SmallRobotParameterType::T_STRING) {
                s = new String();
            }
        };
    };




    class SmallRobotTags {
    public:

        SmallRobotTags& operator+=(const char* tag) {
            tags.insert(String(tag));
            return *this;
        };

        SmallRobotTags& operator+=(String& tag) {
            tags.insert(tag);
            return *this;
        };

        SmallRobotTags& operator-=(const char* tag) {
            tags.erase(String(tag));
            return *this;
        };

        bool operator[](String tag) {
            return tags.find(tag) != tags.end();
        };

    private:
        std::set<String> tags;

    };



    class SmallRobotConfig {
    public:
        SmallRobotConfig();
        ~SmallRobotConfig();

        void add(String name, float value);
        void add(String name, int value);
        void add(String name, String value);

        SmallRobotParameter& operator[](const char* key);
        SmallRobotParameter& operator[](String& key);

        SmallRobotTags tags;
        bool debug = false;
    
        static SmallRobotParameter UNKNOWN_PARAM;

    private:
        std::map<String, SmallRobotParameter*> params;
    };


    extern SmallRobots::SmallRobotConfig robot_config;

};

