
#include "./SmallRobotConfig.h"
#include "math.h"

namespace SmallRobots {



    SmallRobotConfig robot_config;

    SmallRobotParameter SmallRobotConfig::UNKNOWN_PARAM{"unknown", SmallRobotParameterType::T_UNKNOWN};

    SmallRobotConfig::SmallRobotConfig() {
        // configure permanent tags for this robot
        tags += "#all";
        #if defined(SMALLROBOT_SPECIES)
        tags += (String("#")+SMALLROBOT_SPECIES);    
        #endif    
    };

    SmallRobotConfig::~SmallRobotConfig() {
    };






    SmallRobotParameter& SmallRobotConfig::add(const char* name, float value, std::function<void(float)> onchange){
        SmallRobotParameter* p = new SmallRobotParameter(name, value, onchange);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            smallrobots_debug.printf("WARN: could not add parameter %s\n",name);
            delete p;
            return UNKNOWN_PARAM;
        }
        else
            if (debug) smallrobots_debug.printf("Added parameter %s\n",name);
        return *p;
    };




    SmallRobotParameter& SmallRobotConfig::add(const char* name, int value, std::function<void(int)> onchange){
        SmallRobotParameter* p = new SmallRobotParameter(name, value, onchange);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            smallrobots_debug.printf("WARN: could not add parameter %s\n",name);
            delete p;
            return UNKNOWN_PARAM;
        }
        else
            if (debug) smallrobots_debug.printf("Added parameter %s\n",name);
        return *p;
    };




    SmallRobotParameter& SmallRobotConfig::add(const char* name, String value, std::function<void(String*)> onchange){
        SmallRobotParameter* p = new SmallRobotParameter(name, value, onchange);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            smallrobots_debug.printf("WARN: could not add parameter %s\n",name);
            delete p;
            return UNKNOWN_PARAM;
        }
        else
            if (debug) smallrobots_debug.printf("Added parameter %s\n",name);
        return *p;
    };



    SmallRobotParameter& SmallRobotConfig::add(const char* name, bool value, std::function<void(bool)> onchange){
        SmallRobotParameter* p = new SmallRobotParameter(name, value, onchange);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            smallrobots_debug.printf("WARN: could not add parameter %s\n",name);
            delete p;
            return UNKNOWN_PARAM;
        }
        else
            if (debug) smallrobots_debug.printf("Added parameter %s\n",name);
        return *p;
    };



    SmallRobotParameter& SmallRobotConfig::operator[](const char* key) {
        auto result = params.find(key);
        if (result != params.end()) {
            return *result->second;
        }
        else {
            return UNKNOWN_PARAM;
        }
    };




    SmallRobotParameter& SmallRobotConfig::operator[](String& key) {
        return operator[](key.c_str());
    };



};
