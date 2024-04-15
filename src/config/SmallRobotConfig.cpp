
#include "./SmallRobotConfig.h"

namespace SmallRobots {



    SmallRobotConfig robot_config;

    SmallRobotParameter SmallRobotConfig::UNKNOWN_PARAM{"unknown", SmallRobotParameterType::T_UNKNOWN};

    SmallRobotConfig::SmallRobotConfig() {
    };

    SmallRobotConfig::~SmallRobotConfig() {
    };






    void SmallRobotConfig::add(String name, float value){
        SmallRobotParameter* p = new SmallRobotParameter(name, value);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            Serial.println("WARN: could not add parameter "+name);
            delete p;
        }
        else
            if (debug) Serial.println("Added parameter "+name);
    };




    void SmallRobotConfig::add(String name, int value){
        SmallRobotParameter* p = new SmallRobotParameter(name, value);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            Serial.println("WARN: could not add parameter "+name);
            delete p;
        }
        else
            if (debug) Serial.println("Added parameter "+name);
    };




    void SmallRobotConfig::add(String name, String value){
        SmallRobotParameter* p = new SmallRobotParameter(name, value);
        auto result = params.emplace(name, p);
        if (result.second==false) {
            Serial.println("WARN: could not add parameter "+name);
            delete p;
        }
        else
            if (debug) Serial.println("Added parameter "+name);
    };




    SmallRobotParameter& SmallRobotConfig::operator[](const char* key) {
        String skey = String(key);
        return operator[](skey);
    };




    SmallRobotParameter& SmallRobotConfig::operator[](String& key) {
        auto result = params.find(key);
        if (result != params.end()) {
            return *result->second;
        }
        else {
            return UNKNOWN_PARAM;
        }
    };



};
