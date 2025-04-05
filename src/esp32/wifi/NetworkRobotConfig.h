#pragma once

#include "../../config/SmallRobotConfig.h"
#include "../../control/SmallRobotControl.h"
#include "../../debug/SmallRobotsDebug.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <StreamUtils.h>


namespace SmallRobots {
    
    class NetworkRobotConfig {
        public:
            NetworkRobotConfig(PubSubClient& mqtt) : _mqtt(mqtt) {
                // Constructor
                topic = "";
            };

            void begin(String hostname) {
                _hostname = hostname;
                topic = "config/" + hostname;
                SmallRobots::osc_control.addCommand("get-config", [this](OSCMessage& msg) {
                    send_requested = true;
                });
                SmallRobots::osc_control.addCommand("set-config", std::bind(&NetworkRobotConfig::setConfiguration, this, std::placeholders::_1));
            };


            void run() {
                if (send_requested) {
                    send_requested = false;
                    sendConfiguration();
                }
            }

            void sendConfiguration() {
                if (!_mqtt.connected() || topic.length() == 0) {
                    return;
                }
                JsonDocument doc;
                doc["id"] = _hostname;
                doc["tags"] = JsonArray();
                SmallRobots::robot_config.tags.forEach([&doc](String tag) {
                    doc["tags"].add(tag);
                });
                doc["params"] = JsonObject();
                SmallRobots::robot_config.forEach([&doc](const char* name, SmallRobotParameter& param) {
                    if (param.type == SmallRobotParameterType::T_INT) {
                        doc["params"][name] = param.i;
                    }
                    else if (param.type == SmallRobotParameterType::T_FLOAT) {
                        doc["params"][name] = param.f;
                    }
                    else if (param.type == SmallRobotParameterType::T_STRING) {
                        doc["params"][name] = *param.s;
                    }
                    else if (param.type == SmallRobotParameterType::T_BOOL) {
                        doc["params"][name] = param.b;
                    }
                });
                _mqtt.beginPublish(topic.c_str(), measureJson(doc), false);
                BufferingPrint buffered(_mqtt, 128);
                serializeJson(doc, buffered);
                buffered.flush();
                _mqtt.endPublish();
            };

            void setConfiguration(OSCMessage& msg) {
                if (!msg.isString(0))
                    return;
                JsonDocument doc;
                deserializeJson(doc, msg.getBlob(0), msg.getBlobLength(0));
                JsonObject root = doc.as<JsonObject>();
                for (JsonPair kv : root) {
                    SmallRobotParameter& param = robot_config[kv.key().c_str()];
                    if (param == SmallRobotConfig::UNKNOWN_PARAM) {

                    }
                    else {
                        if (param.type == SmallRobotParameterType::T_INT) {
                            param = (int32_t)kv.value().as<int>();
                        }
                        else if (param.type == SmallRobotParameterType::T_FLOAT) {
                            param = kv.value().as<float>();
                        }
                        else if (param.type == SmallRobotParameterType::T_STRING) {
                            param = kv.value().as<String>();
                        }
                        else if (param.type == SmallRobotParameterType::T_BOOL) {
                            param = kv.value().as<bool>();
                        }
                    }
                }
            };

        protected:
            PubSubClient& _mqtt;
            String topic;
            String _hostname;
            bool send_requested = false;
    };


};

