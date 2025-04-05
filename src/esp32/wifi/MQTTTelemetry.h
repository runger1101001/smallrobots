
#pragma once

#include <IPAddress.h>
#include <PubSubClient.h>
#include "debug/SmallRobotsDebug.h"

namespace SmallRobots {

    class MQTTTelemetry {
    public:
        MQTTTelemetry(PubSubClient& mqtt, const char* topic) : mqtt(mqtt), topic(topic) {};

        void begin(const char* hostname, IPAddress broker, uint16_t port = 1883) {
            mqtt.setServer(broker, port);
            if (mqtt.connect(hostname))
                smallrobots_debug.println("MQTT connected");
            else
                smallrobots_debug.println("MQTT connection failed");
            _hostname = hostname;
        };
        
        void sendTelemetry(uint8_t* data, size_t len, const char* subtopic = nullptr) {
            auto c = connect_and_topic(subtopic);
            if (c.first) {
                //smallrobots_debug.printf("MQTT publish %s size %d\n", c.second.c_str(), len);
                mqtt.publish(c.second.c_str(), data, len);
            }
        };


        void sendTelemetry(size_t len, std::function<void(Print&)> data_provider, const char* subtopic = nullptr) {
            auto c = connect_and_topic(subtopic);
            if (c.first) {
                String full_topic = c.second;
                //smallrobots_debug.printf("MQTT publish %s size %d\n", full_topic.c_str(), len);
                mqtt.beginPublish(full_topic.c_str(), len, false);
                data_provider(mqtt);
                mqtt.endPublish();
            }
        }


    protected:
        const char* _hostname = nullptr;
        PubSubClient& mqtt;
        const char* topic;

        std::pair<bool, String> connect_and_topic(const char* subtopic = nullptr) {
            bool connected = false;
            if (!mqtt.connected()) {
                if (_hostname) {
                    if (mqtt.connect(_hostname)) {
                        smallrobots_debug.println("MQTT reconnected");
                        connected = true;
                    }
                    else
                        smallrobots_debug.println("MQTT not connected");
                }    
            }
            else
                connected = true;
            if (connected) {
                String full_topic = String(topic) + '/' + _hostname; // TODO make this more efficient!
                if (subtopic)
                    full_topic += String("/") + subtopic;
                return {connected, full_topic};
            }
            return {false, ""};
        }


    };

}; // namespace SmallRobots
