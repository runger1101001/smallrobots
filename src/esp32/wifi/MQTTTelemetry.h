
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
            if (mqtt.connected()) {
                String full_topic = String(topic) + '/' + _hostname; // TODO make this more efficient!
                if (subtopic)
                    full_topic += String("/") + subtopic;
                //smallrobots_debug.printf("MQTT publish %s size %d\n", full_topic.c_str(), len);
                mqtt.publish(full_topic.c_str(), data, len);
            }
            else if (_hostname) {
                if (mqtt.connect(_hostname))
                    smallrobots_debug.println("MQTT reconnected");
                else
                    smallrobots_debug.println("MQTT not connected");
            }
        };
    protected:
        const char* _hostname = nullptr;
        PubSubClient& mqtt;
        const char* topic;
    };

}; // namespace SmallRobots
