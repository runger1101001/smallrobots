
#pragma once

#include <IPAddress.h>
#include <PubSubClient.h>
#include "debug/SmallRobotsDebug.h"

using namespace SmallRobots;

class MQTTTelemetry {
public:
    MQTTTelemetry(PubSubClient& mqtt, const char* topic) : mqtt(mqtt), topic(topic) {};

    void begin(const char* hostname, IPAddress broker, uint16_t port = 1883) {
        mqtt.setServer(broker, port);
        if (mqtt.connect(_hostname))
            smallrobots_debug.println("MQTT connected");
        else
            smallrobots_debug.println("MQTT connection failed");
        _hostname = hostname;
    };
    
    void sendTelemetry(uint8_t* data, size_t len) {
        if (mqtt.connected()) {
            mqtt.publish(topic, data, len);
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

