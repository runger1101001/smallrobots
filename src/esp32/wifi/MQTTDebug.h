
#pragma once

#include <PubSubClient.h>
#include "debug/SmallRobotsDebug.h"

class MQTTDebug : public Print {
public:
    MQTTDebug(PubSubClient& pubsub) : mqtt(pubsub), Print() {
    };
    ~MQTTDebug(){};

    void begin(String& hostname, String& id, IPAddress server, uint16_t port) {
        mqtt.setServer(server, port);
        if (!mqtt.connect(hostname.c_str())) {
            SmallRobots::smallrobots_debug.println("Debug MQTT connection failed");
            return;
        }
        topic = "/debug/" + id;
    };

    virtual size_t write(uint8_t b) override {
        if (pos >= sizeof(buffer))
            _flush();
        buffer[pos++] = b;
        if (b == '\n')
            _flush();
        return 1;
    };

    virtual void flush() override {
        // ignore flush
    };

protected:
    PubSubClient& mqtt;
    String topic;
    uint8_t buffer[256];
    size_t pos = 0;

    void _flush() {
        // TODO reconnect if we've lost connection
        if (mqtt.connected()) 
            mqtt.publish(topic.c_str(), buffer, pos);
        pos = 0;
    };
};