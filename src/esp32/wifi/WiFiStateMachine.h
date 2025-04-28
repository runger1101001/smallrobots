
#pragma once


#if defined(ESP32)


#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include "StateMachine.h"
#include "config/SmallRobotConfig.h"
#include "control/SmallRobotControl.h"
#include "behaviour/Behaviours.h"
#include "control/SmallRobotEventBus.h"
#include "SmallRobotDebug.h"

namespace SmallRobots {

#define WIFI_CONNECTING_TIMEOUT 10000
#define WIFI_RECONNECT_INTERVAL 5000
#define WIFI_RECONNECT_RETRIES 3

#define UDP_PORT 48048
#define UDP_PORT_SEND 48049

    class WiFiStateMachine {
    public:
        StateMachine machine;        

        STATE(connecting);
        STATE(connected);
        STATE(disconnected);
        STATE(failed);

        TRANSITION(connecting2connected, ev_connected, connecting, connected);
        TRANSITION(connected2disconnected, ev_disconnected, connected, disconnected);
        TRANSITION(connecting2failed, connecting_timeout, connecting, failed);
        TRANSITION(connecting2connecting, connecting_timeout, connecting, connecting);
        TRANSITION(disconnected2failed, disconnected_timeout, disconnected, failed);
        TRANSITION(disconnected2connecting, disconnected_timeout, disconnected, connecting);

        WiFiStateMachine() {
            machine.all_states = {&connecting, &connected, &disconnected, &failed};
            machine.all_transitions = {&connecting2connected, &connected2disconnected, &connecting2failed, &connecting2connecting, &disconnected2failed, &disconnected2connecting};
            machine.initial_state = &connecting;

            connecting.enter = std::bind(&WiFiStateMachine::on_enter_connecting, this);
            connecting.timeout = WIFI_CONNECTING_TIMEOUT;
            connected.enter = std::bind(&WiFiStateMachine::on_enter_connected, this);
            disconnected.enter = std::bind(&WiFiStateMachine::on_enter_disconnected, this);
            disconnected.timeout = WIFI_RECONNECT_INTERVAL;
            connecting2failed.guard = [this](){
                return retries>=WIFI_RECONNECT_RETRIES;
            };
            disconnected2failed.guard = [this](){
                return retries>=WIFI_RECONNECT_RETRIES;
            };
            failed.enter = std::bind(&WiFiStateMachine::on_enter_failed, this);
        };
        ~WiFiStateMachine() {};
        


        void on_enter_connecting() {
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFiStateMachine: connecting to Wifi...");
            checkWiFi();
            retries++;
        };

        void on_enter_connected() {
            retries = 0;
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFiStateMachine: Wifi connected");
            // load robot configuration from server TODO move all this stuff somewhere better!
            if (first_connection) {
                //robot_config.loadDefaultsFromServer();
                first_connection = false;
            }
            startNetwork();
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFiStateMachine: Network up");
            if (event_bus) event_bus->emit("wifi_connected");
        };

        void on_enter_disconnected() {
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFiStateMachine: WiFi is disconnected. Reconnecting in " + String(WIFI_RECONNECT_INTERVAL) + " seconds...");
            if (event_bus) event_bus->emit("wifi_disconnected");    
        };

        void on_enter_failed() {
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFiStateMachine: WiFi has failed");
            if (event_bus) event_bus->emit("wifi_failed");
        };






        void start() {
            // TODO check ssid is set
            // TODO check hostname is set and default it if not
            WiFi.setHostname(hostname.c_str());
            if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Hostname: " + hostname);
            // function pointer to event handler method? or use lambda and reference to static instance?
            using namespace std::placeholders;
            WiFiEventFuncCb func = std::bind(&WiFiStateMachine::onWifiEvent, this, std::placeholders::_1, std::placeholders::_2);
            WiFi.onEvent(func);
            WiFi.mode(WIFI_STA);
            machine.start();
        };




        void tick() {
            machine.tick();
            if (ota && machine==connected) ArduinoOTA.handle();
        };


        String hostname = (const char*)nullptr;
        String ssid = (const char*)nullptr;
        String password = (const char*)nullptr;
        int port = UDP_PORT;
        int send_port = UDP_PORT_SEND;
        bool ota = true;
        uint8_t retries = 0;
        IPAddress multicast_ip = IPAddress(239, 255, 0, 1);
        EventBus<String>* event_bus;

    protected:
        AsyncUDP udp; //global to send OSC Messages
        bool first_connection = false;




        void checkWiFi() {
            wl_status_t wifiStatus = WiFi.status();
            switch (wifiStatus) {
                case WL_NO_SSID_AVAIL:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Can't find network with SSID: " + WiFi.SSID());
                    break;
                case WL_CONNECTED:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Already connected to WiFi");
                    break;
                case WL_CONNECT_FAILED:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFi connection failed.");
                    // fall through and try again
                case WL_IDLE_STATUS:
                case WL_CONNECTION_LOST:
                case WL_DISCONNECTED:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->print("Connecting to WiFi ");
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println(ssid);
                    WiFi.begin(ssid.c_str(), password.c_str());
                    break;
                case WL_SCAN_COMPLETED:
                default:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Unexpected wl_status_t: "+String(wifiStatus));
                    break;
            }
        };




        void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info){
            switch (event) {
                case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
                case ARDUINO_EVENT_WIFI_READY:
                case ARDUINO_EVENT_WIFI_SCAN_DONE:
                case ARDUINO_EVENT_WIFI_STA_START:
                case ARDUINO_EVENT_WIFI_STA_STOP:
                    break;
                case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
                case ARDUINO_EVENT_WIFI_STA_CONNECTED:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->print("Connected to ");
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println(WiFi.SSID());
                    break;
                case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
                    machine.trigger("ev_disconnected");
                    break;
                case ARDUINO_EVENT_WIFI_STA_GOT_IP:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->print("IP address: ");
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println(WiFi.localIP());
                    machine.trigger("ev_connected");
                    break;
                case ARDUINO_EVENT_WIFI_STA_LOST_IP:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("WiFi lost IP address.");
                    machine.trigger("ev_disconnected");
                    break;                
                default:
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->print("Unexpected WiFi event: ");
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println(event);
                    break;
            }
        };



        void startNetwork() {
            // start UDP
            if (udp.listen(port)) { // TODO perhaps put this under control of state machine, or RobotControl class
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->printf("Listening on udp port %d\n", port);
                AuPacketHandlerFunction func = std::bind(&SmallRobotControl::onPacket, &osc_control, std::placeholders::_1);
                udp.onPacket(func);
            }
            else {
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("ERROR: Failed to listen on udp port");
                // TODO trigger failed?
                return;
            }
            if (udp.listenMulticast(multicast_ip, port)) {
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->printf("Listening on multicast ip %s port %d\n", multicast_ip.toString().c_str(), port);
                AuPacketHandlerFunction func = std::bind(&SmallRobotControl::onPacket, &osc_control, std::placeholders::_1);
                udp.onPacket(func);
            }
            else {
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("ERROR: Failed to listen on multicast udp port");
                // TODO trigger failed?
                return;
            }
            
            // start OTA
            if (ota) {
                startOTA();
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Available for OTA.");
            }

            if (!MDNS.begin(hostname.c_str()))
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Bonjour registration error!");
            else {
                MDNS.addService("robot_control", "udp", port);
                #if defined(FIRMWARE_VERSION)
                MDNS.addServiceTxt("robot_control", "udp", "firmware", FIRMWARE_VERSION);
                #endif
                if (ota) 
                    MDNS.enableArduino(); // advertise OTA
                if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Registered in bonjour.");
            }
        };





        void startOTA() {
            ArduinoOTA
                .onStart([this]() {
                    String type;
                    if (ArduinoOTA.getCommand() == U_FLASH)
                        type = "sketch";
                    else // U_SPIFFS
                        type = "filesystem";
                    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("OTA update starting " + type);
                })
                .onEnd([this]() {
                    MDNS.disableArduino();
                    mdns_service_remove("_robot_control","_udp");
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("\nOTA update completed");
                })
                .onProgress([this](unsigned int progress, unsigned int total) {
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->printf("OTA progress: %u%%\n", (progress / (total / 100)));
                })
                .onError([this](ota_error_t error) {
                    if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->printf("OTA error[%u]: ", error);
                    if (error == OTA_AUTH_ERROR) if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Auth Failed");
                    else if (error == OTA_BEGIN_ERROR) if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Begin Failed");
                    else if (error == OTA_CONNECT_ERROR) if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Connect Failed");
                    else if (error == OTA_RECEIVE_ERROR) if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("Receive Failed");
                    else if (error == OTA_END_ERROR) if (smallrobot_debug_print!=nullptr) smallrobot_debug_print->println("End Failed");
                })
                .setHostname(hostname.c_str())
                .setMdnsEnabled(false);
            ArduinoOTA.begin();
        };


    }; // class WiFiStateMachine




};  // namespace SmallRobots



#endif // ESP32
