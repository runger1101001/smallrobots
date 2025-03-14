
#pragma once

#include <Print.h>
#include <WiFiUdp.h>
#include <IPAddress.h>

namespace SmallRobots {

    class UDPDebug : public Print {
        public:
            UDPDebug(WiFiUDP &udp);
            virtual ~UDPDebug();

            void begin(IPAddress ip, uint16_t port);

            virtual size_t write(uint8_t) override;
            virtual void flush() override;

        protected:
            WiFiUDP &_udp;
            IPAddress _ip = INADDR_NONE;
            uint16_t _port = 0;
    };


};  // namespace SmallRobots