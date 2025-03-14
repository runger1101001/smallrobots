
#include "UDPDebug.h"

namespace SmallRobots {



    UDPDebug::UDPDebug(WiFiUDP& udp) : _udp(udp) {
        
    }

    UDPDebug::~UDPDebug() {
    }

    void UDPDebug::begin(IPAddress ip, uint16_t port) {
        _ip = ip;
        _port = port;
        _udp.beginPacket(_ip, _port);
    }


    size_t UDPDebug::write(uint8_t c) {
        if (_ip!=INADDR_NONE && _port>0) 
            return _udp.write(c);
        else
            return 0;
    }


    void UDPDebug::flush() {
        if (_ip!=INADDR_NONE && _port>0) {
            _udp.endPacket();
            _udp.beginPacket(_ip, _port);
        }
    }




};  // namespace SmallRobots
