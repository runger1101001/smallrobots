
#pragma once

#if defined(ESP32)

#include <ESPmDNS.h>
#include <map>
#include <functional>

namespace SmallRobots {

    struct MDNSLookup;

    struct MDNSLookupResult {
        MDNSLookup* lookup;
        int port;
        IPAddress ip;
    };

    struct MDNSLookup {
        const char* service;
        const char* proto;
        std::function<void(MDNSLookupResult result)> callback;
    };



    class AsyncMDNS {
        public:

            static bool lookup(const char* service, const char* proto, std::function<void(MDNSLookupResult result)> callback) {
                MDNSLookup lookup = {service, proto, callback};
                lookups.emplace(service, lookup);
                xTaskCreate(
                    [](void* p){
                        MDNSLookup* l = (MDNSLookup*)p;
                        int nr = 0;
                        while (nr<=0 && lookups.find(l->service)!=lookups.end()) { // TODO thread safety
                            nr = MDNS.queryService(l->service, l->proto);
                            if (nr>0) {
                                MDNSLookupResult r = { l, MDNS.port(0), MDNS.address(0) };
                                l->callback(r);
                            }
                            else 
                                vTaskDelay(250 / portTICK_PERIOD_MS);
                        }
                        lookups.erase(l->service);
                        vTaskDelete(NULL);
                    },
                    "mdns_callback",
                    4096,
                    &lookups[service],
                    1,
                    NULL
                );
                return true;
                // mdns_browse_t* b = mdns_browse_new(service, proto, process_browse);
                // return (b!=NULL);
            };

            static bool cancel_lookup(const char* service, const char* proto) {
                return lookups.erase(service)==1;
            };
            
        protected:
            static std::map<const char*, MDNSLookup> lookups;

    };

}; // namespace SmallRobots

#endif