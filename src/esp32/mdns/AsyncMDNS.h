
#pragma once

#if defined(ESP32)

#include <ESPmDNS.h>

namespace SmallRobots {

    struct MDNSLookupResult {
        const char* service;
        int port;
        char ip[16];
    };


    typedef std::function<void(MDNSLookupResult result)> MDNSLookupCallback;


    class AsyncMDNS {
        public:

            static bool begin(MDNSLookupCallback callback) {
                esp_err_t err = mdns_init();
                if (err) {
                    // TODO debug logging via SmallRobotDebug
                    return false;
                }
                AsyncMDNS::callback = callback;
                return true;
            }

            static bool lookup(const char* service, const char* proto) {
                mdns_browse_t* b = mdns_browse_new(service, proto, process_browse);
                return (b!=NULL);
            };

            static bool cancel_lookup(const char* service, const char* proto) {
                return (mdns_browse_delete(service, proto)==ESP_OK);
            };
            
        protected:
            static MDNSLookupCallback callback;

            static void process_browse(mdns_result_t *results){
                if (results!=NULL) {
                    mdns_ip_addr_t * a = results->addr;
                    while(a){
                        if(a->addr.type == IPADDR_TYPE_V6){
                        } else {
                            MDNSLookupResult result;
                            result.service = results->service_type;
                            result.port = results->port;
                            esp_ip4addr_ntoa(&a->addr.u_addr.ip4, result.ip, sizeof(result.ip));
                            callback(result);
                        }
                        a = a->next;
                    }
                }
                mdns_query_results_free(results);
            };

    };

}; // namespace SmallRobots

#endif