
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
                mdns_search_once_t *s = mdns_query_async_new(NULL, service, proto, MDNS_TYPE_ANY, 3000, 100, process_results);
                return (s!=NULL);
            };
            
        protected:
            static MDNSLookupCallback callback;

            static void process_results(mdns_search_once_s *search) {
                mdns_result_t *results = NULL;
                uint8_t num_results = 0;
                mdns_query_async_get_results(search, 10, &results, &num_results);
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
                mdns_query_async_delete(search);
            };

    };

}; // namespace SmallRobots

#endif