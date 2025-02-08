
#pragma once

#include <Print.h>

#define SMALLROBOTS_DEBUG_MAX_CHANNELS 4


namespace SmallRobots {

    class Debug : public Print {
        public:
            Debug();
            virtual ~Debug();

            int addChannel(Print* delegate);
            void enableChannel(Print* channel, bool enable = true);

            virtual size_t write(uint8_t) override;
            virtual void flush() override;
        
        protected:
            void internal_flush();

            Print* _channels[SMALLROBOTS_DEBUG_MAX_CHANNELS];
            bool _enabled[SMALLROBOTS_DEBUG_MAX_CHANNELS];
    };




};