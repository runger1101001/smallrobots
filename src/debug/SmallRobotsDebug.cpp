
#include "SmallRobotsDebug.h"

namespace SmallRobots {

    Debug::Debug() {
        for (int i = 0; i < SMALLROBOTS_DEBUG_MAX_CHANNELS; i++) {
            _channels[i] = nullptr;
            _enabled[i] = false;
        }
    }

    Debug::~Debug() {
    }

    int Debug::addChannel(Print* delegate) {
        for (int i = 0; i < SMALLROBOTS_DEBUG_MAX_CHANNELS; i++) {
            if (_channels[i] == nullptr) {
                _channels[i] = delegate;
                return i;
            }
        }
        return -1;
    }

    void Debug::enableChannel(Print* channel, bool enable) {
        if (channel != nullptr) {
            for (int i = 0; i < SMALLROBOTS_DEBUG_MAX_CHANNELS; i++) {
                if (_channels[i] == channel) {
                    _enabled[i] = enable;
                    return;
                }
            }
        }
    }

    size_t Debug::write(uint8_t c) {
        size_t count = 0;
        for (int i = 0; i < SMALLROBOTS_DEBUG_MAX_CHANNELS; i++) {
            if (_channels[i] != nullptr && _enabled[i]) {
                count = _channels[i]->write(c);
            }
        }
        if (c == '\n') {
            internal_flush(); // flush on newline
        }
        return count; // return the last count - this could be improved
    }

    void Debug::internal_flush() {
        for (int i = 0; i < SMALLROBOTS_DEBUG_MAX_CHANNELS; i++) {
            if (_channels[i] != nullptr && _enabled[i]) {
                _channels[i]->flush();
            }
        }
    }
    void Debug::flush() {
        // drop the flushes
    }

};

