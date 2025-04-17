#pragma once

#include <Arduino.h>

#if defined(ESP32)
/**
 * @brief A class that provides a mutex lock for FreeRTOS
 * with automatic exception-safe unlock based on RAII.
 * 
 * Usage:
 * ```cpp
 * // instantiate using unique id
 * typedef Lock<1> MyResourceLock;
 * 
 * // or intatiate using string ids, any literal will work
 * char MY_RESOURCE_LOCK_NAME[] = "myresource";
 * typedef Lock<MY_RESOURCE_LOCK_NAME> MyResourceLock;
 * 
 * // use
 * void contentious() {
 *  MyResourceLock lock;
 *  // do something
 *  // lock is automatically released when going out of scope
 * }
 * ```
 */
template<auto id>
class Lock {
public:
    Lock() {
        xSemaphoreTake(mutex, portMAX_DELAY);
    };
    ~Lock() {
        xSemaphoreGive(mutex);
    };
protected:
    inline static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
};

#endif