
#pragma once

#include <Stream.h>
#include <Arduino.h>
#include "SimpleFOCRegisters.h"
#include "CommanderMaster.h"

namespace simplefoc {

    class SerialCommanderMaster : public CommanderMaster {
        public:
            SerialCommanderMaster();

            void begin(Print &serial = Serial);

            int writeRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size);
            int readRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size);

        protected:
            Print* _print;
    };


    #define TELEMETRY_MAX_BUFFER_SIZE 256

    class SerialTelemetryMaster : public SerialCommanderMaster {
        public:
            SerialTelemetryMaster();

            void begin(Stream &serial = Serial);

            void run();

            void setDebugHandler(void (*handler)(char*, uint8_t));
            void setTelemetryHandler(void (*handler)());
            void setOnSyncHandler(void (*handler)());

            void setTelemetryRegisters();
        protected:
            Stream* _stream;
            uint8_t _buffer[TELEMETRY_MAX_BUFFER_SIZE];
            void (*_debugFunc)(char*, uint8_t) = nullptr;
            void (*_telemetryFunc)() = nullptr;
            void (*_onSyncFunc)() = nullptr;

            // StateMachine machine;

            // STATE(idle);
            // STATE(debug);
            // STATE(telemetry);
            // STATE(header);

            // TRANSITION(todebug,"D",idle, debug);
            // TRANSITION(totelemetry,"T",idle, telemetry);
            // TRANSITION(toheader,"H",idle, header);
            // TRANSITION(toidle,"I", State::ANY_STATE, idle);

            // void on_debug() { debug_print->write()};
            // void on_exit_debug();
            // void on_telemetry();
            // void on_exit_telemetry();
            // void on_header();
            // void on_exit_header();

            // void processByte(uint8_t byte);

    };


}; // namespace simplefoc
