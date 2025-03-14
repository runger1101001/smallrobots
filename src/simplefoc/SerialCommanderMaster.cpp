
#include "SerialCommanderMaster.h"
namespace simplefoc {


SerialCommanderMaster::SerialCommanderMaster() {
}

void SerialCommanderMaster::begin(Print &serial) {
    _print = &serial;
}



int SerialCommanderMaster::writeRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size){
    return 0;
};
int SerialCommanderMaster::readRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size){
    return 0;
};



SerialTelemetryMaster::SerialTelemetryMaster() {
}

void SerialTelemetryMaster::begin(Stream &serial) {
    _stream = &serial;
    SerialCommanderMaster::begin(serial);
}

void SerialTelemetryMaster::run() {
    if (_stream->available()) {
        size_t bytes = _stream->available();
        if (bytes > sizeof(_buffer)) bytes = sizeof(_buffer);
        size_t actbytes = _stream->readBytes(_buffer, bytes);
        // TODO processBytes(_buffer, actbytes);
        // for the moment, just send everything we get to debug...
        if (_debugFunc)
            _debugFunc((char*)_buffer, actbytes);
    }
}

void SerialTelemetryMaster::setDebugHandler(void (*handler)(char*, uint8_t)) {
    _debugFunc = handler;
}

void SerialTelemetryMaster::setTelemetryHandler(void (*handler)()) {
    _telemetryFunc = handler;
}

void SerialTelemetryMaster::setOnSyncHandler(void (*handler)()) {
    _onSyncFunc = handler;
}

}; // namespace simplefoc
