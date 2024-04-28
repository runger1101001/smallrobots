

#pragma once


#include "./SimpleFOCRegisters.h"


namespace simplefoc {


    class CommanderMaster {
    public:
        CommanderMaster() {};
        ~CommanderMaster() {};

        virtual int writeRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size) = 0;
        virtual int readRegister(int motor, SimpleFOCRegister registerNum, void* data, uint8_t size) = 0;

    };

};
