#pragma once

#include "./Behaviours.h"
#include "../rgbLeds/RGBLedService.h"


#define RGBLED_UPDATE_RATE 100 //how often is the led updated in the behaviour engine

#define RGBLED_FADE_RATE  100 

#define RGBLED_BLINK_RATE 500

enum RGB_STATUS{
  I2C_INITIALIZING,
  I2C_FAILED,
  I2C_WORKING,
  MOTOR1_INITIALIZING,
  MOTOR1_FAILED,
  MOTOR1_WORKING,
  MOTOR2_INITIALIZING,
  MOTOR2_FAILED,
  MOTOR2_WORKING,
  MOTORS_ALL_INITIALIZING,
  MOTORS_ALL_FAILED,
  MOTORS_ALL_WORKING,
  WIFI_STATUS_CONNECTING,
  WIFI_STATUS_CONNECTED,
  WIFI_STATUS_FAILED,
  BATTERY_STATUS,
  BATTERY_STATUS_WARN,
  OFF,
  GENERATE,
  FROM_WIFI
}; 

RGB_STATUS rgbStatus = WIFI_STATUS_CONNECTING;
extern u_int8_t externalR, externalG, externalB ;

namespace SmallRobots {


    


   class RGBBehaviour : public Behaviour {


        public:
            RGBBehaviour(RGBLedService& _rgbService): Behaviour(RGBLED_UPDATE_RATE), rgbService(_rgbService) {

                rgbService.setup();

            };

            virtual Behaviour* run() {


                switch (rgbStatus) {
                    case I2C_INITIALIZING:
                    printf("I2C_INITIALIZING");
                    break;
                    case I2C_FAILED:
                    printf("I2C_FAILED");
                    break;
                    case I2C_WORKING:
                    printf("I2C_WORKING");
                    break;

                    case MOTORS_ALL_INITIALIZING:
                    rgbService.setFinalCol(200,100,0);
                    
                    rgbService.allBlink();
                    
                    break;
                    case MOTORS_ALL_WORKING:
                    rgbService.setFinalCol(0,255,0);
                    rgbService.allShow();

                    break;
                    case MOTORS_ALL_FAILED:
                    printf("MOTORS_ALL_FAILED");
                    break;
                    
                    case WIFI_STATUS_CONNECTING:
                    rgbService.setFinalCol(0,0,255);
                    rgbService.allShow();
                    break;

                    case WIFI_STATUS_CONNECTED:
                    printf("WIFI_STATUS_CONNECTED");
                    break;

                    case WIFI_STATUS_FAILED:
                    printf("WIFI_STATUS_FAILED");
                    break;

                    case BATTERY_STATUS:
                    rgbService.setBatteryStatus();
                    rgbService.allShow();
                    break;

                    case BATTERY_STATUS_WARN:
                    rgbService.setBatteryStatus();
                    rgbService.allBlink();
                    break;
                    case OFF:
                    rgbService.allOff();
                    break;
                    case GENERATE:
                    printf("GENERATE");
                    break;
                    case FROM_WIFI:
                    rgbService.setFinalCol(externalR, externalG, externalB);
                    rgbService.allShow();
                    break;
                   // default:


                };

                return this;
              
            };

            virtual const char* getName() override { return name.c_str(); };
           

        protected:
            String name = "RGBBehaviour";
            RGBLedService& rgbService;
            
    };
        

}
