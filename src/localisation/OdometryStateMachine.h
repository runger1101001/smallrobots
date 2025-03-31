#pragma once

#include <Arduino.h>
#include "config/SmallRobotConfig.h"
#include "control/SmallRobotEventBus.h"
#include "StateMachine.h"
#include "./Odometry.h"

#define ODOMETRY_UPDATE_RATE_TIMEOUT 1 //ms

namespace SmallRobots {

   


 class OdometryStateMachine {
    public:
        StateMachine machine;       

        STATE(idle);
        STATE(running);
        STATE(reset);

        TRANSITION(idle2running,start_localisation, idle, running);
        TRANSITION(running2running, running_timeout, running, running);
        TRANSITION(running2idle, pause_localisation, running, idle);
        TRANSITION(running2reset, reset_running_localisation, running, reset);
        TRANSITION(idle2reset, reset_idle_localisation, idle, reset);
        TRANSITION(reset2idle, reset_pause_localisation, reset, idle);
        TRANSITION(reset2running, reset_start_localisation, reset, running);


       
        OdometryStateMachine (DifferentialKinematics& _kinematics): kinematics(_kinematics), odometryCtrl(_kinematics){
            machine.all_states = {&idle, &running, &reset};
            machine.all_transitions = { &idle2running, 
                                        &running2running,
                                        &running2idle,
                                        &running2reset,
                                        &idle2reset,
                                        &reset2idle,
                                        &reset2running
                                        };
            machine.initial_state = &idle;

            idle.enter = std::bind(&OdometryStateMachine::on_enter_idle, this);
            running.enter =  std::bind(&OdometryStateMachine::on_enter_running, this);
            reset.enter = std::bind(&OdometryStateMachine::on_enter_reset, this);
     
            idle2running.on = std::bind(&OdometryStateMachine::on_exit_idle_or_reset, this);
            reset2running.on = std::bind(&OdometryStateMachine::on_exit_idle_or_reset, this);

            running.timeout = ODOMETRY_UPDATE_RATE_TIMEOUT;

        };
        ~OdometryStateMachine() {};

        
        void on_enter_idle()
        {
            isRunning = false;
        };
        void on_exit_idle_or_reset(){
            odometryCtrl.resetLastTime();
        };

        void on_enter_running()
        {
            isRunning = true;
            odometryCtrl.updatePose();
            odometryPose = odometryCtrl.getCurPose();
            event_bus.emit("new_odometry_pose");
            
        };
        void on_enter_reset()
        {
            odometryCtrl.resetCurPose();

            if (isRunning) machine.trigger("reset_start_localisation");
            else machine.trigger("reset_pause_localisation");
           
        };

        void start(){
            machine.start();
        };

        void tick() {
            machine.tick();
        };


        protected:

            DifferentialKinematics& kinematics;
            Odometry odometryCtrl;
            bool isRunning = false; //to know previous state in reset
    };
 }; //close namespace SmallRobots