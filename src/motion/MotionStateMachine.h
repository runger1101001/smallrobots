#pragma once

#include <Arduino.h>
#include "config/SmallRobotConfig.h"
#include "control/SmallRobotEventBus.h"
#include "StateMachine.h"
#include "./MotionController.h"
#include "./localisation/Odometry.h"

#define M_ODOMETRY_UPDATE_RATE_TIMEOUT 10 //ms

namespace SmallRobots {


 class MotionStateMachine {
    public:
        StateMachine machine;  

        int subPathIndex = 0;   //to identify which of the 3 path parts of the dubin path are currently dealt with

        STATE(idle);
        STATE(new_target_pose);
        STATE(moving);
        STATE(arrived_at_tangentA);
        STATE(arrived_at_tangentB);
        STATE(arrived_at_target_pose);


        TRANSITION(idle2new_target_pose,set_new_pose, idle, new_target_pose);
        TRANSITION(new_target_pose2moving, start_seg1,new_target_pose, moving);
        TRANSITION(moving2arrived_at_tangentA, finished_seg1, moving, arrived_at_tangentA);
        TRANSITION(arrived_at_tangentA2moving, start_seg2, arrived_at_tangentA, moving);
        TRANSITION(moving2arrived_at_tangentB, finished_seg2, moving, arrived_at_tangentB);
        TRANSITION(arrived_at_tangentB2moving, start_seg3, arrived_at_tangentB, moving);
        TRANSITION(moving2arrived_at_target_pose, finished_seg3, moving, arrived_at_target_pose);
        TRANSITION(arrived_at_target_pose2idle, next, arrived_at_target_pose, idle);
        TRANSITION(moving2moving,moving_timeout, moving, moving);
        TRANSITION(arrived_at_target_pose2new_target_pose, follow_path,arrived_at_target_pose,new_target_pose);

        TRANSITION(moving2idle, pause, moving, idle);
        TRANSITION(idle2moving, startOdometry, idle, moving);


       
        MotionStateMachine(MotionController& _ctrl) : ctrl(_ctrl){
            machine.all_states = {&idle, &new_target_pose, &moving, &arrived_at_tangentA, &arrived_at_tangentB, &arrived_at_target_pose};
            machine.all_transitions = { &idle2new_target_pose, 
                                        &new_target_pose2moving,
                                        &moving2arrived_at_tangentA, 
                                        &arrived_at_tangentA2moving,
                                        &moving2arrived_at_tangentB,
                                        &arrived_at_tangentB2moving,
                                        &moving2arrived_at_target_pose,
                                        &arrived_at_target_pose2idle,
                                        &moving2moving,
                                        &arrived_at_target_pose2new_target_pose,

                                        &moving2idle,
                                        &idle2moving
                                        };
            machine.initial_state = &idle;

            new_target_pose.enter = std::bind(&MotionStateMachine::on_enter_new_target_pose, this);
            arrived_at_tangentA.enter =  std::bind(&MotionStateMachine::on_enter_arrived_at_tangentA, this);
            arrived_at_tangentB.enter = std::bind(&MotionStateMachine::on_enter_arrived_at_tangentB, this);
            arrived_at_target_pose.enter = std::bind(&MotionStateMachine::on_enter_arrived_at_target_pose, this);
            moving.enter = std::bind(&MotionStateMachine::on_enter_moving, this);
            idle.enter =  std::bind(&MotionStateMachine::on_enter_idle, this);

            moving.timeout = M_ODOMETRY_UPDATE_RATE_TIMEOUT;

            moving2idle.on = std::bind(&MotionStateMachine::on_pause, this);
            idle2new_target_pose.on =  std::bind(&MotionStateMachine::on_go, this);
            idle2moving.on = std::bind(&MotionStateMachine::on_start_odometry,this);
        };
        ~MotionStateMachine() {};

        
        void on_pause()
        {
            ctrl.stop();
        };

             
        void on_go()
        {
           ctrl.enableMotors();
        };
        
        
        void on_enter_idle()
        {
            if (first_on) {
                // Serial.println("Motion idle -  disable Motors");
                //ctrl.stop();
                first_on = false;
            }

        };

        void on_enter_new_target_pose() { 
            
            Serial.println("ctrl.setTarget()");
            ctrl.setTarget();                          

            Serial.println("odometry.resetLastTime()");
            odometry.resetLastTime();     

            Serial.println("ctrl.setWheelVelocitiesSeg1()");
            ctrl.setWheelVelocitiesSeg1();        
            
            //reset subPathIndex
            Serial.println("reset subPathIndex to 0");
            subPathIndex = 0;

            Serial.println("machine.trigger(start_seg1)");
            machine.trigger("start_seg1");             

            
        };

        void on_enter_arrived_at_tangentA() { 
            // Serial.println("odometry.resetLastTime()");
            odometry.resetLastTime();
            // Serial.println("ctrl.setWheelVelocitiesSeg2()");
            ctrl.setWheelVelocitiesSeg2();
            machine.trigger("start_seg2");
        };

        void on_enter_arrived_at_tangentB() { 
            // Serial.println("odometry.resetLastTime()");
            odometry.resetLastTime();
            // Serial.println("ctrl.setWheelVelocitiesSeg3()");
            ctrl.setWheelVelocitiesSeg3();
            machine.trigger("start_seg3");
        };

        void on_start_odometry(){
            // Serial.println("odometry.resetLastTime()");
            odometry.resetLastTime();
        };

        void on_enter_arrived_at_target_pose() { 

            if (ctrl.pathBehaviour == LOOP)
            {
                ctrl.loopPath(); //counts up the pathIndex, if reached end, restarts from beginning, returns false if only one Pose in path
                machine.trigger("set_new_pose");
            }
            else if (ctrl.pathBehaviour ==END)
            {
                ctrl.stopMoving();
            }
           

        };
        
        void on_enter_moving() {
            //update odometry
            //check if arrived
            //no -> repeat this state
            //yes -> move to next state 
           
            odometry.updatePose();          //seems to work

            Pose curPose = odometry.getCurPose();
            

            //Serial.println ("curPose : " + (String) curPose.x+ ", " +(String) curPose.y+ ", " + (String) (curPose.angle)) ;
            
            ctrl.setCurPose(curPose);
            if (ctrl.checkIfArrived()){
                //Serial.println (subPathIndex);
                if (subPathIndex == 0){
                    Serial.println ("FINISHED SEG 1");
                    subPathIndex=1;
                    machine.trigger("finished_seg1");
                    
                }
                else if (subPathIndex == 1){
                    Serial.println ("FINISHED SEG 2");
                    subPathIndex=2;
                    machine.trigger("finished_seg2");                    
                }
                else if (subPathIndex == 2) {
                    Serial.println ("FINISHED SEG 3");
                    subPathIndex=0;
                    machine.trigger("finished_seg3");
                }

                
            } 
           // else machine.trigger("wait");
        };

        void trigger(String arg)
        {
            machine.trigger(arg);
        };

        void triggerSetNewPose()
        {
            machine.trigger("set_new_pose");
        };

        void start(){
            machine.start();
        };

        void tick() {
            machine.tick();
        };

     
        protected:
            MotionController& ctrl;
            Odometry odometry = Odometry(ctrl.kinematics);
            bool first_on = true;
        


    }; // class MotionStateMachine

};  // namespace SmallRobots



