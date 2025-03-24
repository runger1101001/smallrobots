#include "Arduino.h"
#include "./Behaviours.h"
#include "../control/SmallRobotEventBus.h"
#include "SmallRobotDebug.h"



namespace SmallRobots {

    BehaviourEngine behaviours;


    BehaviourEngine::BehaviourEngine(){};

    void BehaviourEngine::init(){
  
    };


    void BehaviourEngine::add(Behaviour* behaviour){
        if (behaviour != NULL) {
            if (behaviours.insert(behaviour).second){
                if (debug && smallrobot_debug_print!=nullptr){
                    smallrobot_debug_print->print("Starting: ");
                    smallrobot_debug_print->println(behaviour->getName());
                }
            }
        }
    };
    void BehaviourEngine::remove(Behaviour* behaviour){
        if (behaviour != NULL) {
            behaviours.erase(behaviour);
            if (debug && smallrobot_debug_print!=nullptr){
                smallrobot_debug_print->print("Finished: ");
                smallrobot_debug_print->println(behaviour->getName());
            }
        }
    };
    void BehaviourEngine::replace(Behaviour* oldBehaviour, Behaviour* newBehaviour){
        remove(oldBehaviour);
        add(newBehaviour);
    };
    void BehaviourEngine::delay(Behaviour* behaviour, long ms){
        long now = millis();
        behaviour->nextRun = now + ms;
    };
    bool BehaviourEngine::isRunning(Behaviour* behaviour){
        return (behaviours.find(behaviour) != behaviours.end());
    };
    bool BehaviourEngine::find(Behaviour* behaviour){
        // TODO all behaviours, not just the running ones
        return (behaviours.find(behaviour) != behaviours.end());
    };
    Behaviour* BehaviourEngine::find(String name){
        for (std::set<Behaviour*>::iterator it=behaviours.begin(); it!=behaviours.end(); ++it){
            Behaviour* behaviour = *it;
            if (name == behaviour->getName())
                return behaviour;
        }
        return NULL;
    };
    void BehaviourEngine::put(Behaviour* behaviour){
        if (behaviour != NULL) {
            // TODO all behaviours, not the running ones
            behaviours.insert(behaviour);
        }
    };



    long BehaviourEngine::run(){
        long now = millis();
        long nextScheduledRun = INT32_MAX;
        for (std::set<Behaviour*>::iterator it=behaviours.begin(); it!=behaviours.end(); ++it){
            Behaviour* behaviour = *it;
            if (behaviour->nextRun <= now){
                Behaviour* nextBehaviour = behaviour->run();
                behaviour->lastRun = now;
                if (behaviour->nextRun <= now)
                    behaviour->nextRun = now + behaviour->rate;
                if (nextBehaviour != behaviour)
                    replace(behaviour, nextBehaviour);
                if (nextBehaviour!=NULL && nextBehaviour->nextRun < nextScheduledRun)
                    nextScheduledRun = nextBehaviour->nextRun;
            }
            else 
                if (behaviour->nextRun < nextScheduledRun)
                    nextScheduledRun = behaviour->nextRun;
        }
        now = millis();
        if (nextScheduledRun > (now + maxIdleTime)) {
            if (idleBehaviour)
                add(idleBehaviour);
            nextScheduledRun = now;
        }
        return nextScheduledRun;
    };





};
