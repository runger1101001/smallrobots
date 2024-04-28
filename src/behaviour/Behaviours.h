

#pragma once


#include <set>
#include <Arduino.h>

#define BEHAVIOURENGINE_DEFAULT_MAX_IDLE_TIME 10000

namespace SmallRobots {


    class Behaviour {
    public:
        Behaviour(int _rate = 1000) : rate(_rate) {};
        virtual Behaviour* run() = 0;
        virtual const char* getName() = 0;
        
        int rate = 0; // in milliseconds, 0 is off
        long nextRun = 0;
        long lastRun = -1;
    };



    class BehaviourEngine {
    public:
        BehaviourEngine();

        // TODO rename these methods to make clear what is running and what is just in the set
        // TODO add method to start/stop behaviours by name
        // TODO add method to list all behaviours
        void add(Behaviour* behaviour);
        void remove(Behaviour* behaviour);
        void replace(Behaviour* oldBehaviour, Behaviour* newBehaviour);
        void delay(Behaviour* behaviour, long ms);
        bool isRunning(Behaviour* behaviour);

        bool find(Behaviour* behaviour);
        Behaviour* find(String name);
        void put(Behaviour* behaviour);

        void init();
        long run();

        int maxIdleTime = BEHAVIOURENGINE_DEFAULT_MAX_IDLE_TIME;
        // set this if you want to the engine to start a behaviour whenever it is idle for at least maxIdleTime milliseconds
        Behaviour* idleBehaviour=0;
        // set this if you want the engine to start a behaviour when there are no other behaviours active
        Behaviour* respawnBehaviour=0;
        bool debug = false;
    private:
        std::set<Behaviour*> behaviours;    // use a set for now, but something like a priority queue would be better
    };


    extern BehaviourEngine behaviours;

};