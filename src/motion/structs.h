#pragma once
#include <Arduino.h>

namespace SmallRobots {

    enum class AngleUnit {
        RADIANS,
        DEGREES
    };


    struct RobotState {
        Vector2D position;
        float angle;
        float vx, vy;
    };

    typedef struct Pose {
        float x =0;
        float y =0;
        float angle=0;
    } Pose;

    typedef struct Circle {
        float x = 0;
        float y = 0;
        float radius = 0;
    } Circle;

    typedef struct Line {
        float x1 = 0;
        float y1 = 0;
        float x2 = 0;
        float y2 = 0;
    } Line;

}