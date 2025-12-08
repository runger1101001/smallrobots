#pragma once

#include <Arduino.h>
#include "./motion/structs.h"

namespace SmallRobots {

class StuckDetector {
public:

    StuckDetector(float distThresh = 1.0f, float angleThresh = 0.1f)
        : distanceThreshold(distThresh), angleThreshold(angleThresh) {}

    bool checkStuck(const Pose& odom, const Pose& deadReckoning) {
        float dx = odom.x - deadReckoning.x;
        float dy = odom.y - deadReckoning.y;
        float distance = sqrtf(dx * dx + dy * dy);

        float angleDiff = fabsf(odom.angle - deadReckoning.angle);
        // Normalize angle difference to [-π, π]
        while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
        if (angleDiff < -M_PI) angleDiff += 2 * M_PI;
        angleDiff = fabsf(angleDiff);

        return (distance > distanceThreshold) || (angleDiff > angleThreshold);
    }

    void reset(const Pose& odom, Pose& deadReckoning) {
        
        deadReckoning.x = odom.x;
        deadReckoning.y = odom.y;
        deadReckoning.angle = odom.angle;
    }

    void setThresholds(float distThresh, float angleThresh) {
        distanceThreshold = distThresh;
        angleThreshold = angleThresh;
    }


private:
    float distanceThreshold; // mm
    float angleThreshold;    // radians
};

}; // namespace SmallRobots