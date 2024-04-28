

#include "./DifferentialKinematics.h"


namespace SmallRobots {

    DifferentialKinematics::DifferentialKinematics(float wheel_base, float wheel_diameter) {
        default_speed = 0.5;
        half_wheel_base = wheel_base/2.0f;
        wheel_radius = wheel_diameter/2.0f;
    };

    DifferentialKinematics::~DifferentialKinematics() {
    };


    void DifferentialKinematics::move(float speed, float radius) {
        if (radius == RADIUS_STREIGHT) {
            setSpeed(speed, speed);
        }
        else if (radius == 0.0) {
            setSpeed(speed, -speed);
        }
        else {
            float left = speed * (radius + half_wheel_base) / radius;
            float right = speed * (radius - half_wheel_base) / radius;
            setSpeed(left, right);
        }
    };



    void DifferentialKinematics::rotate(float speed) {
        move(speed, 0.0f);
    };




    DifferentialPathPlanner::DifferentialPathPlanner(DifferentialKinematics& kinematics) : kinematics(kinematics) {
    };

    DifferentialPathPlanner::~DifferentialPathPlanner() {
    };



    void DifferentialPathPlanner::move(float distance, float speed, float radius) {
        // TODO: implement
    };


    void DifferentialPathPlanner::rotate(float angle, float speed) {
        float distance = kinematics.half_wheel_base * angle;
        move(distance, speed, 0.0);
    };


}; // namespace SmallRobots