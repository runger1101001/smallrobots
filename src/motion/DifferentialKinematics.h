
#include <stdint.h>
#include <limits>


#define RADIUS_STREIGHT (std::numeric_limits<float>::infinity())


namespace SmallRobots {


    class DifferentialKinematics {
        public:
            DifferentialKinematics(float wheel_base, float wheel_diameter);
            ~DifferentialKinematics();

            virtual void setSpeed(float left, float right) = 0;

            virtual void move(float speed, float radius = RADIUS_STREIGHT);
            virtual void rotate(float speed);
            virtual void stop() = 0;

            float half_wheel_base;
            float wheel_radius;
            float default_speed;
    };


    class DifferentialPathPlanner {
        public:
            DifferentialPathPlanner(DifferentialKinematics& kinematics);
            ~DifferentialPathPlanner();

            // TODO think  about where these methods belong
            // possible model to seperate the concepts
            //  - Kinematics - models the robot dimensions, type of drive etc, 
            //                 thus implementing raw motion e.g. like DifferentialKinematics
            //  - PathPlanner - composes paths in terms of the robot's capabilities, e.g. Point 
            //                  and Shoot, decomposing curves into circular arcs, etc.
            //  - "MotionController"? - executes paths in terms of raw movements over time/distance.
            //                          "DeadReconningMotionController", or "OdometricMotionController"
            //                          could be examples. Keeps state in terms of motions in progress,
            //                          and provides facilities to queue and/or interrupt motions.
            virtual void move(float distance, float speed, float radius = RADIUS_STREIGHT) = 0;
            virtual void rotate(float angle, float speed) = 0;

            virtual void moveTo(float dx, float dy, float da, float speed) = 0;

            DifferentialKinematics& kinematics;
    };

}; // namespace SmallRobots
