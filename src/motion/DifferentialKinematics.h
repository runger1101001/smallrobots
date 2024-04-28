
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

            virtual void move(float distance, float speed, float radius = RADIUS_STREIGHT);
            virtual void rotate(float angle, float speed);

            void moveTo(float dx, float dy, float da, float speed);

            DifferentialKinematics& kinematics;
    };

}; // namespace SmallRobots
