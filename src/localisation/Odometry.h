#pragma once

#include <Arduino.h>
#include "./motion/DifferentialKinematics.h"

namespace SmallRobots {

    class Odometry{
        public:
            Odometry(DifferentialKinematics& _kinematics, EventBus<String>& event_bus);
            
            ~Odometry();
            void setup();
            void run();
            void updatePose(unsigned long _deltaT);
            void resetLastTime();
            Pose getCurPose();
            void resetCurPose(); //to start pose 0,0,0
            void setCurPose(float x, float y, float angle, AngleUnit angleUnit=AngleUnit::RADIANS);
            Pose getDeltaPose() { return deltaPose; }
            unsigned long getDeltaT() { return deltaT; }
            Pose getRawOdometryData() { return rawOdometryPose; } // Raw unfiltered odometry data
            float getRawOdometryAngle() { return rawOdometryPose.angle; } // Raw angle from local odometry

            // --- IMU heading fusion ---
            // Feed the absolute yaw of an external IMU (any zero point, CCW positive).
            // Only the change between calls is used; it replaces (weight 1) or blends with
            // (weight < 1) the encoder heading change. Falls back to encoders if the IMU goes stale.
            void addImuYaw(float yaw, AngleUnit angleUnit=AngleUnit::DEGREES);
            void setImuHeadingWeight(float weight) { imuWeight = constrain(weight, 0.0f, 1.0f); }
            void setImuTimeout(uint32_t ms) { imuTimeoutMs = ms; }
            bool isImuActive();

            // --- external pose fusion (e.g. camera tracking) ---
            // Complementary filter: pulls curPose a fraction alpha towards the tracked pose.
            // Angle in the global frame (same as setCurPose). Returns the correction applied
            // (internal frame) so other estimators can be shifted by the same amount.
            Pose fuseExternalPose(float x, float y, float angle, AngleUnit angleUnit=AngleUnit::DEGREES);
            void setExternalFusionGains(float alphaXY, float alphaAngle);
            void setExternalLatency(uint32_t ms) { externalLatencyMs = ms; } // tracker age when it arrives
            void setExternalSnapThreshold(float mm, float rad) { snapDistance = mm; snapAngle = rad; }

        protected:
            DifferentialKinematics& kinematics;
            EventBus<String>& event_bus;
            //Start Pose: location x,y and heading angle
            //Local coordinate system at the moment
            //angle = 0 heading in y direction
            Pose curPose = Pose(); 
            Pose deltaPose = Pose();
            Pose rawOdometryPose = Pose(); // Raw encoder/IMU data without external tracking influence
            
            int lastTime=0, deltaT=0; //delat T, read in micros
            uint32_t update_ms = 10;

            // IMU heading fusion
            float imuWeight = 0.95f;
            uint32_t imuTimeoutMs = 200;
            float lastImuYaw = 0.0f;
            float imuYawAccum = 0.0f;
            uint32_t lastImuMs = 0;
            bool imuHasReference = false;

            // external pose fusion
            float alphaXY = 0.3f, alphaAngle = 0.3f;
            uint32_t externalLatencyMs = 0;
            float snapDistance = 100.0f;       // mm, larger errors are snapped instead of blended
            float snapAngle = M_PI / 6.0f;     // rad
            bool hasExternalPose = false;

            // short pose history for latency compensation
            static const int HISTORY_LEN = 64;  // 64 * update_ms = 640 ms
            struct TimedPose { uint32_t t_ms; Pose pose; };
            TimedPose history[HISTORY_LEN];
            int historyHead = 0, historyCount = 0;
            void recordHistory();
            Pose poseAt(uint32_t t_ms);
    };

    extern Pose odometryPose;
    extern Pose odometryDeltaPose;
    extern unsigned long odometryDeltaT;
}; // namespace SmallRobots