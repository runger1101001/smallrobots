#include "./Odometry.h"

namespace SmallRobots {

    Pose odometryPose;
    Pose odometryDeltaPose;
    unsigned long odometryDeltaT;

    Odometry::Odometry(DifferentialKinematics& _kinematics, EventBus<String>& event_bus): kinematics(_kinematics), event_bus(event_bus)
    {
        
    };
    Odometry::~Odometry()
    {};

    void Odometry::setup(){
        lastTime = micros();
    };

    void Odometry::run(){

        unsigned long now = micros();
        if (now - lastTime > update_ms*1000){
            deltaT = now - lastTime;  // Update global deltaT
            lastTime = now;

            updatePose(deltaT);
            recordHistory();
            odometryPose = getCurPose(); // Update global pose
            odometryDeltaPose = getDeltaPose();  // Update global deltaPose
            odometryDeltaT = getDeltaT();  // Update global deltaT
            //event_bus.emit(String("new_odometry_pose"));
        }
    };

    static float wrapPi(float a){
        while (a >  M_PI) a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }

    void Odometry::updatePose(unsigned long _deltaT)
    {
        deltaPose = kinematics.getDeltaPose(_deltaT,curPose, "odometry");

        // heading change: encoders, optionally blended with the IMU
        float dAngle = deltaPose.angle;
        if (isImuActive()) {
            dAngle = imuWeight * imuYawAccum + (1.0f - imuWeight) * deltaPose.angle;
        }
        imuYawAccum = 0.0f;

        // integrate x/y along the midpoint heading (more accurate on arcs)
        float heading = curPose.angle + kinematics.globalCoordinateSystemOffsetAngle;
        float ds = deltaPose.x * cosf(heading) + deltaPose.y * sinf(heading);
        deltaPose.x = ds * cosf(heading + dAngle / 2.0f);
        deltaPose.y = ds * sinf(heading + dAngle / 2.0f);
        deltaPose.angle = dAngle;

        curPose.x += deltaPose.x;
        curPose.y += deltaPose.y;
        curPose.angle += deltaPose.angle;
        
        // Also update raw odometry data (independent of external tracking)
        rawOdometryPose.x += deltaPose.x;
        rawOdometryPose.y += deltaPose.y;
        rawOdometryPose.angle += deltaPose.angle;
    };

    void Odometry::resetLastTime(){
        lastTime = micros();
    };

    
    Pose Odometry::getCurPose()
    {
        return curPose;
    };

    void Odometry::resetCurPose(){
        curPose.x = 0;
        curPose.y = 0;
        curPose.angle = 0;
        rawOdometryPose.x = 0;
        rawOdometryPose.y = 0;
        rawOdometryPose.angle = 0;
        historyCount = 0;
    }; 
    void Odometry::setCurPose(float x, float y, float angle, AngleUnit angleUnit){
        curPose.x = x;
        curPose.y = y;
        if (angleUnit == AngleUnit::DEGREES) {
            curPose.angle = angle * M_PI / 180.0f - kinematics.globalCoordinateSystemOffsetAngle;
        } else {
            curPose.angle = angle - kinematics.globalCoordinateSystemOffsetAngle;
        }
        historyCount = 0; // old history is meaningless after a jump
    };


    // ---------------- IMU heading fusion ----------------

    void Odometry::addImuYaw(float yaw, AngleUnit angleUnit){
        if (angleUnit == AngleUnit::DEGREES) yaw = yaw * M_PI / 180.0f;
        uint32_t now = millis();
        if (imuHasReference && now - lastImuMs <= imuTimeoutMs) {
            imuYawAccum += wrapPi(yaw - lastImuYaw);
        } else {
            imuYawAccum = 0.0f; // (re)start: first sample only sets the reference
        }
        lastImuYaw = yaw;
        lastImuMs = now;
        imuHasReference = true;
    };

    bool Odometry::isImuActive(){
        return imuHasReference && imuWeight > 0.0f && millis() - lastImuMs <= imuTimeoutMs;
    };


    // ---------------- external pose fusion ----------------

    void Odometry::setExternalFusionGains(float _alphaXY, float _alphaAngle){
        alphaXY = constrain(_alphaXY, 0.0f, 1.0f);
        alphaAngle = constrain(_alphaAngle, 0.0f, 1.0f);
    };

    Pose Odometry::fuseExternalPose(float x, float y, float angle, AngleUnit angleUnit){
        if (angleUnit == AngleUnit::DEGREES) angle = angle * M_PI / 180.0f;
        angle -= kinematics.globalCoordinateSystemOffsetAngle; // to internal frame

        // compare with where we thought we were when the tracker saw us
        Pose then = (externalLatencyMs > 0) ? poseAt(millis() - externalLatencyMs) : curPose;
        Pose err;
        err.x = x - then.x;
        err.y = y - then.y;
        err.angle = wrapPi(angle - then.angle);

        Pose corr;
        bool snap = !hasExternalPose
                 || sqrtf(err.x * err.x + err.y * err.y) > snapDistance
                 || fabsf(err.angle) > snapAngle;
        if (snap) {
            corr = err;
        } else {
            corr.x = alphaXY * err.x;
            corr.y = alphaXY * err.y;
            corr.angle = alphaAngle * err.angle;
        }
        hasExternalPose = true;

        curPose.x += corr.x;
        curPose.y += corr.y;
        curPose.angle += corr.angle;
        // shift history too, so a later lookup does not apply the same correction twice
        for (int i = 0; i < historyCount; i++) {
            history[i].pose.x += corr.x;
            history[i].pose.y += corr.y;
            history[i].pose.angle += corr.angle;
        }
        odometryPose = curPose;
        return corr;
    };

    void Odometry::recordHistory(){
        history[historyHead].t_ms = millis();
        history[historyHead].pose = curPose;
        historyHead = (historyHead + 1) % HISTORY_LEN;
        if (historyCount < HISTORY_LEN) historyCount++;
    };

    Pose Odometry::poseAt(uint32_t t_ms){
        // walk back from newest to the first entry not newer than t_ms
        Pose p = curPose;
        for (int i = 1; i <= historyCount; i++) {
            const TimedPose& h = history[(historyHead - i + HISTORY_LEN) % HISTORY_LEN];
            p = h.pose;
            if ((int32_t)(t_ms - h.t_ms) >= 0) break;
        }
        return p; // oldest entry if the latency exceeds the history
    };


}; // namespace SmallRobots