
#pragma once

#include <stdint.h>
#include <limits>
#include <Arduino.h>
#include "./Vector.h"
#include "./DifferentialKinematics.h"

namespace SmallRobots {

class DifferentialPathPlanner {

    private:

        Pose startPose, endPose; //start and end pose (x,y,angle)
        Vector S, E; //start and end vector of Pose for calculating
        Vector Sdir, Edir; //direction vector rotated around angle of pose

        Vector unitX = Vector(1,0,0);
        Vector unitZ = Vector(0,0,1);

        Vector R1, R2, R3, L1, L2, L3;

        Vector A, B, C, D;

        Vector T1, T2, T3, T4, T5, T6; //tangent points, R first
        Vector T7, T8, T9, T10, T11, T12; //tangent points, L first
        Vector T5dir, T11dir;

        String *name;
        String allNames [6] = {"RSL", "RSR", "RLR", "LSR", "LSL", "LRL"};
        float allLength [6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //order: RSLlen, RSRlen, RLRlen, LSRlen, LSLlen, LRLlen
        int shortestPathIndex = -1; //0=RSL, 1=RSR, 2=RLR, 3=LSR, 4=LSL, 5=LRL

        //helpers
        Vector unitV;
        Vector cross;
        Vector a, b, c, d;
        float alpha;
        Vector u, v;
        Vector RSL;
        Vector w;
        Vector sub;
        Vector LSR;

    public:
        DifferentialPathPlanner();
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
       // virtual void move(float distance, float speed, float radius = RADIUS_STREIGHT) = 0;
        //virtual void rotate(float angle, float speed) = 0;
        //virtual void move(float distance, float speed, float radius);
        //virtual void moveTo(float dx, float dy, float da, float speed);

        void calculate(Pose start, Pose end) ;

        int getShortestPathIndex ();
        String getShortestPathName();

        void setPathRadius(float r);

        float turnRadius = MINRADIUS; //radius of dubin path, variable

        Vector arcCenter1;
        float arcRadius1 = turnRadius;
        float arcAngle1 = 0;
        String arcDirName1 = "DUBIN1";

        Vector arcCenter2;
        float arcRadius2 = turnRadius;
        float arcAngle2 = 0;
        String arcDirName2 = "DUBIN3";

        Vector arcCenter12;
        float arcRadius12 = turnRadius;
        float arcAngle12 = 0;
        String arcDirName12 = "DUBIN2";

        Vector lineStart;
        Vector lineEnd;
        float lineLength = 0;

};


}; // namespace SmallRobots