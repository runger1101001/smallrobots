
#pragma once
#include <inttypes.h>
#include <math.h>


namespace SmallRobots {

    class Vector {
    public:

        float x = 0.f;
        float y = 0.f;
        float z = 0.f;

        Vector (float x=0, float y=0, float z=0);
   
        Vector operator*(float v) {
           Vector B;
           B.x = this->x *v;
           B.y = this->y *v;
           B.z = this->z *v;
           return B;
        };

        Vector operator+(Vector A) {
           Vector B;
           B.x = this->x +A.x;
           B.y = this->y +A.y;
           B.z = this->z +A.z;
           return B;
        };

        Vector operator-(Vector A) {
           Vector B;
           B.x = this->x -A.x;
           B.y = this->y -A.y;
           B.z = this->z -A.z;
           return B;
        };
    
    };

    Vector rotation (Vector& dir, float& angle);

    float circularArcLengthCW (Vector& dirA, Vector& A, Vector& B, float& radius); 
    float circularArcAngleCW( Vector& dirA, Vector& A, Vector& B);
    float circularArcLengthCCW (Vector& dirA, Vector& A, Vector& B, float& radius); 
    float circularArcAngleCCW( Vector& dirA, Vector& A, Vector& B);

    Vector crossProduct (Vector& A, Vector& B);
    float scalarProduct(Vector& A, Vector& B);

    float magnitude (Vector& A);
    Vector unit (Vector& A);
    float distance (Vector& A, Vector& B);
    
}; //end: namespace SmallRobots