#include "Vector.h"

namespace SmallRobots {


Vector::Vector (float x, float y, float z){
  this->x = x;
  this->y = y;
  this->z = z;
};

// TODO make all these functions member functions of the Vector class 

Vector rotation (Vector& dir, float& angle) {
  float r = (angle);
  float xx  =  dir.x * cos ((r)) -  dir.y * sin ((r));
  float yy = dir.x * sin ((r)) +  dir.y * cos ((r));
  Vector P = Vector (xx, yy, dir.z);
  return P;
};

float circularArcLengthCW (Vector& dirA, Vector& A, Vector& B, float& radius) {
  float alpha = circularArcAngleCW(dirA, A, B);
  float s = radius * alpha;
  return s;
};

float circularArcAngleCW( Vector& dirA, Vector& A, Vector& B) {
  Vector unitA = unit(dirA);
  Vector AB = B - A;
  float sca = scalarProduct(unitA, AB);
  float beta = acos( sca / (magnitude (unitA) * magnitude(AB) ) );
  float alpha = 2*beta;
  return alpha;
};

float circularArcLengthCCW (Vector& dirA, Vector& A, Vector& B, float& radius) {
  float alpha = circularArcAngleCCW(dirA, A, B);
  float s = radius * alpha;
  return s;
}

float circularArcAngleCCW( Vector& dirA, Vector& A, Vector& B) {
  Vector unitA = unit(dirA);
  Vector AB = B - A;
  float sca = scalarProduct(unitA, AB);
  float beta = acos( sca / (magnitude (unitA) * magnitude(AB) ) );
  float alpha = 2*M_PI - 2*beta;
  return alpha; 
}




Vector crossProduct (Vector& A, Vector& B)
{
  Vector C ;

  C.x = A.y*B.z - A.z*B.y;
  C.y = A.z*B.x - A.x*B.z;
  C.z = A.x*B.y - A.y*B.x;

  return C;
};

float scalarProduct(Vector& A, Vector& B)
{
  float scalar = A.x*B.x + A.y*B.y + A.z*B.z;
  return scalar;
};

float magnitude (Vector &A)
{
    float mag = sqrt(  A.x*A.x + A.y*A.y + A.z*A.z);
    return mag;
};
Vector unit (Vector &A)
{
    Vector U;
    float mag = magnitude(A);
    if(mag >0)
    {
        U.x = A.x/mag;
        U.y = A.y/mag;
        U.z = A.z/mag;
    }
    return U;
}

 float distance (Vector& A, Vector& B)
 {
    float dist = sqrt( ((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y)));
    return dist;
 };


};//end namespace SmallRobots