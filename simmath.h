/*******************************************************************************
This file contains datatype definitions and some macros for doing 
vector math. All of them are quite simple, but it makes the rest of 
simulator code easier to read and understand, as well as lowering 
the rate of (nasty) math bugs. At least that is the idea.

The basic datatype is a 2D vector called Vector2. It is a struct 
with two members: x and y

All macros do tricks with one or more of these entities.

*******************************************************************************/

#ifndef SIMMATH_H
#define SIMMATH_H



#include <math.h>
#include <vector>
#include <list>
#include <stdio.h>

using namespace std;

/******************************************************************************/
/******************************************************************************/

#define EPSILON 0.0000001
/**
 *  * @brief Conversion factor from degrees to radians.
 *   */
const double DEGREES_TO_RADIANS = M_PI / 180.0;

/**
 *  * @brief Conversion factor from radians to degrees.
 *   */
const double RADIANS_TO_DEGREES = 180.0 / M_PI;

/**
 *  * @brief 360° in radians
 *   */
const double TWO_PI             = 2 * M_PI;

/**
 *  * @brief A very small number, used to compare doubles.
 *   */
/*const double EPSYLON = 0.001;*/

/******************************************************************************/
// Data types:
/******************************************************************************/

struct dVector2
{
    double x;
    double y;
};


typedef vector<dVector2>            TPoints;
typedef vector<dVector2>::iterator  TPointsIterator;

extern dVector2 ZEROVECTOR2; 
extern dVector2 UNITVECTOR2; // { 1, 0 }


struct dRangeBound 
{
  char     name;
  double   angle;
  unsigned bound;
};

/******************************************************************************/
// Utility functions
/******************************************************************************/
double ModPI( double angle );
bool   RangeBoundLess( dRangeBound l1, dRangeBound l2 );
double CrossCorrelation( vector<double>* vec_first, vector<double>* vec_second, int n_num_steps );
bool DoubleEq( const double x, const double y );
bool DoubleEqAbsolute( const double x, const double y, const double absolute_epsylon );
bool DoubleGT( const double x, const double y );
bool DoubleLT( const double x, const double y );
double Max( const double v1, const double v2 );
double Max( const int v1, const int v2 );
double Min( const double v1, const double v2 );
double Min( const int v1, const int v2 );


/******************************************************************************/
// Vector2 macro functions:
/******************************************************************************/

// Returns the squared length of a vector
#define dVec2LengthSquared(vec) (vec.x*vec.x + vec.y*vec.y)

// Returns the length of a vector
#define dVec2Length(vec) sqrt(dVec2LengthSquared(vec))

// Returns the vector going _from_ A to B (thus B - A)
#define dVec2Sub(result, A, B)   \
 {                               \
     result.x = B.x - A.x;       \
     result.y = B.y - A.y;       \
 }
   
// Adds two vectors:
#define dVec2Add(result, A, B)   \
 {                               \
     result.x = B.x + A.x;       \
     result.y = B.y + A.y;       \
 }
   
// Returns the dot product of two vectors:
#define dVec2Dot(A, B) ((A.x * B.x) + (A.y * B.y))

// Projection of A onto B, 
// where B should be a unit vector (otherwise the result points in the right direction, but the length is multiplied by length(B)):
#define dVec2Projection(result, A, B) \
  {                                \
    result.x = dVec2Dot(A,B)*B.x;  \
    result.y = dVec2Dot(A,B)*B.y;  \
  }

// Set a vector to (0,0):
#define dVec2Zero(A)  { A.x = 0; A.y = 0; }

// Get the perpendicular dot product 
// (used when computing the torque of a ridget body):
#define dVec2PrepDot(R, F) (-R.y * F.x + R.x * F.y) 

// Rotate a vector:
#define dVec2Rotate(angle, vec)                         \
  {                                                     \
     double xt_ = vec.x;                                 \
     vec.x = cos(angle) * vec.x - sin(angle) * vec.y;   \
     vec.y = cos(angle) * vec.y + sin(angle) * xt_;     \
  }

// Normal of a vector (not to confuse with normalized vector!!!):
// result is the right hand normal of vec
#define dVec2Normal(result,vec)				\
  {                                                     \
     result.x = -vec.y;                                \
     result.y =  vec.x;                                \
  }


// Normalize a vector:
#define dVec2Normalize(vec)                             \
  {                                                     \
     double length___ = dVec2Length(vec);                \
     vec.x /= length___;                                \
     vec.y /= length___;                                \
  }


// Make vec perpendicular to itself
#define dVec2Perpendicular(vec) \
  {                             \
     double tempx___ = vec.x;    \
     vec.x  = -vec.y;           \
     vec.y  = tempx___;         \
  }

// Multiply a vector by a scalar
#define dVec2MultiplyScalar(vec, scalar) \
  {                                      \
      vec.x *= scalar;                   \
      vec.y *= scalar;                   \
  }                            
     
// Find the cos to the angle between two vectors
#define dVec2CosAngle(vec1, vec2) \
     ((vec1.x * vec2.x + vec1.y * vec2.y) / (dVec2Length(vec1) * dVec2Length(vec2))) 

// Find the angle between two vectors
#define dVec2Angle(vec1, vec2) \
     (acos(dVec2CosAngle(vec1, vec2)))

// Find the angle of one vector
#define dVec2OwnAngle(vec) \
     (atan2(vec.y, vec.x))

// Returns the normalized angle in the range [0,2*M_PI)
#define NormalizeAngle(ang) \
  (ang < 0 ? fmod(ang, 2*M_PI)+2*M_PI : fmod(ang, 2*M_PI))

// Returns the normalized angle in the range [-PI,PI)
#define NormalizeAngleNegativePIPositivePI(ang) \
  (NormalizeAngle(ang) > M_PI ? NormalizeAngle(ang) - 2*M_PI : NormalizeAngle(ang))


// Returns the distance between two points
#define dVec2Distance(A,B) sqrt(dVec2DistanceSquared(A,B))


// Returns the square distance between two points
#define dVec2DistanceSquared(A,B) ( (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) )

/******************************************************************************/
/******************************************************************************/
     
#endif
