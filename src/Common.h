///////////////////////////////////////////////////////////////////////////////
//
// Common.h contains some common defines used in the rest of application files.
//
// Written and (c) by Radomir Stevanovic, Aug 2005.
// E-mail: stevie@ieee.org
//
///////////////////////////////////////////////////////////////////////////////


#ifndef COMMON_H
#define COMMON_H
#define USE_LASER
#define VELODYNE64
//#define BUILD_MAP
//#define USE_VTK
#define LOG_TIME
#define USE_CPP_COV
//#define USE_ODOM_COVARIANCE
//#define BUILD_GLOBAL_MAP
//#define SEND_MAP
//#define UPDATE_PLANES
//#define BRLLASER
//#define USE_COOP

#ifndef PI
#define PI	3.14159265358979323846
#endif

#ifndef _2PI
#define _2PI	6.28318530717959
#endif

#define APPROX_ZERO		1e-10

// used in iteration for matrix square root
// iteration stops if given accuracy is achieved or max. number of iterations passed
#define MAX_SQUARE_ROOT_ITER 4
#define SQUARE_ROOT_ACCURACY 0.01

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))


inline double minusOneRaisedTo(int exp) { return ( (exp % 2) == 0 ? 1 : -1 ); }

const double APPROX_EQUAL_EPS = 1e-8;
inline bool approx_equal(double a, double b, double eps = APPROX_EQUAL_EPS) { return (fabs(a-b) <= eps); }

// signum function: returns -1 for negative number, +1 otherwise
inline double sgn(double x) { return (x >= 0) ? 1.0 : -1.0; }

// just for/during debugging!
extern bool myDEBUG;


#endif	// COMMON_H
