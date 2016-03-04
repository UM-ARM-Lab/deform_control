#ifndef COLAB_CLOTH_DEFINES_H
#define COLAB_CLOTH_DEFINES_H

//#define PROFILER
//#define USE_PR2
//#define USE_QUATERNION //NOT IMPLEMENTED!!!
//#define USE_TABLE

//#define USE_ADAPTIVE_JACOBIAN //not working
#define USE_RADIUS_CONTACT
#define PRESERVE_LENGTH
#define AVOID_COLLISION

#ifdef USE_PR2
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include "robots/grabbing.h"
#endif

//////////////////
////Rope with Cylinder Coverage
//#define USE_RADIUS_CONTACT
//#define DO_COVERAGE
//#define ROPE
//#define ROTATION_SCALING 50.0f
//#define DO_ROTATION



////////////////////////
////Cloth covering table
#define DO_COVERAGE
#define DO_ROTATION
#define ROTATION_SCALING 1.0f


////////////////////////
////Colab Cloth folding
//#define DO_ROTATION
//#define ROTATION_SCALING 1.0f


//#define USE_NOISE

#endif // COLAB_CLOTH_DEFINES_H
