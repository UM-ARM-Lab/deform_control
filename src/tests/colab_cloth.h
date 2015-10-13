#ifndef COLAB_CLOTH_H
#define COLAB_CLOTH_H

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/rope.h"

#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>

//#define PROFILER
//#define USE_PR2
//#define USE_QUATERNION //NOT IMPLEMENTED!!!
//#define USE_TABLE

//#define USE_ADAPTIVE_JACOBIAN //not working
#define USE_RADIUS_CONTACT
#define PRESERVE_LENGTH
#define AVOID_COLLISION

//////////////////
////Rope with Cylinder Coverage
#define USE_RADIUS_CONTACT
#define DO_COVERAGE
#define ROPE
#define ROTATION_SCALING 0.05f
#define DO_ROTATION

////////////////////////
////Cloth covering table
//#define DO_COVERAGE
//#define DO_ROTATION
//#define ROTATION_SCALING 50.0f


////////////////////////
////Colab Cloth folding
//#define DO_ROTATION
//#define ROTATION_SCALING 1.0f

// #define USE_NOISE

#endif // COLAB_CLOTH_H
