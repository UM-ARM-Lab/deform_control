#ifndef FIND_PROJECTION
#define FIND_PROJECTION

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <vector>

#include <time.h>

std::vector<double> find_projection();

std::vector<double> find_center();

std::vector<double> find_projected_curve();

std::vector<double> BiotSavart(std::vector<double> point, 
	std::vector<std::vector<double> > curve);

std::vector<double> crossProduct (std::vector<double> u, std::vector<double> v);

std::vector<std::vector<double> > findDirection (std::vector<std::vector<double> > curve);

std::vector<std::vector<double> > findProjection (std::vector<std::vector<double> > curve, 
	std::vector<double> normal);

std::vector<std::vector<double> > completeLoop (std::vector<std::vector<double> > curve);

int numIntersection (std::vector<std::vector<double> > curve);

bool HopfLink (std::vector<std::vector<double> > curve1, 
				std::vector<std::vector<double> > curve2);

bool lineSegmentIntersect (double x1c, double y1c, double x1n, double y1n, 
							double x2c, double y2c, double x2n, double y2n);

bool overlap (double a1, double b1, double a2, double b2);

bool inRange (double x, double a, double b);

// return the index of the last intersection
int findLastIntersection (std::vector<std::vector<double> > curve);

// find the points of comparison when only 1 intersection;
bool findPoints (std::vector<std::vector<double> > curve, std::vector<double> center);

// another function to test if two points are on the same side;

//

bool threaded (std::vector<std::vector<double> > curve);

// when inside, switch to another control strategy: let the tip follow the magnetic force;
// when stuck, release, follow new strategy, let the new gripping point follow 
// the magnetic force;
// regrasp


std::vector<double> findLargestCircle(std::vector<std::vector<double> > curve);

double findIntersection(std::vector<std::vector<double> > curve, 
						std::vector<double> derivate, 
						std::vector<double> point);

int circleCurveIntersection(std::vector<std::vector<double> > curve, 
							std::vector<double> circle);


#endif