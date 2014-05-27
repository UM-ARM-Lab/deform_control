#ifndef FIND_PROJECTION
#define FIND_PROJECTION

#include "torus.h"

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

std::vector<double> findDirection (std::vector<std::vector<double> > curve);

std::vector<std::vector<double> > findProjection (std::vector<std::vector<double> > curve, 
	std::vector<double> normal);

void completeLoop (std::vector<std::vector<double> > curve);

#endif