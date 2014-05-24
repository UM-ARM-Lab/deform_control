#ifndef FIND_PROJECTION
#define FIND_PROJECTION

#include "torus.h"

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <vector>

std::vector<double> find_projection();

std::vector<double> find_center();

std::vector<double> find_projected_curve();

std::vector<double> BiotSavart(std::vector<double> point, 
	std::vector<std::vector<double> > curve);

std::vector<double> crossProduct (std::vector<double> u, std::vector<double> v);

#endif