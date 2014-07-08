#ifndef VECTORFIELD
#define VECTORFIELD

#include "torus.h"

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <time.h>

std::vector<std::vector<double> > circles;

// 25, 5, 20.5;
// center of the torus;

// radius of torus: 0.8 currently; can be larger can be smaller;
// torusRadius - 2*torusHeight
// around line 2332 in test_colab_cloth.cpp

void findCircles() {
	// for this rope world, we cheat a little bit, find circles on the line penetrating the 
	// torus;

	double start = 1;
	double end = 9;
	double step = 0.5;
	for (double i = start; i <= end; i += step) {
		std::vector<double> circle;
		circle.push_back(25);
		circle.push_back(i);
		circle.push_back(20.5);

		// next parameter is radius;

		if (i == 5) {
			circle.push_back(0.35);
		} else if (i == 4.5 || i == 5.5) {
			circle.push_back(0.5);
		} else {
			circle.push_back(0.81);
		}

		circle.push_back(0);
		circle.push_back(-1);
		circle.push_back(0);

		circles.push_back(circle);
	}
	
}

#endif