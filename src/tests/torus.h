#ifndef TORUS
#define TORUS

#include <vector>
#include <cmath>

std::vector<std::vector<double> > csc; // representing closed-space-curve;

void makeTorus() {

	double radius = 2;
	double smallRadius = 0.7;

	double numSteps = 30;
	double step = 2*M_PI/30;
	// 5, 25. 20.5


	for (int i = 0; i < numSteps; i++) {
	std::vector<double> position;
	position.push_back(5);
	position.push_back(25+radius*cos(i*step));
	position.push_back(20.5+radius*sin(i*step));

	csc.push_back(position);
	}

}


#endif