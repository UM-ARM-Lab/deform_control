#ifndef VECTORFIELD
#define VECTORFIELD

#include "torus.h"

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <time.h>

std::vector<std::vector<double> > circles;
std::vector<bool> penetrated;
// 25, 5, 20.5;
// center of the torus;

// radius of torus: 0.8 currently; can be larger can be smaller;
// torusRadius - 2*torusHeight
// around line 2332 in test_colab_cloth.cpp

void findCircles() {
	// for this rope world, we cheat a little bit, find circles on the line penetrating the 
	// torus;

	// double start = 2;
	// double end = 8;
	// double step = 0.25;
	// srand(time(NULL));
	// // This is next step: multiple rings;
	// // First try only one right;
	// double rx, rz, rr;
	// for (double i = start; i <= end; i += step) {
	// 	rx = (rand()/(RAND_MAX+1.0)) * 1 + 24.5;
 //        rz = (rand()/(RAND_MAX+1.0)) * 1 + 20;
 //        if (i == 5) {
 //        	rx = 25;
 //        	rz = 20.5;
 //        } else if (i >= 4 && i <= 6) {
 //        	rx = 25;
 //        	rz = 20.5;
 //        } 
	// 	std::vector<double> circle;
	// 	//circle.push_back(25);
	// 	circle.push_back(rx);
	// 	circle.push_back(i);

	// 	//circle.push_back(20.5);
	// 	circle.push_back(rz);
	// 	// next parameter is radius;

	// 	if (i == 5) {
	// 		circle.push_back(0.25);
	// 	} else if (i >= 4 && i <= 6) {
	// 		circle.push_back(0.35);
	// 	} else {
	// 		//circle.push_back(0.81);
	// 		rr = (rand()/(RAND_MAX+1.0)) * 0.5 + 0.51;
	// 		circle.push_back(rr);
	// 	}

	// 	circle.push_back(0);
	// 	circle.push_back(-1);
	// 	circle.push_back(0);

	// 	circles.push_back(circle);
	// }

	// Now, work on finding circles for 3 torus;

	

	// manually creating circles;
	// both guiding circles and target circles;
	// radius difference must be small;
	// also remember to change the strategy of 
	// activiting circles to "disable after penetrate"

	for (int i = 2; i <= 14; i++) {
		std::vector<double> circle;

		// center x
		if (i <= 5) {
			circle.push_back(25);
		} else if (i == 6) {
			circle.push_back(24.66);
		} else if (i == 7) {
			circle.push_back(24.33);
		} else if (i == 8) {
			circle.push_back(24);
		} else if (i == 9) {
			circle.push_back(24.67);
		} else if (i == 10) {
			circle.push_back(25.33);
		} else if (i >= 11) {
			circle.push_back(26);
		}

		// center y
		circle.push_back(i);

		// center z
		if (i <= 5) {
			circle.push_back(20.5);
		} else if (i == 6) {
			circle.push_back(21.13);
		} else if (i == 7) {
			circle.push_back(21.76);
		} else if (i == 8) {
			circle.push_back(22.5);
		} else if (i == 9) {
			circle.push_back(22.2);
		} else if (i == 10) {
			circle.push_back(21.8);
		} else if (i >= 11) {
			circle.push_back(21.5);
		}

		// radius
		if (i == 5 || i == 8 || i == 11) {
			circle.push_back(0.25);
		} else {
			circle.push_back(0.6);
		}
		
		// First test all circles still normal along y;
		// normal x
		circle.push_back(0);
		// normal y
		circle.push_back(1);
		// normal z
		circle.push_back(0);



		circles.push_back(circle);
		penetrated.push_back(false);
	}

	// end of creating guiding circles;

	/*
	double step = 2*M_PI / 30;
    srand(time(NULL));
    double rx, ry, rz, rr;
    for (double p = 2; p <= 8; p += 0.5) {
        rx = (rand()/(RAND_MAX+1.0)) * 10 + 20;
        ry = p;
        rz = (rand()/(RAND_MAX+1.0)) * 10 + 15.5;
        if (p != 5) {
            rr = (rand()/(RAND_MAX+1.0)) * 0.5 + 0.51;
        } else {
            rr = 0.25;
        }
        for (int index = 0; index < 30; index ++) {
            btVector3 point;
            point.setX(rx+rr*cos(index*step));
            point.setY(ry);
            point.setZ(rz+rr*sin(index*step));
            testPlotting.push_back(point);
        }

    }
    */

	// This is only one ring at where the original torus
	// was;

	// std::vector<double> circle;

	// circle.push_back(25);
	// circle.push_back(5);
	// circle.push_back(20.5);
	// circle.push_back(0.2);

	// circle.push_back(0);
	// circle.push_back(-1);
	// circle.push_back(0);

	// circles.push_back(circle);
	
}

#endif