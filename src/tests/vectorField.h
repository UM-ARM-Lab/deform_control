#ifndef VECTORFIELD
#define VECTORFIELD

#include "torus.h"

#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <time.h>

std::vector<std::vector<double> > circles;
std::vector<std::vector<double> > rings;
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

 //    std::vector<double> circle0;
 //    circle0.push_back(25);
 //    circle0.push_back(2);
 //    circle0.push_back(20.5);
 //    circle0.push_back(0.4);
 //    circle0.push_back(0);
 //    circle0.push_back(1);
 //    circle0.push_back(0);


 //    circles.push_back(circle0);
 //    penetrated.push_back(false);

 //    std::vector<double> circle;
 //    circle.push_back(25);
 //    circle.push_back(5);
 //    circle.push_back(20.5);
 //    circle.push_back(0.15);
 //    circle.push_back(0);
 //    circle.push_back(1);
 //    circle.push_back(0);


	// circles.push_back(circle);
 //    penetrated.push_back(false);

 //    std::vector<double> circle1;
 //    circle1.push_back(25);
 //    circle1.push_back(8);
 //    circle1.push_back(20.5);
 //    circle1.push_back(0.15);
 //    circle1.push_back(0);
 //    circle1.push_back(1);
 //    circle1.push_back(0);


 //    circles.push_back(circle1);
 //    penetrated.push_back(false);


 //    std::vector<double> ring0;
 //    ring0.push_back(25);
 //    ring0.push_back(2);
 //    ring0.push_back(20.5);
 //    ring0.push_back(0.8);
 //    ring0.push_back(0);
 //    ring0.push_back(1);
 //    ring0.push_back(0);


 //    rings.push_back(ring0);

 //    std::vector<double> ring;
 //    ring.push_back(25);
 //    ring.push_back(5);
 //    ring.push_back(20.5);
 //    ring.push_back(0.3);
 //    ring.push_back(0);
 //    ring.push_back(1);
 //    ring.push_back(0);


 //    rings.push_back(ring);

 //    std::vector<double> ring1;
 //    ring1.push_back(25);
 //    ring1.push_back(8);
 //    ring1.push_back(20.5);
 //    ring1.push_back(0.3);
 //    ring1.push_back(0);
 //    ring1.push_back(1);
 //    ring1.push_back(0);


 //    rings.push_back(ring1);


	// manually creating circles;
	// both guiding circles and target circles;
	// radius difference must be small;
	// also remember to change the strategy of 
	// activiting circles to "disable after penetrate"

	// for (int i = 2; i <= 14; i++) {
	// 	std::vector<double> circle;

	// 	// center x
	// 	if (i <= 5) {
	// 		circle.push_back(25);
	// 	} else if (i == 6) {
	// 		circle.push_back(24.66);
	// 	} else if (i == 7) {
	// 		circle.push_back(24.33);
	// 	} else if (i == 8) {
	// 		circle.push_back(24);
	// 	} else if (i == 9) {
	// 		circle.push_back(24.67);
	// 	} else if (i == 10) {
	// 		circle.push_back(25.8);
	// 	} else if (i >= 11) {
	// 		circle.push_back(26);
	// 	}

	// 	// center y
	// 	circle.push_back(i);

	// 	// center z
	// 	if (i <= 5) {
	// 		circle.push_back(20.5);
	// 	} else if (i == 6) {
	// 		circle.push_back(21.13);
	// 	} else if (i == 7) {
	// 		circle.push_back(21.76);
	// 	} else if (i == 8) {
	// 		circle.push_back(22.5);
	// 	} else if (i == 9) {
	// 		circle.push_back(22.2);
	// 	} else if (i == 10) {
	// 		circle.push_back(21.8);
	// 	} else if (i >= 11) {
	// 		circle.push_back(21.5);
	// 	}

	// 	// radius
	// 	if (i == 5 || i == 8 || i == 11) {
	// 		circle.push_back(0.25);
	// 	} else {
	// 		circle.push_back(0.6);
	// 	}
		
	// 	// First test all circles still normal along y;
	// 	// normal x
	// 	circle.push_back(0);
	// 	// normal y
	// 	circle.push_back(1);
	// 	// normal z
	// 	circle.push_back(0);



	// 	circles.push_back(circle);
	// 	penetrated.push_back(false);
	// }

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
	
    std::vector<double> circle;
    circle.push_back(26);
    circle.push_back(0);
    circle.push_back(20.7);
    circle.push_back(0.3);
    circle.push_back(0);
    circle.push_back(1);
    circle.push_back(0);


    circles.push_back(circle);
    penetrated.push_back(false);

    std::vector<double> circle1;
    
    circle1.push_back(20+3*sqrt(2));
    circle1.push_back(3*sqrt(2));
    circle1.push_back(20.5);
    circle1.push_back(0.3);
    circle1.push_back(-sqrt(2)/2);
    circle1.push_back(sqrt(2)/2);
    circle1.push_back(0);
    circles.push_back(circle1);
    penetrated.push_back(false);

    std::vector<double> circle2;
    
    circle2.push_back(20);
    circle2.push_back(6);
    circle2.push_back(20.5);
    circle2.push_back(0.3);
    circle2.push_back(-1);
    circle2.push_back(0);
    circle2.push_back(0);
    circles.push_back(circle2);
    penetrated.push_back(false);

    std::vector<double> circle3;
    
    circle3.push_back(20-3*sqrt(2));
    circle3.push_back(3*sqrt(2));
    circle3.push_back(20.5);
    circle3.push_back(0.3);
    circle3.push_back(-sqrt(2)/2);
    circle3.push_back(-sqrt(2)/2);
    circle3.push_back(0);
    circles.push_back(circle3);
    penetrated.push_back(false);

    std::vector<double> circle4;
    
    circle4.push_back(14);
    circle4.push_back(0);
    circle4.push_back(20.5);
    circle4.push_back(0.3);
    circle4.push_back(0);
    circle4.push_back(-1);
    circle4.push_back(0);
    circles.push_back(circle4);
    penetrated.push_back(false);

    std::vector<double> circle5;
    
    circle5.push_back(20-3*sqrt(2));
    circle5.push_back(-3*sqrt(2));
    circle5.push_back(20.5);
    circle5.push_back(0.3);
    circle5.push_back(sqrt(2)/2);
    circle5.push_back(-sqrt(2)/2);
    circle5.push_back(0);
    circles.push_back(circle5);
    penetrated.push_back(false);

    std::vector<double> circle6;
    
    circle6.push_back(20);
    circle6.push_back(-6);
    circle6.push_back(20.5);
    circle6.push_back(0.3);
    circle6.push_back(1);
    circle6.push_back(0);
    circle6.push_back(0);
    circles.push_back(circle6);
    penetrated.push_back(false);

    std::vector<double> circle7;
    
    circle7.push_back(20+3*sqrt(2));
    circle7.push_back(-3*sqrt(2));
    circle7.push_back(20.5);
    circle7.push_back(0.3);
    circle7.push_back(sqrt(2)/2);
    circle7.push_back(sqrt(2)/2);
    circle7.push_back(0);
    circles.push_back(circle7);
    penetrated.push_back(false);

    // std::vector<double> circle;
    // circle.push_back(20);
    // circle.push_back(0);
    // circle.push_back(20.5);
    // circle.push_back(0.3);
    // circle.push_back(0);
    // circle.push_back(0);
    // circle.push_back(1);


    // circles.push_back(circle);
    // penetrated.push_back(false);

    // std::vector<double> circle1;
    // circle1.push_back(20);
    // circle1.push_back(6-3*sqrt(2));
    // circle1.push_back(20.5+3*sqrt(2));
    // circle1.push_back(0.3);
    // circle1.push_back(0);
    // circle1.push_back(sqrt(2)/2);
    // circle1.push_back(sqrt(2)/2);


    // circles.push_back(circle1);
    // penetrated.push_back(false);

    // std::vector<double> circle2;
    // circle2.push_back(20);
    // circle2.push_back(6);
    // circle2.push_back(20.5+6);
    // circle2.push_back(0.3);
    // circle2.push_back(0);
    // circle2.push_back(1);
    // circle2.push_back(0);


    // circles.push_back(circle2);
    // penetrated.push_back(false);

    // std::vector<double> circle3;
    // circle3.push_back(20);
    // circle3.push_back(6+3*sqrt(2));
    // circle3.push_back(20.5+3*sqrt(2));
    // circle3.push_back(0.3);
    // circle3.push_back(0);
    // circle3.push_back(sqrt(2)/2);
    // circle3.push_back(-sqrt(2)/2);


    // circles.push_back(circle3);
    // penetrated.push_back(false);

    // std::vector<double> circle4;
    // circle4.push_back(20);
    // circle4.push_back(12);
    // circle4.push_back(20.5);
    // circle4.push_back(0.3);
    // circle4.push_back(0);
    // circle4.push_back(0);
    // circle4.push_back(-1);


    // circles.push_back(circle4);
    // penetrated.push_back(false);



    // std::vector<double> circle5;
    // circle5.push_back(20);
    // circle5.push_back(12);
    // circle5.push_back(0);
    // circle5.push_back(0.3);
    // circle5.push_back(0);
    // circle5.push_back(0);
    // circle5.push_back(-1);


    // circles.push_back(circle5);
    // penetrated.push_back(false);



}

void switchCircles(){
    // switch to new set of rings;

    circles.clear();
    penetrated.clear();
    rings.clear();

    std::vector<double> circle0;
    circle0.push_back(20);
    circle0.push_back(-2);
    circle0.push_back(18.5);
    circle0.push_back(0.8);
    circle0.push_back(0);
    circle0.push_back(1);
    circle0.push_back(0);


    circles.push_back(circle0);
    penetrated.push_back(false);

    std::vector<double> circle;
    circle.push_back(20);
    circle.push_back(0);
    circle.push_back(18.5);
    circle.push_back(0.3);
    circle.push_back(0);
    circle.push_back(1);
    circle.push_back(0);


    circles.push_back(circle);
    penetrated.push_back(false);

    std::vector<double> circle1;
    circle1.push_back(20);
    circle1.push_back(2);
    circle1.push_back(18.5);
    circle1.push_back(0.3);
    circle1.push_back(0);
    circle1.push_back(1);
    circle1.push_back(0);


    circles.push_back(circle1);
    penetrated.push_back(false);

    std::vector<double> ring0;
    ring0.push_back(20);
    ring0.push_back(-2);
    ring0.push_back(18.5);
    ring0.push_back(0.8);
    ring0.push_back(0);
    ring0.push_back(1);
    ring0.push_back(0);


    rings.push_back(ring0);

    std::vector<double> ring;
    ring.push_back(20);
    ring.push_back(0);
    ring.push_back(18.5);
    ring.push_back(0.3);
    ring.push_back(0);
    ring.push_back(1);
    ring.push_back(0);


    rings.push_back(ring);

    std::vector<double> ring1;
    ring1.push_back(20);
    ring1.push_back(2);
    ring1.push_back(18.5);
    ring1.push_back(0.3);
    ring1.push_back(0);
    ring1.push_back(1);
    ring1.push_back(0);


    rings.push_back(ring1);
}

#endif