#include "find_projection.h"
// #include other h files if use other shapes;


std::vector<double> find_projection() {
	std::vector<double> projection;

	projection.push_back(0);
	projection.push_back(0);
	projection.push_back(0);
	int next = 0;
	for (int i = 0; i < csc.size(); i++) {
		next = i+1;
		if (i == csc.size()-1) {
			next = 0;
		}

	}

}

std::vector<double> find_center() {
	std::vector<double> center;

}

double test_AL() {
	// use torus, and some configuration of the string;
	std::vector<std::vector<double> > circle;
	int numStep = 50;
	double step = 2*M_PI / 50;
	double radius = 3;
	for (int i = 0; i < numStep; i++) {
		std::vector<double> point;
		point.push_back(3);
		point.push_back(radius * cos(i*step));
		point.push_back(radius * sin(i*step) + 3);
		circle.push_back(point);
	}

	double start = 2.5;
	double end = -0.5;
	double numPoints = 50;
	double ropeStep = (start - end) / numPoints;
	std::vector<std::vector<double> > rope;
	for (int i = 0; i < numPoints; i++) {
		std::vector<double> point;
		point.push_back(start-i*ropeStep);
		point.push_back(0);
		point.push_back(3);

		rope.push_back(point);
	}

	// for (int i = 0; i < numPoints; i++) {
	// 	std::cout << "circle " << i << ": " << circle[i][0] << ", " << circle[i][1] << ", " << circle[i][2] << std::endl;
	// 	std::cout << "rope " << i << ": " << rope[i][0] << ", " << rope[i][1] << ", " << rope[i][2] << std::endl;
	// }



	BiotSavart(rope[0], circle);

}

std::vector<double> BiotSavart(std::vector<double> point, std::vector<std::vector<double> > curve) {

	std::vector<double> d;
	std::vector<double> p;
	std::vector<double> pp;
	std::vector<double> result;

	result.push_back(0);
	result.push_back(0);
	result.push_back(0);

	d.push_back(0);
	d.push_back(0);
	d.push_back(0);

	p.push_back(0);
	p.push_back(0);
	p.push_back(0);

	pp.push_back(0);
	pp.push_back(0);
	pp.push_back(0);

	std::vector<double> current;
	std::vector<double> next;

	std::vector<double> temp;

	temp.push_back(0);
	temp.push_back(0);
	temp.push_back(0);

	for(int i = 0; i < curve.size(); i++) {
		current = curve[i];
		if (i == curve.size()-1) {
			next = curve[0];
		} else {
			next = curve[i+1];
		}

		p[0] = current[0]-point[0];
		p[1] = current[1]-point[1];
		p[2] = current[2]-point[2];

		pp[0] = next[0]-point[0];
		pp[0] = next[1]-point[1];
		pp[0] = next[2]-point[2];


	}

}

// local main function for testing

int main(int argc, char **argv) {
	std::cout << "here in main: " << std::endl;
	std::cout << "length of torus: " << csc.size() << std::endl;
	makeTorus();
	std::cout << "length of torus: " << csc.size() << std::endl;
	test_AL();
}