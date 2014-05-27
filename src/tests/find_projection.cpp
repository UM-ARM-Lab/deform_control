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


	std::vector<double> value;
	value = BiotSavart(rope[0], circle);
	std::cout << "back from BS: " << std::endl;

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
	std::vector<double> sij;

	sij.push_back(0);
	sij.push_back(0);
	sij.push_back(0);

	double normij = 0;
	double normd = 0;
	double normp = 0;
	double normpp = 0;

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
		pp[1] = next[1]-point[1];
		pp[2] = next[2]-point[2];

		sij[0] = next[0]-current[0];
		sij[1] = next[1]-current[1];
		sij[2] = next[2]-current[2];

		normij = sij[0]*sij[0] + sij[1]*sij[1] + sij[2]*sij[2];
		std::vector<double> temp;
		temp = crossProduct(sij, crossProduct(p, pp));
		d[0] = temp[0] / normij;
		d[1] = temp[1] / normij;
		d[2] = temp[2] / normij;
		temp.clear();
		normd = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
		normpp = sqrt(pp[0]*pp[0]+pp[1]*pp[1]+pp[2]*pp[2]);
		normp = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
		temp = crossProduct(d, pp);
		temp[0] = temp[0] / normpp;
		temp[1] = temp[1] / normpp;
		temp[2] = temp[2] / normpp;
		std::vector<double> temp1;
		temp1 = crossProduct(d, p);

		temp1[0] = temp1[0] / normp;
		temp1[1] = temp1[1] / normp;
		temp1[2] = temp1[2] / normp;
		result[0] += (temp[0]-temp1[0]) / normd;
		result[1] += (temp[1]-temp1[1]) / normd;
		result[2] += (temp[2]-temp1[2]) / normd;
		temp.clear();
		temp1.clear();
	}

	return result;

}

std::vector<double> crossProduct (std::vector<double> u, std::vector<double> v) {
	std::vector<double> result;
	result.push_back(u[1]*v[2]-u[2]*v[1]);
	result.push_back(u[2]*v[0]-u[0]*v[2]);
	result.push_back(u[0]*v[1]-u[1]*v[0]);
	
	return result;
}

std::vector<double> findDirection (std::vector<std::vector<double> > curve) {
	std::vector<double> center;
	center.reserve(3);
	center.push_back(0);
	center.push_back(0);
	center.push_back(0);

	for (int i = 0; i < curve.size(); i++) {
		center[0] += curve[i][0];
		center[1] += curve[i][1];
		center[2] += curve[i][2];
	}
	center[0] = center[0] / curve.size();
	center[1] = center[1] / curve.size();
	center[2] = center[2] / curve.size();

	// noise
	// center[0] = center[0] + (rand()/(RAND_MAX+1.0)) * 3 - 1.5;
	// center[1] = center[1] + (rand()/(RAND_MAX+1.0)) * 3 - 1.5;
	// center[2] = center[2] + (rand()/(RAND_MAX+1.0)) * 3 - 1.5;

	// noise


	double difference = 1000;
	std::vector<double> result;
	double normValue = 0;
	std::vector<double> step;
	std::vector<double> next;
	next.push_back(0);
	next.push_back(0);
	next.push_back(0);
	double normStep = 0;
	int count = 0;
	while (difference > 0.0001) {
		result = BiotSavart(center, curve);

		normValue = sqrt(result[0]*result[0]+result[1]*result[1]+result[2]*result[2]);
		result[0] = result[0] / normValue;
		result[1] = result[1] / normValue;
		result[2] = result[2] / normValue;

		next[0] = center[0] + result[0];
		next[1] = center[1] + result[1];
		next[2] = center[2] + result[2];

		step = BiotSavart(next, curve);
		normStep = sqrt(step[0]*step[0]+step[1]*step[1]+step[2]*step[2]);
		step[0] = step[0] / normStep;
		step[1] = step[1] / normStep;
		step[2] = step[2] / normStep;

		difference = sqrt((result[0]-step[0])*(result[0]-step[0]) + 
							(result[1]-step[1])*(result[1]-step[1]) + 
							(result[2]-step[2])*(result[2]-step[2]));
		center[0] = next[0] - step[0];
		center[1] = next[1] - step[1];
		center[2] = next[2] - step[2];
		result.clear();
		step.clear();
		count += 1;
	}
	std::cout << "after all: " << count << std::endl;
	std::cout << "result: " << result[0] << ", " << result[1] << ", " << result[2] << std::endl;
	center[0] = result[0];
	center[1] = result[1];
	center[2] = result[2];
	return center;
}

std::vector<std::vector<double> > findProjection (std::vector<std::vector<double> > curve, 
	std::vector<double> normal) {
	std::vector<std::vector<double> > transformed;
	for (int i = 0; i < curve.size(); i++) {
		std::vector<double> point;
		point.push_back(0);
		point.push_back(0);
		point.push_back(0);
		transformed.push_back(point);
	}


	

	// next step: transform to 2D transformation.
	double a = normal[0];
	double b = normal[1];
	double c = normal[2];
	double xo = curve[0][0];
	double yo = curve[0][1];
	double zo = curve[0][2];
	// double d = - a*xo - b*yo - c*zo;
	double d = 0;
	double x = 0, y = 0, z = 0;
	// plane formula: ax + by + cz + d = 0;

	double costheta = c / sqrt(a*a+b*b+c*c);
	std::vector<double> standard;
	standard.push_back(0);
	standard.push_back(0);
	standard.push_back(1);
	std::vector<double> axis;
	axis = crossProduct(normal, standard);
	double normAxis = sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
	axis[0] = axis[0]/normAxis;
	axis[1] = axis[1]/normAxis;
	axis[2] = axis[2]/normAxis;

	double s = sqrt(1-costheta*costheta);
	double C = 1-costheta;
	double t11, t12, t13, t21, t22, t23, t31, t32, t33;
	if (fabs(normAxis) > 0.001) {
		t11 = axis[0]*axis[0]*C + costheta;
		t12 = axis[0]*axis[1]*C - axis[2]*s;
		t13 = axis[0]*axis[2]*C + axis[1]*s;

		t21 = axis[1]*axis[0]*C + axis[2]*s;
		t22 = axis[1]*axis[1]*C + costheta;
		t23 = axis[1]*axis[2]*C - axis[0]*s;

		t31 = axis[2]*axis[0]*C - axis[1]*s;
		t32 = axis[2]*axis[1]*C + axis[0]*s;
		t33 = axis[2]*axis[2]*C + costheta;
	} else {
		t11 = 1;
		t12 = 0;
		t13 = 0;

		t21 = 0;
		t22 = 1;
		t23 = 0;

		t31 = 0;
		t32 = 0;
		t33 = 1;
	}

	for (int i = 0; i < curve.size(); i++) {
		x = t11 * curve[i][0] + t12 * curve[i][1] + t13 * curve[i][2];
		y = t21 * curve[i][0] + t22 * curve[i][1] + t23 * curve[i][2];
		z = t31 * curve[i][0] + t32 * curve[i][1] + t33 * curve[i][2];

		// the third element should indicate how far it need to project to 
		// get on the plane

		// std::cout << "coordinates: " << x << ", " << y << ", " << z << std::endl;
		transformed[i][0] = x;
		transformed[i][1] = y;
		transformed[i][2] = (a*curve[i][0] + b*curve[i][1] + c*curve[i][2] + d) / 
			(a*a + b*b + c*c);
		// std::cout << "coordinates: " << x << ", " << y << ", " << z << ", " 
		// 	<< transformed[i][2] << std::endl;
	}

	// end finding 2D coordinates;


	return transformed;

}

void completeLoop (std::vector<std::vector<double> > curve) {
	// make sure to use this function, there are at least 2 intersections between bounding box and the curve;
	std::cout << "in completeLoop!" << std::endl;
	std::cout << "boundingBox: " << boundingBox[0] << ", " << boundingBox[1] << ", " 
		<< boundingBox[2] << ", " << boundingBox[3] << std::endl;


	// At current stage, assume all bounding box are spheres;
	std::vector<double> first;
	std::vector<double> last;
	int length = curve.size();
	double d = 1000;
	int indexFirst = -1, indexLast = -1;
	for (int i = 0; i < length; i++) {
		d = sqrt((curve[i][0]-boundingBox[0])*(curve[i][0]-boundingBox[0]) + 
				(curve[i][1]-boundingBox[1])*(curve[i][1]-boundingBox[1]) + 
				(curve[i][2]-boundingBox[2])*(curve[i][2]-boundingBox[2]));

		if (d < boundingBox[3]) {
			first.push_back(curve[i][0]);
			first.push_back(curve[i][1]);
			first.push_back(curve[i][2]);
			indexFirst = i;
			break;

		}
	}

	for (int i = length-1; i >= 0; i--) {
		d = sqrt((curve[i][0]-boundingBox[0])*(curve[i][0]-boundingBox[0]) + 
				(curve[i][1]-boundingBox[1])*(curve[i][1]-boundingBox[1]) + 
				(curve[i][2]-boundingBox[2])*(curve[i][2]-boundingBox[2]));
		if (d < boundingBox[3]) {
			last.push_back(curve[i][0]);
			last.push_back(curve[i][1]);
			last.push_back(curve[i][2]);
			indexLast = i;
			break;
		}
	}
	double temp;
	if (indexLast < indexFirst) {
		temp = indexLast;
		indexLast = indexFirst;
		indexFirst = temp;
	} 

	std::vector<std::vector<double> > loop;
	// parts that are inside;
	for (int i = indexFirst; i <= indexLast; i++) {
		std::vector<double> point;
		point.push_back(curve[i][0]);
		point.push_back(curve[i][1]);
		point.push_back(curve[i][2]);

		loop.push_back(point);
	}
	// parts on the bounding box;

	std::vector<double> r1;
	std::vector<double> r2;

	r1.push_back(curve[indexFirst][0]-boundingBox[0]);
	r1.push_back(curve[indexFirst][1]-boundingBox[1]);
	r1.push_back(curve[indexFirst][2]-boundingBox[2]);

	r2.push_back(curve[indexLast][0]-boundingBox[0]);
	r2.push_back(curve[indexLast][1]-boundingBox[1]);
	r2.push_back(curve[indexLast][2]-boundingBox[2]);
	std::cout << "r1: " << r1[0] << ", " << r1[1] << ", " << r1[2] << std::endl;
	std::cout << "r2: " << r2[0] << ", " << r2[1] << ", " << r2[2] << std::endl;
	std::vector<double> circleNormal;
	circleNormal = crossProduct(r1, r2);

	// std::cout << "circleNormal: " << circleNormal[0] << ", " << circleNormal[1] << ",  " << circleNormal[2] << std::endl;

	// if (fabs(circleNormal[0]) < 0.001 && fabs(circleNormal[1]) < 0.001 && fabs(circleNormal[2]) < 0.001) {
	// 	// colinear
	// 	// choose any line;
	// 	std::cout << "colinear: " << std::endl;
	// 	double step = M_PI / length;
	// 	double angle1 = atan2(r1[2], r1[1]);
	// 	double angle2 = atan2(r2[2], r2[1]);
	// 	if (angle1 > angle2) {
	// 		double temp = angle2;
	// 		angle2 = angle1;
	// 		angle1 = temp;
	// 	}
	// 	for (int i = 0; i < length; i++) {
	// 		std::vector<double> point;
	// 		point.push_back(boundingBox[0]);
	// 		point.push_back(boundingBox[1] + boundingBox[3]*cos(angle1+i*step));
	// 		point.push_back(boundingBox[2] + boundingBox[3]*sin(angle1+i*step));
	// 		std::cout << "point: " << point[0] << ", " << point[1] << ", " << point[2] << std::endl;
	// 		loop.push_back(point);
	// 	}

	// } else {
		// circleNormal is the normal for the plane for the geodesic;
		

		double delta1 = asin((r1[2])/boundingBox[3]);
		double delta2 = asin((r2[2])/boundingBox[3]);
		double lambda1 = atan2(r1[1], r1[0]);
		double lambda2 = atan2(r2[1], r2[0]);

		double stepD = (delta2-delta1) / length;
		double stepL = (lambda2-lambda1) / length;
		for (int i = 0; i < length; i++) {
			std::vector<double> point;
			point.push_back(boundingBox[0] + boundingBox[3] * cos(lambda1 + i*stepL) * cos(delta1 + i*stepD));
			point.push_back(boundingBox[1] + boundingBox[3] * sin(lambda1 + i*stepL) * cos(delta1 + i*stepD));
			point.push_back(boundingBox[2] + boundingBox[3] * sin(delta1 + i*stepD));



			loop.push_back(point);
		}

	// }

}

// local main function for testing

int main(int argc, char **argv) {
	srand(time(NULL));
	std::cout << "here in main: " << std::endl;
	std::cout << "length of torus: " << csc.size() << std::endl;
	makeTorus();
	makeBoundingBox();
	std::cout << "length of torus: " << csc.size() << std::endl;
	//test_AL();
	std::vector<std::vector<double> > circle;
	int numStep = 50;
	double step = 2*M_PI / 50;
	double radius = 3;
	for (int i = 0; i < numStep; i++) {
		std::vector<double> point;
		point.push_back(3 + radius * sin(i*step) * cos(M_PI/6));
		point.push_back(radius * cos(i*step));
		point.push_back(radius * sin(i*step) * sin(M_PI/6) + 6);
		// point.push_back(3+radius*cos(i*step));
		// point.push_back(radius*sin(i*step));
		// point.push_back(3);
		circle.push_back(point);
	}

	std::vector<double> normal;

	normal = findDirection(circle);
	std::vector<std::vector<double> > projected;
	projected = findProjection(circle, normal);
	std::vector<std::vector<double> > testCurve;

	for (int i = 0; i < 30; i++) {
		std::vector<double> point;
		point.push_back(5);
		point.push_back(25);
		point.push_back(i+15);

		testCurve.push_back(point);
	}
	// findProjection(testCurve, normal);

	completeLoop(testCurve);
}