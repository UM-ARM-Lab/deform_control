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

std::vector<std::vector<double> > findDirection (std::vector<std::vector<double> > curve) {
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
	std::vector<double> point;
	point.push_back(0);
	point.push_back(0);
	point.push_back(0);
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
		point[0] = result[0];
		point[1] = result[1];
		point[2] = result[2];
		result.clear();
		step.clear();
		count += 1;
	}
	std::cout << "after all: " << count << std::endl;
	// center[0] = result[0];
	// center[1] = result[1];
	// center[2] = result[2];
	std::vector<std::vector<double> > points;
	
	points.push_back(point);

	points.push_back(center);
	return points;
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

std::vector<std::vector<double> > completeLoop (std::vector<std::vector<double> > curve) {
	// make sure to use this function, there are at least 2 intersections between bounding box and the curve;
	


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

	// std::vector<double> circleNormal;
	// circleNormal = crossProduct(r1, r2);

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
	return loop;
}

int numIntersection (std::vector<std::vector<double> > curve) {
	bool inSide = false;
	int count = 0;
	double d = 0;
	for (int i = 0; i < curve.size(); i++) {
		d = sqrt((curve[i][0]-boundingBox[0])*(curve[i][0]-boundingBox[0]) + 
				(curve[i][1]-boundingBox[1])*(curve[i][1]-boundingBox[1]) + 
				(curve[i][2]-boundingBox[2])*(curve[i][2]-boundingBox[2]));
		if (inSide) {
			if (d > boundingBox[3]) {
				inSide = false;
				count ++;
			}
		} else {
			if (d <= boundingBox[3]) {
				inSide = true;
				count ++;
			}
		}
	}
	return count;
}

bool HopfLink (std::vector<std::vector<double> > curve1, std::vector<std::vector<double> > curve2) {

	std::vector<bool> sequence;
	double x1c = 0, y1c = 0, x2c = 0, y2c = 0, s1c = 0, s2c = 0;
	double x1n = 0, y1n = 0, x2n = 0, y2n = 0, s1n = 0, s2n = 0;
	//std::cout << "curve size: " << curve1.size() << ", " << curve2.size() << std::endl;
	for (int i = 0; i < curve1.size(); i++) {
		x1c = curve1[i][0];
		y1c = curve1[i][1];
		s1c = curve1[i][2];
		if (i == curve1.size()-1) {
			x1n = curve1[0][0];
			y1n = curve1[0][1];
			s1n = curve1[0][2];
		} else {
			x1n = curve1[i+1][0];
			y1n = curve1[i+1][1];
			s1n = curve1[i+1][2];
		}
		//std::cout << "i: " << i << std::endl;

		for (int j = 0; j < curve2.size(); j++) {
			x2c = curve2[j][0];
			y2c = curve2[j][1];
			s2c = curve2[j][2];
			if (j == curve2.size()) {
				x2n = curve2[0][0];
				y2n = curve2[0][1];
				s2n = curve2[0][2];
			} else {
				x2n = curve2[j+1][0];
				y2n = curve2[j+1][1];
				s2n = curve2[j+1][2];
			}

			if (!lineSegmentIntersect(x1c, y1c, x1n, y1n, x2c, y2c, x2n, y2n)) {
				continue;
			}
			if (s1c+s1n < s2c+s2n) {
				sequence.push_back(true);
			} else {
				sequence.push_back(false);
			}
		}

	}
	// now have the crossing sequence;
	// detect;
	if (sequence.size() % 2 != 0 || sequence.size() == 0) {
		return false;
	} 
	// do a concatination, if the length is > 0, then true, else false;
	// use erase function
	std::cout << "about to detect" << std::endl;
	bool del = true;
	while (del) {
		del = false;
		for (int i = 0; i < sequence.size(); i++) {
			if (i == sequence.size()-1) {
				break;
			}
			if (sequence[i] == sequence[i+1]) {
				sequence.erase(sequence.begin()+i);
				sequence.erase(sequence.begin()+i+1);
				del = true;
			}
		}
	}
	if (sequence.size() > 0) {
		return true;
	} 
	return false;
	
}

bool lineSegmentIntersect (double x1c, double y1c, double x1n, double y1n, double x2c, double y2c, double x2n, double y2n) {

	double a1 = 0, b1 = 0, a2 = 0, b2 = 0;
	double x, y;
	if (x1c == x1n) {
		if (x2c == x2n) {
			if (x2c == x2n) {
				if (overlap(y1c, y1n, y2c, y2n)) {
					return true;
				}
				return false;
			} else {
				return false;
			}
		} else {
			a2 = (y2c-y2n) / (x2c-x2n);
			b2 = y2c - a2 * x2c;
			x = x1c;
			y = a2 * x + b2;
			if (inRange(x, x2c, x2n) && inRange(y, y2c, y2n)) {
				return true;
			} else {
				return false;
			}
		}
	} else {
		a1 = (y1c-y1n) / (x1c-x1n);
		b1 = y1c - a1 * x1c;
		a2 = (y2c-y2n) / (x2c-x2n);
		b2 = y2c - a2 * x2c;
		if (a1 == a2) {
			if (b1 == b2) {
				return true;
			} else {
				return false;
			}
		} else {
			x = (b2-b1) / (a1-a2);
			y = a1 * x + b1;
			if (inRange(x, x1c, x1n) && inRange(y, y1c, y1n) && inRange(x, x2c, x2n) && inRange(y, y2c, y2n)) {
				return true;
			} else {
				return false;
			}
		}
	}

	return false;
}

bool inRange (double x, double a, double b) {
	double small, big;
	if (a < b) {
		small = a;
		big = b;
	} else {
		small = b;
		big = a;
	}

	if (small <= x && x <= big) {
		return true;
	}
	return false;
}

bool overlap (double a1, double b1, double a2, double b2) {
	double small, big;
	if (a1 < b1) {
		small = a1;
		big = b1;
	} else {
		small = b1;
		big = a1;
	}
	if ((small <= a2 && a2 <= big) || (small <= b2 && b2 <= big)) {
		return true;
	} else {
		if ((a2 <= small && big <= b2) || (b2 <= small && big <= a2)) {
			return true;
		}
		return false;
	}
	return false;
}

int findLastIntersection (std::vector<std::vector<double> > curve) {
	double d;
	for (int i = curve.size()-1; i >= 0; i--) {
		d = sqrt((curve[i][0]-boundingBox[0])*(curve[i][0]-boundingBox[0]) + 
				(curve[i][1]-boundingBox[1])*(curve[i][1]-boundingBox[1]) + 
				(curve[i][2]-boundingBox[2])*(curve[i][2]-boundingBox[2]));
		if (d < boundingBox[3]) {
			return i;
		}
	}
	return -1;
}

// return true if penetrate
bool findPoints (std::vector<std::vector<double> > curve, std::vector<double> center) {

	bool intersect = false;
	double x1c = 0, y1c = 0, x2c = 0, y2c = 0, s1c = 0, s2c = 0;
	double x1n = 0, y1n = 0, x2n = 0, y2n = 0, s1n = 0, s2n = 0;
	int indexLast = findLastIntersection(curve);
	int intersectIndex = -1;
	for (int i = indexLast; i > 0; i--) {
		x1c = curve[i][0];
		y1c = curve[i][1];
		s1c = curve[i][2];
		
		x1n = curve[i-1][0];
		y1n = curve[i-1][1];
		s1n = curve[i-1][2];
		
		for (int j = 0; j < csc.size(); j++) {
			x2c = csc[i][0];
			y2c = csc[i][1];
			s2c = csc[i][2];
			if (j == csc.size()) {
				x2n = csc[0][0];
				y2n = csc[0][1];
				s2n = csc[0][2];
			} else {
				x2n = csc[i+1][0];
				y2n = csc[i+1][1];
				s2n = csc[i+1][2];
			}

			if (!lineSegmentIntersect(x1c, y1c, x1n, y1n, x2c, y2c, x2n, y2n)) {
				continue;
			}
			intersect = true;
			break;
		}
		if (intersect) {
			intersectIndex = i;
			break;

		}
	}
	std::vector<double> point;
	if (intersectIndex < 0) {
		point.push_back(curve[indexLast][0]);
		point.push_back(curve[indexLast][1]);
		point.push_back(curve[indexLast][2]);
	} else {
		point.push_back(curve[intersectIndex][0]);
		point.push_back(curve[intersectIndex][1]);
		point.push_back(curve[intersectIndex][2]);
	}

	std::vector<double> tip;
	tip.push_back(curve[0][0]);
	tip.push_back(curve[0][1]);
	tip.push_back(curve[0][2]);
	// found point; now step;

	std::vector<double> direction1;
	direction1 = BiotSavart(point, csc);
	std::vector<double> direction2;
	direction2 = BiotSavart(tip, csc);

	double d1Old = 0, d1New = 0;
	double d2Old = 0, d2New = 0;
	
	d1Old = sqrt((point[0]-center[0])*(point[0]-center[0]) + 
				(point[1]-center[1])*(point[1]-center[1]) + 
				(point[2]-center[2])*(point[2]-center[2]));

	d2Old = sqrt((tip[0]-center[0])*(tip[0]-center[0]) + 
				(tip[1]-center[1])*(tip[1]-center[1]) + 
				(tip[2]-center[2])*(tip[2]-center[2]));
	if (d1Old < 1) {
		point[0] = point[0] + direction1[0] * d1Old * 0.5;
		point[1] = point[1] + direction1[1] * d1Old * 0.5;
		point[2] = point[2] + direction1[2] * d1Old * 0.5;
	} else {
		point[0] = point[0] + direction1[0];
		point[1] = point[1] + direction1[1];
		point[2] = point[2] + direction1[2];
	}

	if (d2Old < 1) {
		tip[0] = tip[0] + direction2[0] * d2Old * 0.5;
		tip[1] = tip[1] + direction2[1] * d2Old * 0.5;
		tip[2] = tip[2] + direction2[2] * d2Old * 0.5;
	} else {
		tip[0] = tip[0] + direction2[0];
		tip[1] = tip[1] + direction2[1];
		tip[2] = tip[2] + direction2[2];
	}
	
	d1New = sqrt((point[0]-center[0])*(point[0]-center[0]) + 
				(point[1]-center[1])*(point[1]-center[1]) + 
				(point[2]-center[2])*(point[2]-center[2]));
	d2New = sqrt((tip[0]-center[0])*(tip[0]-center[0]) + 
				(tip[1]-center[1])*(tip[1]-center[1]) + 
				(tip[2]-center[2])*(tip[2]-center[2]));
	if ((d1New-d1Old) * (d2New-d2Old) > 0) {
		// same side, not penetrate;
		return false;
	} else {
		// penetrate;
		return true;
	}

}

bool threaded (std::vector<std::vector<double> > curve) {
	int num = numIntersection(curve);
	std::vector<std::vector<double> > points;
	points = findDirection(csc);
	std::vector<double> normal;
	std::vector<double> center;
	normal = points[0];
	center = points[1];
	std::cout << "number: " << num << std::endl;
	if (num == 0) {
		return false;
		// question is, do we need to do something else? 
	} else if (num == 1) {
		return findPoints(curve, center);
	} else {
		std::cout << "here: " << std::endl;
		std::vector<std::vector<double> > loop;
		loop = completeLoop(curve);
		std::cout << "after completeLoop" << std::endl;
		std::vector<std::vector<double> > projectedLoop;
		std::vector<std::vector<double> > projectedcsc;
		projectedcsc = findProjection(csc, normal);
		std::cout << "after projectedcsc" << std::endl;
		projectedLoop = findProjection(loop, normal);
		std::cout << "after projectedLoop" << std::endl;
		return HopfLink(projectedLoop, projectedcsc);
	}

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

	std::vector<std::vector<double> > normalRelated;
	normalRelated = findDirection(csc);
	std::vector<double> normal;
	std::vector<double> center;
	normal = normalRelated[0];
	std::cout << "normal: " << normal[0] << ", " << normal[1] << ", " << normal[2] << std::endl;
	center = normalRelated[1];
	std::cout << "center: " << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
	std::vector<std::vector<double> > projected;
	projected = findProjection(circle, normal);
	std::vector<std::vector<double> > testCurve;

	for (int i = 0; i < 30; i++) {
		std::vector<double> point;
		point.push_back(4);
		point.push_back(25);
		point.push_back(i+15);

		testCurve.push_back(point);
	}
	// findProjection(testCurve, normal);
	std::vector<std::vector<double> > loop;
	loop = completeLoop(testCurve);

	int result = numIntersection(testCurve);
	std::cout << "num of intersections: " << result << std::endl;

	std::vector<std::vector<double> > really;
	really = findProjection(testCurve, normal);
	// for (int i = 0; i < really.size(); i++) {
	// 	std::cout << "on projected: " << really[i][0] << ", " << really[i][1] << ", " << really[i][2] << std::endl;
	// }

	// bool r = HopfLink(really, projected);
	// std::cout << "r: " << r << std::endl;
	// bool p = findPoints(testCurve, center);
	// std::cout << "p: " << p << ", " << false << std::endl;

	bool k = threaded(testCurve);
	std::cout << "k: " << k << std::endl;
}