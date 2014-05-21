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

// local main function for testing

int main(int argc, char **argv) {
	std::cout << "here in main: " << std::endl;
	std::cout << "length of torus: " << csc.size() << std::endl;
	makeTorus();
	std::cout << "length of torus: " << csc.size() << std::endl;
}