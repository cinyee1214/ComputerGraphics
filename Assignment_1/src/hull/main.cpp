////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
  // x + y * i
	return atan2(u.imag() - v.imag(), u.real() - v.real());
}

struct Compare {
	Point p0; // Bottom&Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		return det(p1, p0) < det(p2, p0);
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
  // x + y * i
	double area = (b.real() - a.real()) * (c.imag() - a.imag()) - (b.imag() - a.imag()) * (c.real() - a.real());
	// area > 0, counterclockwise
  // area < 0, clockwise
  // area = 0, colinear
  
  return area <= 0;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// bottom most, left most point
  Point bottom = points[0];
	for(int i = 1; i < points.size(); i++){
    // x + y * i
		if(points[i].imag() < bottom.imag() || (points[i].imag() == bottom.imag() && points[i].real() < bottom.real())){
			bottom = points[i];
		}
	}
	order.p0 = bottom;
	std::sort(points.begin(), points.end(), order);

	Polygon hull;
  hull.push_back(points[0]);
  hull.push_back(points[1]);
  for (int i = 2; i < points.size(); ++i) {
    Point c = points[i];
    Point b = hull.back();
    hull.pop_back();
    while (hull.size() > 0 && salientAngle(hull.back(), b, c)) {
      b = hull.back();
      hull.pop_back();
    }
    hull.push_back(b);
    hull.push_back(c);
  }
  Point b = hull.back();
  hull.pop_back();
  if (!salientAngle(hull.back(), b, bottom)) {
    hull.push_back(b);
  }

	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	// Read each line, initialize points
  double x, y, z;
  std::string curLine;
  // Skip the first line
  std::getline(in, curLine);
  while (std::getline(in, curLine)) {
    std::istringstream streamLine(curLine);
    streamLine >> x >> y >> z;
    points.push_back(Point(x, y));
  }
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
