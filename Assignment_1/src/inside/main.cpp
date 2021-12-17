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
  if ((u.real() - v.real()) == 0) return 0;
	return (u.imag() - v.imag()) / (u.real() - v.real());
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
  // check whether [a,b] and [c,d] is colinear
  if (det(a, b) == det(c, d)) return false;

  double x1 = a.real(), x2 = b.real(), x3 = c.real(), x4 = d.real();
  double y1 = a.imag(), y2 = b.imag(), y3 = c.imag(), y4 = d.imag();
  double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
	double u = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
  if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return false;

  double x = x1 + t * (x2 - x1), y = y1 + t *(y2 - y1);
  ans = Point(x, y);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
  // x + y * i
  double min_x = poly[0].real(), min_y = poly[0].imag();
  for(int i = 0; i < poly.size(); i++){
    double x = poly[i].real(), y = poly[i].imag();
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
	}

	Point outside(min_x - 1.0, min_y - 1.0);
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	int cnt = 0, flag = 0;
  Point intersect;

  for (int i = 0; i < poly.size(); ++i) {
    int j = (i == poly.size() - 1) ? 0 : i + 1;
    if (intersect_segment(query, outside, poly[i], poly[j], intersect)) {
      cnt++;
      if ((intersect.real() == poly[i].real() && intersect.imag() == poly[i].imag()) ||
          (intersect.real() == poly[j].real() && intersect.imag() == poly[j].imag())) {
            flag = 1;
          }
    }
  }
	return (cnt - flag) % 2 == 1;
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

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
  Polygon poly;
  char ch;
  double x, y, z;
  std::string curLine;
  while (std::getline(in, curLine)) {
    std::istringstream streamLine(curLine);
    streamLine >> ch >> x >> y >> z;
    if (ch != 'v') break;
    poly.push_back(Point(x, y));
  }
	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
  out << points.size() << std::endl;
	for (const auto &v : points) {
		out << v.real() << " " << v.imag() << " 0" << std::endl;
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
