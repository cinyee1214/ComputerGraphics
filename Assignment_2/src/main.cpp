// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

bool raytrace_sphere_intersection(Vector3d ray_origin, Vector3d ray_direction, Vector3d center, 
                                  double radius, double& a, double& b){
	double d = pow(ray_direction.dot(ray_origin - center), 2) - ray_direction.dot(ray_direction) * ((ray_origin - center).dot(ray_origin - center) - pow(radius, 2));
	if (d < 0) return false;
	a = ((-ray_direction).dot(ray_origin - center) - sqrt(d)) / (ray_direction.dot(ray_direction));
  b = ((-ray_direction).dot(ray_origin - center) + sqrt(d)) / (ray_direction.dot(ray_direction));
	return true;
}

void raytrace_sphere_perspective() {
	std::cout << "Simple ray tracer, one sphere with perspective projection" << std::endl;

	const std::string filename("sphere_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

  // Sphere 
  const Vector3d sphere_center(-0.3, 0.3, -0.5);
  // const Vector3d sphere_center(0, 0, 0);
  const double sphere_radius = 0.7;

	// Single light source
	const Vector3d light_position(-1,1,1);
  Vector3d ray_origin(1, -0.5, 1.5);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_direction = origin + double(i)*x_displacement + double(j)*y_displacement - ray_origin;			
      double a, b;
			if (raytrace_sphere_intersection(ray_origin, ray_direction, sphere_center, sphere_radius, a, b)) {
				// The ray hit the sphere, compute the exact intersection point
        Vector3d ray_intersection;
        if (a < b) {
          ray_intersection = ray_origin + a * ray_direction;
        } else {
          ray_intersection = ray_origin + b * ray_direction;
        }

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection - sphere_center).normalized();
        //if (ray_normal.dot(ray_origin-ray_intersection) < 0) ray_normal = -ray_normal;

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

// referenced by https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool raytrace_triangle(Vector3d ray_origin, Vector3d ray_direction, Vector3d origin, 
                       Vector3d edge_u, Vector3d edge_v, Vector3d& ray_intersection) {
    const double e = 0.000000001;
    Vector3d cross_res = ray_direction.cross(edge_v);
    double a = edge_u.dot(cross_res);
    // parallel
    if (a > -e && a < e) return false; 

    double u = 1.0/a * (ray_origin - origin).dot(cross_res);
    if (u < 0.0 || u > 1.0) return false;

    Vector3d cross_res2 = (ray_origin - origin).cross(edge_u);
    double v = 1.0/a * ray_direction.dot(cross_res2);
    if (v < 0.0 || u + v > 1.0) return false;

    double res = 1.0/a * edge_v.dot(cross_res2);
    if (res > e) {
      // there is a ray intersection
      ray_intersection = ray_origin + ray_direction * res;
      return true;
    } 
    return false;
}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Parameters of the parallelogram (position of the lower-left corner + two sides)
	// Vector3d pgram_origin(-0.5, -0.3, -0.5);
	// Vector3d pgram_u(1, -0.2, 0);
	// Vector3d pgram_v(0.4, 0.6, 0.3);
  // Vector3d pgram_origin(-1,-1,0);
	// Vector3d pgram_u(0, 2, 0);
	// Vector3d pgram_v(2, 0, 0);
  Vector3d pgram_origin(-0.5, 0.5, -0.6);
	Vector3d pgram_u(0.8, -0.3, 0);
	Vector3d pgram_v(0.4, 0.5, 0.3);

	// Single light source
	const Vector3d light_position(-1,1,1);
  Vector3d plane_normal = pgram_u.cross(pgram_v);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

      // Compute the exact intersection point if the ray hit the parallelogram
      Vector3d ray_intersection;
			// Check if the ray intersects with the parallelogram
      bool has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_u + pgram_v, ray_intersection);
      if (!has_intersection) has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u + pgram_v, ray_intersection);
			
      if (has_intersection) {
				// Compute normal at the intersection point
        if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = pgram_v.cross(pgram_u);
				Vector3d ray_normal = (ray_intersection + plane_normal).normalized();
				
        // Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// raytrace parallelogram perspective
void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Parameters of the parallelogram (position of the lower-left corner + two sides)
	// Vector3d pgram_origin(-0.5, -0.3, -0.5);
	// Vector3d pgram_u(1, -0.2, 0);
	// Vector3d pgram_v(0.4, 0.6, 0.3);
  // Vector3d pgram_origin(-1,-1,0);
	// Vector3d pgram_u(0, 2, 0);
	// Vector3d pgram_v(2, 0, 0);
  Vector3d pgram_origin(-0.5, 0.5, -0.6);
	Vector3d pgram_u(0.8, -0.3, 0);
	Vector3d pgram_v(0.4, 0.5, 0.3);
  // Vector3d pgram_origin(0.2, -0.1, 0.1);
	// Vector3d pgram_u(-0.5, 0, 0.2);
	// Vector3d pgram_v(-0.5, 0.5, 0);

	// Single light source
	const Vector3d light_position(-1,1,1);
  Vector3d plane_normal = pgram_u.cross(pgram_v);
  Vector3d ray_origin(0, 1, 1.5);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray (origin point and direction)
			Vector3d ray_direction = origin + double(i)*x_displacement + double(j)*y_displacement - ray_origin;

      // Compute the exact intersection point if the ray hit the parallelogram
      Vector3d ray_intersection;
      // Check if the ray intersects with the parallelogram
      bool has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_u + pgram_v, ray_intersection);
      if (!has_intersection) has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u + pgram_v, ray_intersection);
      
			if (has_intersection) {
				// Compute normal at the intersection point
        if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = pgram_v.cross(pgram_u);
        Vector3d ray_normal = (ray_intersection + plane_normal).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// sphere shading
void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("sphere_shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Add shading parameter here
        Vector3d light_normal = (light_position-ray_intersection).normalized();
				diffuse(i,j) = 0.7 * std::max(0., ray_normal.dot(light_normal));
				Vector3d h = (light_normal + ray_normal.cross(light_normal).cross(light_normal)).normalized();
				double k = std::max(0., ray_normal.dot(h));
				specular(i,j) = pow(k, 14);

				// Simple diffuse model
				C(i,j) = ambient + diffuse(i,j) + specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
  write_matrix_to_png(0.8 * C, 0.05 * C, 0.15 * C, A, filename);
}

// parallelogram shading with perspective projection
void raytrace_parallelogram_shading(){
  std::cout << "Simple ray tracer, one parallelogram with different shading" << std::endl;

	const std::string filename("parallelogram_shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

  Vector3d pgram_origin(0.2, -0.1, 0.1);
	Vector3d pgram_u(-0.5, 0, 0.2);
	Vector3d pgram_v(-0.5, 0.5, 0);

	// Single light source
	const Vector3d light_position(-1,1,1);
  Vector3d ray_origin(1, -0.5, 1.5);
  Vector3d plane_normal = pgram_u.cross(pgram_v);

  double ambient = 0.2;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);
  
  for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray (origin point and direction)
			Vector3d ray_direction = origin + double(i)*x_displacement + double(j)*y_displacement - ray_origin;

      // Compute the exact intersection point if the ray hit the parallelogram
      Vector3d ray_intersection;
      // Check if the ray intersects with the parallelogram
      bool has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_u + pgram_v, ray_intersection);
      if (!has_intersection) has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u + pgram_v, ray_intersection);
      
			if (has_intersection) {
				// Compute normal at the intersection point
        if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = pgram_v.cross(pgram_u);
        Vector3d ray_normal = (ray_intersection + plane_normal).normalized();

        // Add shading parameter here
        Vector3d light_normal = (light_position-ray_intersection).normalized();
				diffuse(i,j) = 0.6 * std::max(0., ray_normal.dot(light_normal));
				Vector3d h = (light_normal + ray_normal.cross(light_normal).cross(light_normal)).normalized();
				double k = std::max(0., ray_normal.dot(h));
				specular(i,j) = pow(k, 21);

				// Simple diffuse model
				C(i,j) = ambient + diffuse(i,j) + specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
  write_matrix_to_png(0.1 * C, 0.6 * C, 0.2 * C, A, filename);
}

int main() {
	raytrace_sphere();
    raytrace_sphere_perspective();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();
    raytrace_parallelogram_shading();

	return 0;
}
