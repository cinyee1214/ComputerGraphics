////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>
#include <algorithm>
#include <math.h>
#include <cfloat>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
	Vector3d position;
	Vector3d intensity;
};

struct Intersection {
	Vector3d position;
	Vector3d normal;
	double ray_param;
};

struct Camera {
	bool is_perspective;
	Vector3d position;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
};

struct Material {
	Vector3d ambient_color;
	Vector3d diffuse_color;
	Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Vector3d reflection_color;
	Vector3d refraction_color;
	double refraction_index;
};

struct Object {
	Material material;
	virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
	virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
	struct Node {
		AlignedBox3d bbox;
		int parent; // Index of the parent node (-1 for root)
		int left; // Index of the left child (-1 for a leaf)
		int right; // Index of the right child (-1 for a leaf)
		int triangle; // Index of the node triangle (-1 for internal nodes)
	};

	std::vector<Node> nodes;
	int root;

	AABBTree() = default; // Default empty constructor
	AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
  int construct(const MatrixXd &V, const MatrixXi &F, const std::vector<int> &sorted_idx, const int &start, const int &end, const int &parent);
  bool intersect_bvh(const MatrixXd &V, const MatrixXi &F, const Ray &ray, Intersection &closest_hit, int idx);
};

struct Mesh : public Object {
	MatrixXd vertices; // n x 3 matrix (n points)
	MatrixXi facets; // m x 3 matrix (m triangles)

	AABBTree bvh;

	Mesh() = default; // Default empty constructor
	Mesh(const std::string &filename);
	virtual ~Mesh() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
		assert(s == 3);
	}
}

Mesh::Mesh(const std::string &filename) {
	// Load a mesh from a file (assuming this is a .off file), and create a bvh
	load_off(filename, vertices, facets);
	bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Implementation
////////////////////////////////////////////////////////////////////////////////

// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	box.extend(c);
	return box;
}

int AABBTree::construct(const MatrixXd &V, const MatrixXi &F, const std::vector<int> &sorted_idx, const int &start, const int &end, const int &parent) {
  if (start > end) return -1;

  nodes.push_back(Node());
  int idx = nodes.size() - 1;
  nodes[idx].parent = parent;
  
  if (start == end) {
		Vector3d a = V.row(F(sorted_idx[start], 0));
		Vector3d b = V.row(F(sorted_idx[start], 1));
		Vector3d c = V.row(F(sorted_idx[start], 2));
		nodes[idx].bbox = bbox_triangle(a, b, c);
    nodes[idx].triangle = sorted_idx[start];
		nodes[idx].left = -1;
	  nodes[idx].right = -1;
    return idx;
  }
  
  nodes[idx].triangle = -1;
  int mid = start + (end - start) / 2;
  nodes[idx].left = construct(V, F, sorted_idx, start, mid, idx);
  nodes[idx].right = construct(V, F, sorted_idx, mid + 1, end, idx);

  nodes[idx].bbox.extend(nodes[nodes[idx].left].bbox);
  nodes[idx].bbox.extend(nodes[nodes[idx].right].bbox);
  return idx;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {
	// Compute the centroids of all the triangles in the input mesh
  // triangle = F
  // vertix = V
	MatrixXd centroids(F.rows(), V.cols());
	centroids.setZero();
  // std::vector<VectorXd> vec;
	for (int i = 0; i < F.rows(); ++i) {
		for (int k = 0; k < F.cols(); ++k) {
			centroids.row(i) += V.row(F(i, k));
		}
		centroids.row(i) /= F.cols();
    // vec.push_back(centroids.row(i));
	}

	// TODO (Assignment 3)

	// Method (1): Top-down approach.
	// Split each set of primitives into 2 sets of roughly equal size,
	// based on sorting the centroids along one direction or another.

  std::vector<int> sorted_idx(F.rows());
  //std::cout << F.rows() << std::endl;
	for (int i = 0; i < F.rows(); ++i) {
    sorted_idx[i] = i;
  }
	std::sort(sorted_idx.begin(), sorted_idx.end(), [&](const int &a, const int &b) {
		return centroids(a, 0) < centroids(b, 0);
	});

  // std::sort(vec.begin(), vec.end(), [](VectorXd const& a, VectorXd const& b) { 
  //   return a(0) < b(0); 
  // });
  // for (int i = 0; i < centroids.rows(); ++i) {
  //   centroids.row(i) = vec[i];
  // }

  // for (int i = 0; i < sorted_idx.size(); ++i) {
  //   std::cout << sorted_idx[i] << std::endl;
  // }
  // std::cout << centroids.size() << std::endl;
  // std::cout << sorted_idx.size() << std::endl;
  root = construct(V, F, sorted_idx, 0, sorted_idx.size() - 1, -1);
  // std::cout << nodes.size() << std::endl;
  // for (int i = 0; i < 1000; ++i) std::cout << nodes[i].parent << std::endl;
	// Method (2): Bottom-up approach.
	// Merge nodes 2 by 2, starting from the leaves of the forest, until only 1 tree is left.
}

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit);
bool intersect_box(const Ray &ray, const AlignedBox3d &box);

bool AABBTree::intersect_bvh(const MatrixXd &V, const MatrixXi &F, const Ray &ray, Intersection &closest_hit, int idx) {
  // std::cout << idx << std::endl;
  if (idx == -1) return false;

  Node node = nodes[idx];
	if (!intersect_box(ray, node.bbox)) return false;

	if (node.triangle == -1) {
		if (intersect_bvh(V, F, ray, closest_hit, node.left)) return true;
		return intersect_bvh(V, F, ray, closest_hit, node.right);
	}

  Vector3d a = V.row(F(node.triangle, 0));
  Vector3d b = V.row(F(node.triangle, 1));
  Vector3d c = V.row(F(node.triangle, 2));
  Intersection hit;
  // std::cout << "line 258" << std::endl;
  if (intersect_triangle(ray, a, b, c, hit) && hit.ray_param < closest_hit.ray_param) {
    closest_hit = hit;
    //std::cout << idx << std::endl;
    return true;
  }
  // if (intersect_triangle(ray, a, b, c, hit)) {
  //   closest_hit = hit;
  //   std::cout << "line 270" << std::endl;
  //   return true;
  // }
  return false;
}
////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO (Assignment 2)
	Vector3d ray_origin = ray.origin;
  Vector3d ray_direction = ray.direction;
  Vector3d sphere_center = this->position;
  double sphere_radius = this->radius;

	double d = pow(ray_direction.dot(ray_origin - sphere_center), 2)  - ray_direction.dot(ray_direction) * ((ray_origin - sphere_center).dot(ray_origin - sphere_center) - pow(sphere_radius, 2));
	if(d < 0) return false;

  double tmp = (-ray_direction).dot(ray_origin - sphere_center);
	double a = (tmp - sqrt(d)) / (ray_direction.dot(ray_direction));
	double b = (tmp + sqrt(d)) / (ray_direction.dot(ray_direction));

  if (a < 0 && b < 0) return false;

  hit.ray_param = std::fmin(fmax(0.0, a), fmax(0.0, b));
	hit.position = ray.origin + hit.ray_param * ray.direction;
	hit.normal = (hit.position - this->position).normalized();
	
	return true;
}

bool raytrace_triangle(Vector3d ray_origin, Vector3d ray_direction, Vector3d origin, 
                       Vector3d edge_u, Vector3d edge_v, Vector3d& ray_intersection, Intersection &hit) {
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
      hit.ray_param = res;
      return true;
    } 
    return false;
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// TODO (Assignment 2)
	Vector3d ray_origin = ray.origin;
	Vector3d ray_direction = ray.direction;
	Vector3d pgram_origin = this->origin;
  Vector3d pgram_u = this->u;
  Vector3d pgram_v = this->v;

  // Vector3d ray_intersection;
  // // Check if the ray intersects with the parallelogram
  // bool has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_u + pgram_v, ray_intersection);
  // if (!has_intersection) has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u + pgram_v, ray_intersection);
  // if (!has_intersection) return false;

	// hit.position = ray_intersection;
  // Vector3d plane_normal = pgram_u.cross(pgram_v);
  // if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = pgram_v.cross(pgram_u);
	// hit.normal = (ray_intersection + plane_normal).normalized();

	return true;
}

// -----------------------------------------------------------------------------

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit) {
	// TODO (Assignment 3)
	// Compute whether the ray intersects the given triangle.
	// If you have done the parallelogram case, this should be very similar to it.
	Vector3d ray_origin = ray.origin;
	Vector3d ray_direction = ray.direction;
  Vector3d triangle_origin = a;

  Vector3d ray_intersection;
  // Check if the ray intersects with the parallelogram
  bool has_intersection = raytrace_triangle(ray_origin, ray_direction, triangle_origin, b - a, c - a, ray_intersection, hit);
  if (!has_intersection) return false;

	hit.position = ray_intersection;
  Vector3d plane_normal = a.cross(-c);
  if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = -c.cross(a);
	hit.normal = (ray_intersection + plane_normal).normalized();
	return true;
}

bool intersect_box(const Ray &ray, const AlignedBox3d &box) {
	// TODO (Assignment 3)
	// Compute whether the ray intersects the given box.
	// There is no need to set the resulting normal and ray parameter, since
	// we are not testing with the real surface here anyway.
  Vector3d ray_origin = ray.origin;
	Vector3d ray_direction = ray.direction;
	Vector3d box_max = box.max();
  Vector3d box_min = box.min();

  for (int i = 0; i < 3; i++) {
		if (ray_direction(i) == 0
        && (ray_origin[i] > box.max()[i] || ray_origin[i] < box.min()[i])) 
			return false;
	}

  double x1 = (box_max[0] - ray_origin[0]) / (ray_direction[0]);
	double x2 = (box_min[0] - ray_origin[0]) / (ray_direction[0]);
  double x_min = std::min(x1, x2);
  double x_max = std::max(x1, x2);
	double y1 = (box_max[1] - ray_origin[1]) / (ray_direction[1]);
	double y2 = (box_min[1] - ray_origin[1]) / (ray_direction[1]);
  double y_min = std::min(y1, y2);
  double y_max = std::max(y1, y2);
	double z1 = (box_max[2] - ray_origin[2]) / (ray_direction[2]);
	double z2 = (box_min[2] - ray_origin[2]) / (ray_direction[2]);
  double z_min = std::min(z1, z2);
  double z_max = std::max(z1, z2);
	
	double min = std::max(x_min, std::min(y_min, z_min));
	double max = std::min(x_max, std::max(y_max, z_max));
	return max >= min;
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
	// TODO (Assignment 3)

	// Method (1): Traverse every triangle and return the closest hit.

	// Method (2): Traverse the BVH tree and test the intersection with a
	// triangles at the leaf nodes that intersects the input ray.

  // std::cout << "line 399" << std::endl;
  closest_hit.ray_param = INFINITY;
  //std::cout << bvh.root << std::endl;
	return bvh.intersect_bvh(vertices, facets, ray, closest_hit, bvh.root);
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
  // Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
  Vector3d N = hit.normal;
  const double e = 0.000000001;
	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();

		// TODO (Assignment 2, shadow rays)

		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO (Assignment 2, specular contribution)
		Vector3d specular(0, 0, 0);
    Vector3d h = (Li + (ray.origin - hit.position).normalized()).normalized();
    specular = std::fmax(pow(h.dot(N), mat.specular_exponent), 0.0) * mat.specular_color;

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// TODO (Assignment 2, reflected ray)
	Vector3d reflection_color(0, 0, 0);

	// TODO (Assignment 2, refracted ray)
	Vector3d refraction_color(0, 0, 0);

	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// TODO (Assignment 2, find nearest hit)

	double tmp_param = INFINITY;
	for(int i = 0; i < scene.objects.size(); ++i){
		if(scene.objects[i]->intersect(ray, closest_hit) && closest_hit.ray_param < tmp_param){
			closest_index = i;
      tmp_param = closest_hit.ray_param;
		}
	}

	if (closest_index < 0) {
		// Return a NULL pointer
		return nullptr;
	} else {
		// Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
		return scene.objects[closest_index].get();
	}
}

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light) {
	// TODO (Assignment 2, shadow ray)
  Intersection shadow_hit;
  for (int i = 0; i < scene.objects.size(); ++i) {
    const ObjectPtr& o = scene.objects[i];
		if(o->intersect(ray, shadow_hit)) {
			if (shadow_hit.ray_param >= 0 
        && shadow_hit.ray_param <= ((light.position[0] - ray.origin[0]) / ray.direction[0])) return false;
		}
	}
	return true;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
	std::cout << "Simple ray tracer." << std::endl;

	int w = 640;
	int h = 480;
	MatrixXd R = MatrixXd::Zero(w, h);
	MatrixXd G = MatrixXd::Zero(w, h);
	MatrixXd B = MatrixXd::Zero(w, h);
	MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

	// The camera always points in the direction -z
	// The sensor grid is at a distance 'focal_length' from the camera center,
	// and covers an viewing angle given by 'field_of_view'.
	double aspect_ratio = double(w) / double(h);
	double scale_y = tan(atan(scene.camera.field_of_view) * 0.5) * scene.camera.focal_length; // TODO: Stretch the pixel grid by the proper amount here
	double scale_x = 1.0; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
		for (unsigned j = 0; j < h; ++j) {
			// TODO (Assignment 2, depth of field)
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray
			Ray ray;

			if (scene.camera.is_perspective) {
				// Perspective camera
				// TODO (Assignment 2, perspective camera)
        double random_x = scene.camera.lens_radius / w * scale_x * rand() / RAND_MAX;
        double random_y = -scene.camera.lens_radius / h * scale_y * rand() / RAND_MAX;
        ray.origin = scene.camera.position + Vector3d(random_x, random_y, 0);
        ray.direction = (scene.camera.position + shift - ray.origin).normalized();
			} else {
				// Orthographic camera
				ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
				ray.direction = Vector3d(0, 0, -1);
			}

			int max_bounce = 5;
			Vector3d C = shoot_ray(scene, ray, max_bounce);
			R(i, j) = C(0);
			G(i, j) = C(1);
			B(i, j) = C(2);
			A(i, j) = 1;
		}
	}

	std::cout << "Ray tracing: 100%  " << std::endl;

	// Save to png
	const std::string filename("bunny.png");
	write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
	Scene scene;

	// Load json data from scene file
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3 = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	// Read scene info
	scene.background_color = read_vec3(data["Scene"]["Background"]);
	scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

	// Read camera info
	scene.camera.is_perspective = data["Camera"]["IsPerspective"];
	scene.camera.position = read_vec3(data["Camera"]["Position"]);
	scene.camera.field_of_view = data["Camera"]["FieldOfView"];
	scene.camera.focal_length = data["Camera"]["FocalLength"];
	scene.camera.lens_radius = data["Camera"]["LensRadius"];

	// Read materials
	for (const auto &entry : data["Materials"]) {
		Material mat;
		mat.ambient_color = read_vec3(entry["Ambient"]);
		mat.diffuse_color = read_vec3(entry["Diffuse"]);
		mat.specular_color = read_vec3(entry["Specular"]);
		mat.reflection_color = read_vec3(entry["Mirror"]);
		mat.refraction_color = read_vec3(entry["Refraction"]);
		mat.refraction_index = entry["RefractionIndex"];
		mat.specular_exponent = entry["Shininess"];
		scene.materials.push_back(mat);
	}

	// Read lights
	for (const auto &entry : data["Lights"]) {
		Light light;
		light.position = read_vec3(entry["Position"]);
		light.intensity = read_vec3(entry["Color"]);
		scene.lights.push_back(light);
	}

	// Read objects
	for (const auto &entry : data["Objects"]) {
		ObjectPtr object;
		if (entry["Type"] == "Sphere") {
			auto sphere = std::make_shared<Sphere>();
			sphere->position = read_vec3(entry["Position"]);
			sphere->radius = entry["Radius"];
			object = sphere;
		} else if (entry["Type"] == "Parallelogram") {
			// TODO
      auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
			parallelogram->u = read_vec3(entry["U"]);
      parallelogram->v = read_vec3(entry["V"]);
			object = parallelogram;
		} else if (entry["Type"] == "Mesh") {
			// Load mesh from a file
			std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
			object = std::make_shared<Mesh>(filename);
		}
		object->material = scene.materials[entry["Material"]];
		scene.objects.push_back(object);
	}

	return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	render_scene(scene);
	return 0;
}
