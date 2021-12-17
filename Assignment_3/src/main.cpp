////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <gif.h>

// Eigen for matrix operations
#include <Eigen/Dense>

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

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////
bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// Compute the intersection between the ray and the sphere
	// If the ray hits the sphere, set the result of the intersection in the
	// struct 'hit'
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

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
  // Compute the exact intersection point if the ray hit the parallelogram
  Vector3d ray_origin = ray.origin;
	Vector3d ray_direction = ray.direction;
	Vector3d pgram_origin = this->origin;
  Vector3d pgram_u = this->u;
  Vector3d pgram_v = this->v;

  Vector3d ray_intersection;
  // Check if the ray intersects with the parallelogram
  bool has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_u + pgram_v, ray_intersection);
  if (!has_intersection) has_intersection = raytrace_triangle(ray_origin, ray_direction, pgram_origin, pgram_v, pgram_u + pgram_v, ray_intersection);
  if (!has_intersection) return false;

	hit.position = ray_intersection;
  Vector3d plane_normal = pgram_u.cross(pgram_v);
  if (plane_normal.dot(ray_origin-ray_intersection) < 0) plane_normal = pgram_v.cross(pgram_u);
	hit.normal = (ray_intersection + plane_normal).normalized();

	return true;
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
Vector3d refraction_direction_helper(const double &idx, const Vector3d &N, const Vector3d &R){
  double ratio = idx;
  double cos = R.dot(N); 
  Vector3d n = -N;
	if (cos < 0) {
    cos = -cos;
    ratio = 1 / idx;
    n = N;
  } 
  
	double tmp = 1 - pow(ratio, 2) * (1 - pow(cos, 2));	
	if (tmp < 0) return (Vector3d(0, 0, 0)).normalized(); 
  return (ratio * R + (ratio * cos - sqrt(tmp)) * N).normalized(); 
}

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
	// Base case
	if (max_bounce <= 0) return scene.background_color;
  
  // Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
  const double e = 0.000000001;
  //const double e = 0;
  Vector3d N = hit.normal;	

	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();	

		// Shoot a shadow ray to determine if the light should affect the intersection point
		Vector3d shadow_orgin = hit.position + Li * e;

		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

    // Specular contribution
		Vector3d specular(0, 0, 0);

		const Ray shadow_ray(shadow_orgin, Li);
    bool shadow = false;
		Intersection shadow_hit;
    for(int i = 0; i < scene.objects.size() && !shadow; ++i){
      ObjectPtr o = scene.objects[i];
			if(o->intersect(shadow_ray, shadow_hit)) shadow = true;
		}
		
    if (shadow) continue;
    Vector3d h = (Li - ray.direction.normalized()).normalized();
    specular = std::fmax(pow(h.dot(N), mat.specular_exponent), 0.0) * mat.specular_color;

    // Attenuate lights according to the squared distance to the lights
    Vector3d D = light.position - hit.position;
    lights_color += (diffuse + specular).cwiseProduct(light.intensity) / D.squaredNorm();	
	}

  double tmp = ray.direction.normalized().dot(N);
  if (tmp == 0) N = -N;

	// Compute the color of the reflected ray and add its contribution to the current point color.
  Vector3d reflection_orgin(0, 0, 0);
  reflection_orgin = hit.position + N * e;

  Vector3d reflection_direction(0, 0, 0);
  reflection_direction = (2 * N * (ray.origin - hit.position).normalized().dot(N) - (ray.origin - hit.position).normalized()).normalized();
	
  const Ray reflection_ray(reflection_orgin, reflection_direction);
  
  Vector3d reflection_color(0, 0, 0);
	reflection_color = shoot_ray(scene, reflection_ray, max_bounce - 1);

	// Compute the color of the refracted ray and add its contribution to the current point color.
	// Make sure to check for total internal reflection before shooting a new ray.	
	Vector3d refraction_orgin(0, 0, 0);
	refraction_orgin = hit.position - N * e;
  
  Vector3d refraction_direction(0, 0, 0);
  refraction_direction = refraction_direction_helper(mat.refraction_index, N, ray.direction.normalized());
	const Ray refraction_ray(refraction_orgin, refraction_direction);

  Vector3d refraction_color(0, 0, 0);
	if(refraction_direction.dot(N) < 0)	refraction_color = shoot_ray(scene, refraction_ray, max_bounce - 1);
 
	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color.cwiseProduct(mat.reflection_color) + refraction_color.cwiseProduct(mat.refraction_color);
	return C;
}
// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// Find the object in the scene that intersects the ray first
	// The function must return 'nullptr' if no object is hit, otherwise it must
	// return a pointer to the hit object, and set the parameters of the argument
	// 'hit' to their expected values.

  double distance = INFINITY;
	for(int i = 0; i < scene.objects.size(); ++i){
		if(scene.objects[i]->intersect(ray, closest_hit) && closest_hit.ray_param < distance){
			closest_index = i;
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
	// TODO: Determine if the light is visible here
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
  double scale_y = tan(atan(scene.camera.field_of_view) * 0.5) * scene.camera.focal_length; // Stretch the pixel grid by the proper amount here
	double scale_x = scale_y * aspect_ratio; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {
			// Implement depth of field
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray
			Ray ray;

			if (scene.camera.is_perspective) {
				// Perspective camera
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

	// Save to png
	const std::string filename("raytrace-with-offset.png");
	write_matrix_to_png(R, G, B, A, filename);
}

void render_scene_dof(const Scene &scene) {
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
  double scale_y = tan(atan(scene.camera.field_of_view) * 0.5) * scene.camera.focal_length; // Stretch the pixel grid by the proper amount here
	double scale_x = scale_y * aspect_ratio; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {
			// Implement depth of field
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray	
      std::vector<Ray> rays;
      int size = 10;
      rays.resize(size);	
			for (Ray ray : rays) {
				if (scene.camera.is_perspective) {
					// Perspective camera
					// Perspective camera
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
				R(i, j) += C(0);
				G(i, j) += C(1);
				B(i, j) += C(2);
			}

			R(i, j) = R(i, j) / size;
			G(i, j) = G(i, j) / size;
			B(i, j) = B(i, j) / size;
			A(i, j) = 1;
		}
	}

	// Save to png
	const std::string filename("raytrace-with-dof-10.png");
	write_matrix_to_png(R, G, B, A, filename);
}

void render_scene_gif(const Scene &scene) {
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
  double scale_y = tan(atan(scene.camera.field_of_view) * 0.5) * scene.camera.focal_length; // Stretch the pixel grid by the proper amount here
	double scale_x = scale_y * aspect_ratio; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {
			// Implement depth of field
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray	
      std::vector<Ray> rays;
      int size = 1;

      rays.resize(size);	
			for (Ray ray : rays) {
				if (scene.camera.is_perspective) {
					// Perspective camera
					// Perspective camera
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
				R(i, j) += C(0);
				G(i, j) += C(1);
				B(i, j) += C(2);
			}

			R(i, j) = R(i, j) / size;
			G(i, j) = G(i, j) / size;
			B(i, j) = B(i, j) / size;
			A(i, j) = 1;
		}
	}

	// Save to png
	const std::string filename("raytrace-2.0.png");
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
      auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
			parallelogram->u = read_vec3(entry["U"]);
      parallelogram->v = read_vec3(entry["V"]);
			object = parallelogram;
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
  // render_scene_dof(scene);
  // render_scene_gif(scene);
	return 0;
}
