#include "SDLViewer.h"

#include <Eigen/Core>

#include <functional>
#include <iostream>
#include <float.h>
#include <math.h>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

# define FRAME_COUNT 20

Eigen::Vector4f origin_position(int x, int y, int width, int height) {
  Eigen::Vector4f res((2 * float(x) / float(width)) - 1,
                      (2 * float(height - 1 - y) / float(height)) - 1, 
                      0, 
                      1);
  return res;
}

Eigen::Vector4f get_vector(int x, int y, int width, int height) {
  Eigen::Vector4f res(2 * float(x) / float(width), 
                      -2 * float(y) / float(height), 
                      0, 
                      0);
  return res;
}

bool inside_triangle(const Eigen::Vector4f &pos, const Eigen::Vector4f &a, const Eigen::Vector4f &b, const Eigen::Vector4f &c) {
  Eigen::Vector3f p(pos[0], pos[1], 1);
  Eigen::Matrix3f M;
  M << a[0], b[0], c[0],
       a[1], b[1], c[1],
       1, 1, 1;

  Eigen::Vector3f r = M.inverse() * p;
  return r[0] >= 0 && r[1] >= 0 && r[2] >= 0;
}

int get_triangle(const UniformAttributes &uniform, const std::vector<VertexAttributes> &vertices, const Eigen::Vector4f &pos) {
  for (int i = 0; i + 2 < vertices.size(); i += 3) {
    Eigen::Vector4f a = uniform.transformation * vertices[i].model * vertices[i].position;
    Eigen::Vector4f b = uniform.transformation * vertices[i + 1].model * vertices[i + 1].position;
    Eigen::Vector4f c = uniform.transformation * vertices[i + 2].model * vertices[i + 2].position;

    if (inside_triangle(pos, a, b, c)) {
      return i / 3;
    }
  }

  return -1;
}

int get_vertex(const UniformAttributes &uniform, const std::vector<VertexAttributes> &vertices, const Eigen::Vector4f &pos) {
  int index = -1;
  float min_distance = FLT_MAX;

  for (int i = 0; i < vertices.size(); ++i) {
    float distance = (uniform.transformation * vertices[i].model * vertices[i].position - pos).squaredNorm();
    if (distance < min_distance) {
      min_distance = distance;
      index = i;
    }
  }

  return index;
}

void remove_triangle(std::vector<VertexAttributes> &vertices, int index) {
  vertices.erase(vertices.begin() + index, vertices.begin() + index + 3);
}

void rotate_triangle(std::vector<VertexAttributes> &vertices, int index, float degree) {
  degree = M_PI * (degree / 180);
  Eigen::Vector4f barycenter = (vertices[index].position 
                        + vertices[index + 1].position
                        + vertices[index + 2].position) / 3;
  Eigen::Matrix4f R;
  float a = std::cos(degree), b = std::sin(degree);
  R << a,-b,0,-barycenter[0]*a+barycenter[1]*b+barycenter[0],
       b,a,0,-barycenter[0]*b-barycenter[1]*a+barycenter[1],
       0,0,1,0,
       0,0,0,1;

  for (int i = 0; i < 3; ++i) {
    vertices[index + i].model = vertices[index + i].model * R;
  }
}

void scale_triangle(std::vector<VertexAttributes> &vertices, int index, float percentage) {
  Eigen::Vector4f barycenter = (vertices[index].position 
                                + vertices[index + 1].position
                                + vertices[index + 2].position) / 3;
  Eigen::Matrix4f S;

  S << (1+percentage),0,0,-barycenter[0]*percentage,
      0,(1+percentage),0,-barycenter[1]*percentage,
      0,0,1,0,
      0,0,0,1;

  for (int i = 0; i < 3; ++i) {
    vertices[index + i].model = vertices[index + i].model * S;
  }
}

void add_keyframes(std::vector<VertexAttributes> &keyframes, std::vector<VertexAttributes> &vertices, int index) {
  for (int i = 0; i < 3; ++i) {
    keyframes.push_back(vertices[index + i]);
  }
}

void update_line_vertices(const UniformAttributes &uniform, std::vector<VertexAttributes> &line_vertices, std::vector<VertexAttributes> &vertices, VertexAttributes curr_vertex, bool clicked) {
  switch (line_vertices.size()) {
    case 0:
    case 1: {
      line_vertices.push_back(curr_vertex);
      break;
    }
    case 2: {
      line_vertices.push_back(line_vertices[1]);
      line_vertices.push_back(curr_vertex);
      line_vertices.push_back(curr_vertex);
      line_vertices.push_back(line_vertices[0]);
    }
    case 6: {
      if (clicked) {
        for (int i = 0; i < 3; ++i) {
          line_vertices[i * 2].color = Eigen::Vector4f(1,0,0,1);
          vertices.push_back(line_vertices[i * 2]);
        }
        line_vertices.clear();
      }
      break;
    }
    default: {
      break;
    }
  }
}

int main(int argc, char *args[]) {
  int width = 500;
  int height = 500;
  // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
    return VertexAttributes(uniform.transformation * va.model * va.position, va.color);
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// One triangle in the center of the screen
	// std::vector<VertexAttributes> vertices;
	// vertices.push_back(VertexAttributes(-1,-1,0));
	// vertices.push_back(VertexAttributes(1,-1,0));
	// vertices.push_back(VertexAttributes(0,1,0));
  // vertices[0].color << 1,0,0,1;
  // vertices[1].color << 0,1,0,1;
  // vertices[2].color << 0,0,1,1;

  // Interactive triangles
  std::vector<VertexAttributes> vertices; // triangle vertices
  std::vector<VertexAttributes> line_vertices; // line vertices
  VertexAttributes curr_vertex;
  int vertex_selected = -1; // index
  int triangle_selected = -1;
  bool pressed = false;
  bool clicked = false;

  // Ex.6: Keyframing (Linear Interpolation)
  bool keyframing = false;
  int curr_keyframe = -1;
  int frame_cnt = 0;
  std::vector<VertexAttributes> keyframes;
  std::vector<Eigen::Matrix4f> tmp(3);
  
  // Initialize the viewer and the corresponding callbacks
  SDLViewer viewer;
  viewer.init("Viewer Example", width, height);
  Mode mode;

  viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
    if (keyframing) return;

    switch (mode.curr) {
      case Mode::INSERTION: {
        curr_vertex.position = uniform.transformation.inverse() * origin_position(x, y, width, height);
        viewer.redraw_next = true;
        break;
      }
      case Mode::TRANSLATION: {
        if (!pressed || triangle_selected == -1 || triangle_selected * 3 + 2 >= vertices.size()) break;

        Eigen::Vector4f vec = get_vector(xrel, yrel, width, height);
        for (int i = 0; i < 3; ++i) {
          vertices[triangle_selected * 3 + i].transform(vec, uniform.transformation);
        }
        viewer.redraw_next = true;
        break;
      }
      default: {
        break;
      }
    }
  };

  viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
    if (keyframing) return;

    std::cout << "mouse_pressed: " << is_pressed << "; " << x << "," << y << " " << std::endl;
    pressed = is_pressed;

    switch (mode.curr) {
      case Mode::INSERTION: {
        if (!is_pressed) return;

        curr_vertex.position = uniform.transformation.inverse() * origin_position(x, y, width, height);
        viewer.redraw_next = true;
        clicked = true;
        break;
      }
      case Mode::TRANSLATION: {
        if (is_pressed) {
          Eigen::Vector4f pos = origin_position(x, y, width, height);
          triangle_selected = get_triangle(uniform, vertices, pos);

          if (triangle_selected != -1) {
            Eigen::Vector4f tmp_color(0,0,1,1);
            vertices[triangle_selected * 3].change_color(tmp_color);
            vertices[triangle_selected * 3 + 1].change_color(tmp_color);
            vertices[triangle_selected * 3 + 2].change_color(tmp_color);
          }
        } else {
          if (triangle_selected != -1 && triangle_selected * 3 + 2 < vertices.size()) {
            vertices[triangle_selected * 3].reset_color();
            vertices[triangle_selected * 3 + 1].reset_color();
            vertices[triangle_selected * 3 + 2].reset_color();
          }
        }
        viewer.redraw_next = true;
        break;
      }
      case Mode::DELETION: {
        if (!is_pressed) return;

        Eigen::Vector4f pos = origin_position(x, y, width, height);
        triangle_selected = get_triangle(uniform, vertices, pos);
        if (triangle_selected != -1) {
          remove_triangle(vertices, triangle_selected * 3);
          viewer.redraw_next = true;
        }
        break;
      }
      case Mode::COLOR: {
        if (!is_pressed) return;

        Eigen::Vector4f pos = origin_position(x, y, width, height);
        vertex_selected = get_vertex(uniform, vertices, pos);
        triangle_selected = vertex_selected / 3;
        viewer.redraw_next = true;
        break;
      }
      default:
        break;
    }
  };

  viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
  };

  viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
    std::cout << "key_pressed: " << key << std::endl;
    if (!is_pressed || keyframing) return;
    mode.prev = mode.curr;

    switch (key) {
      case 'i': {
        mode.curr = Mode::INSERTION;
        break;
      }
      case 'o': {
        mode.curr = Mode::TRANSLATION;
        break;
      }
      case 'p': {
        mode.curr = Mode::DELETION;
        break;
      }
      case 'h': {
        if (mode.curr == Mode::TRANSLATION && triangle_selected != -1 && triangle_selected * 3 + 2 < vertices.size()) {
          rotate_triangle(vertices, triangle_selected * 3, 10);
        }
        break;
      }
      case 'j': {
        if (mode.curr == Mode::TRANSLATION && triangle_selected != -1 && triangle_selected * 3 + 2 < vertices.size()) {
          rotate_triangle(vertices, triangle_selected * 3, -10);
        }
        break;
      }
      case 'k': {
        if (mode.curr == Mode::TRANSLATION && triangle_selected != -1 && triangle_selected * 3 + 2 < vertices.size()) {
          scale_triangle(vertices, triangle_selected * 3, 0.25);
        }
        break;
      }
      case 'l': {
        if (mode.curr == Mode::TRANSLATION && triangle_selected != -1 && triangle_selected * 3 + 2 < vertices.size()) {
          scale_triangle(vertices, triangle_selected * 3, -0.25);
        }
        break;
      }
      case 'c': {
        mode.curr = Mode::COLOR;
        break;
      }
      case '1': 
      case '2': 
      case '3': 
      case '4': 
      case '5': 
      case '6': 
      case '7': 
      case '8': 
      case '9': {
        if (mode.curr != Mode::COLOR || vertex_selected == -1 || vertex_selected >= vertices.size()) break;
        vertices[vertex_selected].color = uniform.colors[key - '1'];
        break;
      }
      case '=': { // '+'
        Eigen::Matrix4f S;
        S << 1.2,0,0,0,
            0,1.2,0,0,
            0,0,1,0,
            0,0,0,1;
        uniform.transformation = uniform.transformation * S;
        break;
      }
      case '-': {
        Eigen::Matrix4f S;
        S << 0.8,0,0,0,
            0,0.8,0,0,
            0,0,1,0,
            0,0,0,1;
        uniform.transformation = uniform.transformation * S;
        break;
      }
      case 'w': { // down
        uniform.transformation(1,3) -= 0.4;
        break;
      }
      case 'a': { // right
        uniform.transformation(0,3) += 0.4;
        break;
      }
      case 's': { // up
        uniform.transformation(1,3) += 0.4;
        break;
      }
      case 'd': { // left
        uniform.transformation(0,3) -= 0.4;
        break;
      }
      case 'n': { // create a new frame
        if (triangle_selected == -1 || triangle_selected * 3 + 2 >= vertices.size()) break;
        
        add_keyframes(keyframes, vertices, triangle_selected * 3);
        break;
      }
      case 'm': { // play the animation
        if (keyframes.size() < 3) break;

        keyframing = true;
        frame_cnt = FRAME_COUNT;
        curr_keyframe = -1;
        break;
      }
      case 'q': { // clear the frames of the animation
        keyframes.clear();
        break;
      }
      default: {
        mode.curr = Mode::DEFAULT;
      }
        break;
    }

    if (mode.curr != mode.prev) {
      triangle_selected = -1;
      line_vertices.clear();
      keyframes.clear();
    }
    viewer.redraw_next = true;
  };

  viewer.redraw = [&](SDLViewer &viewer) {
    std::size_t lv_size = line_vertices.size();

    // Clear the framebuffer
    for (unsigned i=0;i<frameBuffer.rows();i++)
        for (unsigned j=0;j<frameBuffer.cols();j++)
            frameBuffer(i,j).color << 255,255,255,255;

    update_line_vertices(uniform, line_vertices, vertices, curr_vertex, clicked);

    // animation
    if (keyframing) {
      if (frame_cnt == FRAME_COUNT) {
        curr_keyframe++;
        for (int i = 0; i < 3; ++i) {
          tmp[i] = keyframes[(curr_keyframe + 1) * 3 + i].model - keyframes[curr_keyframe * 3 + i].model;
        }
        frame_cnt = 0;
      }
      
      for (int i = 0; i < 3; ++i) {
        vertices[triangle_selected * 3 + i].model = float(frame_cnt) / FRAME_COUNT * tmp[i] 
                                                    + keyframes[curr_keyframe * 3 + i].model;
      }

      frame_cnt++;
      if (3 * (curr_keyframe + 1) >= keyframes.size()) keyframing = false;
      viewer.redraw_next = true;
    }

    // rendering
    for (int i = 0; i + 2 < vertices.size(); i += 3) {
      std::vector<VertexAttributes> curr_frames {vertices[i], vertices[i + 1], vertices[i + 2]};
      rasterize_triangles(program, uniform, curr_frames, frameBuffer);

      std::vector<VertexAttributes> curr_lines {vertices[i], vertices[i + 1], 
                                                vertices[i + 1], vertices[i + 2], 
                                                vertices[i + 2], vertices[i]};
      rasterize_lines(program, uniform, curr_lines, 0.5, frameBuffer);
    }

    rasterize_lines(program, uniform, line_vertices, 0.5, frameBuffer);
    if (!clicked) {
      line_vertices.resize(lv_size);
    } else {
      clicked = false;
    }

    // Buffer for exchanging data between rasterizer and sdl viewer
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

    for (unsigned i=0; i<frameBuffer.rows();i++)
    {
        for (unsigned j=0; j<frameBuffer.cols();j++)
        {
            R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
            G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
            B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
            A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
        }
    }
    viewer.draw_image(R, G, B, A);
  };

  viewer.launch();

  return 0;
}
