#pragma once

#include <Eigen/Core>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
		color << 0,0,0,1;
    prev_color << 0,0,0,1;
    model.setIdentity();
	}

  VertexAttributes(Eigen::Vector4f position, Eigen::Vector4f color) : 
    position(std::move(position)), color(std::move(color)), prev_color(std::move(color)){
      model.setIdentity();
  }

  void transform(Eigen::Vector4f vec, Eigen::Matrix4f M) {
    model.col(3) += M.inverse() * vec;
  }

  void change_color(Eigen::Vector4f new_color) {
    prev_color = color;
    color = new_color;
  }

  void reset_color() {
    color = prev_color;
  }

  // Interpolates the vertex attributes
  static VertexAttributes interpolate(
    const VertexAttributes& a,
    const VertexAttributes& b,
    const VertexAttributes& c,
    const float alpha, 
    const float beta, 
    const float gamma
  ) 
  {
    VertexAttributes r;
    r.position = alpha*a.position + beta*b.position + gamma*c.position;
    r.color = alpha*a.color + beta*b.color + gamma*c.color;
    return r;
  }

	Eigen::Vector4f position;
	Eigen::Vector4f color;
  Eigen::Vector4f prev_color;
  Eigen::Matrix4f model;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}

	Eigen::Matrix<uint8_t,4,1> color;
};

class UniformAttributes
{
	public:
  UniformAttributes() {
		transformation.setIdentity();

    colors.push_back(Eigen::Vector4f(1,0,0,1)); // 1
		colors.push_back(Eigen::Vector4f(0,1,0,1)); // 2
		colors.push_back(Eigen::Vector4f(0,0,1,1)); // 3
    colors.push_back(Eigen::Vector4f(0,0,0,1)); // 4
		colors.push_back(Eigen::Vector4f(1,1,1,1)); // 5
    colors.push_back(Eigen::Vector4f(1,0,1,1)); // 6
    colors.push_back(Eigen::Vector4f(1,1,0,1)); // 7
    colors.push_back(Eigen::Vector4f(0,1,1,1)); // 8
    colors.push_back(Eigen::Vector4f(0.5,0.1,0.2,1)); // 9
	}

	Eigen::Matrix4f transformation;
  std::vector<Eigen::Vector4f> colors;
};

class Mode
{
  public:
  Mode() {
		curr = DEFAULT;
    prev = DEFAULT;
	};

	enum MODE_STATUS { 
    DEFAULT,
    INSERTION,
    TRANSLATION,
    DELETION,
    COLOR 
  };
	MODE_STATUS curr;
  MODE_STATUS prev;
};