// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <memory>

// Utilities for the Assignment
#include "raster.h"
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;

// Read vertices from an off file
void load_off(const string &filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F, vector<VertexAttributes> &vertices) {
	ifstream in(filename);
	string token;
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

    for (int j = 0; j < 3; ++j){
			vertices.push_back(VertexAttributes(V(F(i, j), 0), V(F(i, j), 1), V(F(i, j), 2), 1));
		}
	}
}

// Ex1
void rendermodel(const vector<VertexAttributes> &vertices) {
  // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return va;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(1,0,0);
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	rasterize_triangles(program,uniform,vertices,frameBuffer);

	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("Ex1_bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
}

// EX2
void wireframe(const vector<VertexAttributes> &vertices) {
	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants
	UniformAttributes uniform;

	// Basic rasterization program
  Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return va;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(1,0,0);
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	rasterize_lines(program, uniform, vertices, 0.03, frameBuffer);

  vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("Ex2_wireframe_bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
}


int main() 
{
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
	// One triangle in the center of the screen
	vector<VertexAttributes> vertices;

  load_off(filename, V, F, vertices);

  rendermodel(vertices);

  // wireframe(vertices);
	
	return 0;
}
