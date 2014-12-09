#include "project.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace Eigen;

void project(
	const char* point_cloud_file_name,
	float* image_data,
	unsigned int w,
	unsigned int h,
	const Eigen::Matrix4f& view_matrix,
	const Eigen::Matrix4f& projection_matrix
) {
	using image_array = Array<float, Dynamic, Dynamic, RowMajor>;

	Map<image_array> image(image_data, h, w);
	
	image_array depth_buffer(h, w);
	depth_buffer.setConstant(INFINITY);
	
	image.fill(0);
	
	std::ifstream point_cloud(point_cloud_file_name, std::ios_base::in);
	std::string line;
	
	while(std::getline(point_cloud, line)) {
		Vector3f point;
		std::istringstream line_stream(line);
		line_stream >> point[0];
		line_stream >> point[1];
		line_stream >> point[2];

		Vector4f view_point = view_matrix * point.homogeneous();
		
		Vector4f projected_point = projection_matrix * view_point;
		projected_point /= projected_point[3];
		
		if(projected_point[0] < -1.0f || projected_point[0] > 1.0f) continue;
		if(projected_point[1] < -1.0f || projected_point[1] > 1.0f) continue;
		if(projected_point[2] < -1.0f || projected_point[2] > 1.0f) continue;		
		
		std::ptrdiff_t x = ((projected_point[0] + 1.0f) * w) / 2;
		std::ptrdiff_t y = h - ((projected_point[1] + 1.0f) * h) / 2;
		if(x < 0 || x >= w || y < 0 || y >= h) continue;
		
		float depth = (view_point / view_point[3]).head(3).norm();
		float& buffered_depth = depth_buffer(y, x);
		
		if(depth < buffered_depth) {
			buffered_depth = depth;
			image(y, x) = depth;
		}
	}
}
