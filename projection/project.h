#include <Eigen/Eigen>

void project(
	const char* point_cloud_file_name,
	float* image_data,
	unsigned int w,
	unsigned int h,
	const Eigen::Matrix4f& view_matrix,
	const Eigen::Matrix4f& projection_matrix
);