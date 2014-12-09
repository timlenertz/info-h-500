#include "project.h"
#include <iostream>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <Eigen/Eigen>

int main(int argc, const char* argv[]) {
	if(argc < 37) {
		std::cerr << "Usage: project infile outfile w h viewmatrix[16] projectionmatrix[16]" << std::endl;
		std::cerr << "       37 arguments total, matrices in column-major order." << std::endl;
		return EXIT_FAILURE;
	}

	const char* in_filename = argv[1];
	const char* out_filename = argv[2];
	unsigned int w = std::atoi(argv[3]);
	unsigned int h = std::atoi(argv[4]);

	float view_matrix_data[16];
	for(std::ptrdiff_t i = 0; i < 16; ++i)
		view_matrix_data[i] = std::atof(argv[5 + i]);
	Eigen::Matrix4f view_matrix = Eigen::Map<Eigen::Matrix4f>(view_matrix_data);

	float projection_matrix_data[16];
	for(std::ptrdiff_t i = 0; i < 16; ++i)
		projection_matrix_data[i] = std::atof(argv[21 + i]);
	Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(projection_matrix_data);

	std::unique_ptr<float[]> image(new float[w * h]);
	
	project(
		in_filename,
		image.get(),
		w,
		h,
		view_matrix,
		projection_matrix
	);
	
	std::fstream out_file(out_filename, std::ios_base::out | std::ios_base::binary);
	out_file.write(
		reinterpret_cast<const char*>(image.get()),
		w * h * sizeof(float)
	);
	
	return EXIT_SUCCESS;
}