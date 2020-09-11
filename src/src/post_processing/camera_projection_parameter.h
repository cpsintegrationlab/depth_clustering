/*
 * camera_projection_parameter.h
 *
 *  Created on: Sep 11, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_CAMERA_PROJECTION_PARAMETER_H_
#define SRC_POST_PROCESSING_CAMERA_PROJECTION_PARAMETER_H_

#include <vector>

struct CameraProjectionParameter
{
	std::vector<double> intrinsic;
	std::vector<double> extrinsic;
	int width;
	int height;
	bool correct_distortions;

	CameraProjectionParameter() :
			intrinsic(), extrinsic(), width(0), height(0), correct_distortions(false)
	{
	}
};

#endif /* SRC_POST_PROCESSING_CAMERA_PROJECTION_PARAMETER_H_ */
