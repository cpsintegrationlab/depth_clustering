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
	double filter_height;
	double filter_tunnel_left;
	double filter_tunnel_right;
	double filter_tunnel_front;
	bool correct_distortions;
	bool use_filter_height;
	bool use_filter_tunnel;

	CameraProjectionParameter() :
			intrinsic(), extrinsic(), width(0), height(0), filter_height(0), filter_tunnel_left(10), filter_tunnel_right(
					10), filter_tunnel_front(75), correct_distortions(false), use_filter_height(
					false), use_filter_tunnel(false)
	{
	}
};

#endif /* SRC_POST_PROCESSING_CAMERA_PROJECTION_PARAMETER_H_ */
