/*
 * test.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "depth_clustering.h"

int
main()
{
	DepthClustering depth_clustering;

	depth_clustering.process("../../data/moosmann/velodyne_slam/scenario_1/");
}
