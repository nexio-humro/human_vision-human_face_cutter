#ifndef ROS_SERVICE_FUNCTIONS_HPP
#define ROS_SERVICE_FUNCTIONS_HPP

#include <iostream>
#include <vector>

#include "human_vision_exchange/Keypoints2d.h"

namespace RSF
{
	std::vector<std::vector<double> > changeKeypointsToVector(human_vision_exchange::Keypoints2d& keypoints);
}

#endif
