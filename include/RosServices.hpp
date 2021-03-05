#ifndef ROS_SERVICES_HPP
#define ROS_SERVICES_HPP

#include <iostream>

#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "human_vision_exchange/CutFaces.h"
#include "FaceCuttingFunctions.hpp"

#include <RosServiceFunctions.hpp>

namespace RS
{
	bool extractFacesFromImage(human_vision_exchange::CutFaces::Request  &req, human_vision_exchange::CutFaces::Response &res);
}

#endif
