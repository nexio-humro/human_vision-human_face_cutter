#include <SystemFunctions.hpp>
#include <MainData.hpp>
#include <RosServices.hpp>

#include "ros/ros.h"
#include "ros/package.h"

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "human_face_cutter");
	ros::NodeHandle node("/human_face_cutter/");
	
	ros::Rate loop_rate(10);
	
	ros::ServiceServer faceVectorService = node.advertiseService("cutFaces", RS::extractFacesFromImage);
	
	ros::spin();
    
    return 0;
}
