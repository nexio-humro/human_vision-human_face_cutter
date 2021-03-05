#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/Image.h"

#include "human_vision_exchange/CutFaces.h"
#include "human_vision_exchange/Keypoints2d.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/package.h"

std::string getPathToImage(int argc, char **argv);
cv_bridge::CvImage readImage(std::string& pathToImage);
std::vector<std::vector<double>> prepareAmyData();
std::vector<std::vector<double>> prepareHoltData();
human_vision_exchange::Keypoints2d prepareKeypoint2d(std::vector<std::vector<double>>& data);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cut_faces_mock");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<human_vision_exchange::CutFaces>("/human_face_cutter/cutFaces");
	human_vision_exchange::CutFaces cutFaces;

	std::string pathToImage;
	pathToImage = getPathToImage(argc, argv);

	// read image
    cv_bridge::CvImage cvImage;
    cvImage = readImage(pathToImage);

	// add two persons data
	// Amy
	std::vector< std::vector<double>> Amy;
	Amy = prepareAmyData();
	
	// Holt
	std::vector< std::vector<double>> Holt;
	Holt = prepareHoltData();
	
	human_vision_exchange::CutFaces srv;
	
	
	// fill request
	cvImage.toImageMsg(srv.request.image);
	srv.request.keypoints.resize(2);
	human_vision_exchange::Keypoints2d keypointsAmy = prepareKeypoint2d(Amy);
	srv.request.keypoints[0] = prepareKeypoint2d(Amy);
	human_vision_exchange::Keypoints2d keypointsHolt = prepareKeypoint2d(Holt);
	srv.request.keypoints[1] = keypointsHolt;
	
	client.call(srv);
	
	// save faces
	std::string pathToOutput = ros::package::getPath("human_face_cutter") + "/output/";
	for(size_t i = 0; i < srv.response.faces.size(); i++)
	{
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.faces[i], sensor_msgs::image_encodings::BGR8);
		cv::Mat cvImage = cv_ptr->image;
		std::string pathToFaceImage = pathToOutput + "face" + std::to_string(i) + ".png";
		cv::imwrite(pathToFaceImage, cvImage);
	}

	return 0;
}

std::string getPathToImage(int argc, char **argv)
{
	std::string pathToImage;
	if(argc > 1)
	{
		pathToImage = argv[1];
	}
	else
	{
		pathToImage = ros::package::getPath("human_face_cutter") + "/images/Brooklyn.png";
	}
	
	return pathToImage;
}

cv_bridge::CvImage readImage(std::string& pathToImage)
{
#if CV_MAJOR_VERSION < 4
    cv::Mat image = cv::imread(pathToImage.c_str(), CV_LOAD_IMAGE_COLOR); 
#else
    cv::Mat image = cv::imread(pathToImage.c_str(), cv::IMREAD_COLOR); 
#endif 

    cv::waitKey(30);
    cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image);	
    
    return cvImage;
}

std::vector<std::vector<double>> prepareAmyData()
{	
	std::vector< std::vector<double>> Amy;

	Amy.push_back(std::vector<double>{670.0, 340.0}); // NOSE
	Amy.push_back(std::vector<double>{666.0, 444.0}); // NECK
	Amy.push_back(std::vector<double>{501.0, 423.0}); // RIGHT_SHOULDER
	Amy.push_back(std::vector<double>{523.0, 480.0}); // RIGHT_ELBOW
	Amy.push_back(std::vector<double>{616.0, 454.0}); // RIGHT_WRIST
	Amy.push_back(std::vector<double>{792.0, 456.0}); // LEFT_SHOULDER
	Amy.push_back(std::vector<double>{804.0, 645.0}); // LEFT_ELBOW
	Amy.push_back(std::vector<double>{727.0, 813.0}); // LEFT_WRIST
	Amy.push_back(std::vector<double>{472.0, 820.0}); // RIGHT_HIP
	Amy.push_back(std::vector<double>{529.0, 1071.0}); // RIGHT_KNEE
	Amy.push_back(std::vector<double>{0.0, 0.0}); // RIGHT_ANKLE
	Amy.push_back(std::vector<double>{759.0, 831.0}); // LEFT_HIP
	Amy.push_back(std::vector<double>{708.0, 1066.0}); // LEFT_KNEE
	Amy.push_back(std::vector<double>{0.0, 0.0}); // LEFT_ANKLE
	Amy.push_back(std::vector<double>{637.0, 307.0}); // RIGHT_EYE
	Amy.push_back(std::vector<double>{699.0, 307.0}); // LEFT_EYE
	Amy.push_back(std::vector<double>{588.0, 295.0}); // RIGHT_EAR
	Amy.push_back(std::vector<double>{729.0, 313.0}); // LEFT_EAR

	return Amy;
}

std::vector<std::vector<double>> prepareHoltData()
{
	std::vector< std::vector<double>> Holt;
	
	Holt.push_back(std::vector<double>{1377.0, 276.0}); // NOSE
	Holt.push_back(std::vector<double>{1396.0, 373.0}); // NECK
	Holt.push_back(std::vector<double>{1255.0, 358.0}); // RIGHT_SHOULDER
	Holt.push_back(std::vector<double>{1216.0, 502.0}); // RIGHT_ELBOW
	Holt.push_back(std::vector<double>{1404.0, 424.0}); // RIGHT_WRIST
	Holt.push_back(std::vector<double>{1573.0, 340.0}); // LEFT_SHOULDER
	Holt.push_back(std::vector<double>{1731.0, 457.0}); // LEFT_ELBOW
	Holt.push_back(std::vector<double>{1440.0, 522.0}); // LEFT_WRIST
	Holt.push_back(std::vector<double>{1305.0, 786.0}); // RIGHT_HIP
	Holt.push_back(std::vector<double>{1329.0, 1074.0}); // RIGHT_KNEE
	Holt.push_back(std::vector<double>{0.0, 0.0}); // RIGHT_ANKLE
	Holt.push_back(std::vector<double>{1626.0, 775.0}); // LEFT_HIP
	Holt.push_back(std::vector<double>{1554.0, 1072.0}); // LEFT_KNEE
	Holt.push_back(std::vector<double>{0.0, 0.0}); // LEFT_ANKLE
	Holt.push_back(std::vector<double>{1336.0, 249.0}); // RIGHT_EYE
	Holt.push_back(std::vector<double>{1407.0, 240.0}); // LEFT_EYE
	Holt.push_back(std::vector<double>{1294.0, 250.0}); // RIGHT_EAR
	Holt.push_back(std::vector<double>{1446.0, 231.0}); // LEFT_EAR
	
	return Holt;
}

human_vision_exchange::Keypoints2d prepareKeypoint2d(std::vector<std::vector<double>>& data)
{
	human_vision_exchange::Keypoints2d result;
	
	for(size_t i = 0; i < data.size(); i++)
	{
		geometry_msgs::Point32 point;
		point.x = data.at(i).at(0);
		point.y = data.at(i).at(1);
		point.z = 0.0;
		result.points[i] = point;
	}
	
	return result;
}
