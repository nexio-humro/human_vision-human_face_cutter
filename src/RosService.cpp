#include <RosServices.hpp>

namespace RS
{
	bool extractFacesFromImage(human_vision_exchange::CutFaces::Request  &req, human_vision_exchange::CutFaces::Response &res)
	{
		std::vector< std::vector< std::vector<double>>> objects;
		
		for(size_t i = 0; i < req.keypoints.size(); i++)
		{
			std::vector<std::vector<double>> keypoints;
			keypoints = RSF::changeKeypointsToVector(req.keypoints[i]);
			objects.push_back(keypoints);
		}
		
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
		cv::Mat cvImage = cv_ptr->image;
		
		std::vector<cv::Mat> faces;
		faces = FCF::extractAllFaces(cvImage, objects);
		
		// resize responses faces
		res.faces.resize(faces.size());
		
		for(size_t i = 0; i < faces.size(); i++)
		{
			cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::Header(), "rgb8", faces.at(i));	
			cvImage.toImageMsg(res.faces[i]);
		}
		
		return true;
	}
}
