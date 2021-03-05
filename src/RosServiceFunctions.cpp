#include <RosServiceFunctions.hpp>

namespace RSF
{
	std::vector<std::vector<double> > changeKeypointsToVector(human_vision_exchange::Keypoints2d& keypoints)
	{
		std::vector<std::vector<double> > result;
		
		for(size_t i = 0; i < keypoints.points.size(); i++)
		{
			std::vector<double> data;
			data.resize(2);
			data.at(0) = keypoints.points[i].x;
			data.at(1) = keypoints.points[i].y;
			
			result.push_back(data);
		}
		
		return result;
	}
}
