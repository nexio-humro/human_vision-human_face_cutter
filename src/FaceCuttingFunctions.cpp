#include <FaceCuttingFunctions.hpp>

namespace FCF
{
	double distance(std::vector<double>& vec1, std::vector<double>& vec2)
	{
		double result = 0.0;
		
		if( (vec1.size() >= 2) && (vec2.size() >= 2) & (vec1.size() <= 3) && (vec2.size() <= 3) )
		{
			result = std::sqrt(std::pow(vec1.at(0) - vec2.at(0), 2) + std::pow(vec1.at(1) - vec2.at(1), 2));
		}
		
		return result;
	}
	
	cv::Rect getFrameEarNoseDistance(std::vector<std::vector<double>>& points)
	{
		cv::Rect result;
		
		if(FCF::areNosePointsCorrect(points))
		{
			std::vector<double> nose_cord = points.at(static_cast<int>(BODY_PARTS::NOSE));
			std::vector<double> left_ear_cord = points.at(static_cast<int>(BODY_PARTS::LEFT_EAR));
			std::vector<double> right_ear_cord = points.at(static_cast<int>(BODY_PARTS::RIGHT_EAR));
			
			float first_distance = distance(nose_cord, left_ear_cord);
			float second_distance = distance(nose_cord, right_ear_cord);
			
			float longestDistance;
			
			if(first_distance < second_distance)
			{
				longestDistance = second_distance;
			}
			else
			{
				longestDistance = first_distance;
			}
			
			cv::Point leftTop(nose_cord[0] - static_cast<int>(longestDistance), nose_cord[1] - static_cast<int>(longestDistance));
			cv::Point rightBottom(nose_cord[0] + static_cast<int>(longestDistance), nose_cord[1] + static_cast<int>(longestDistance));
			
			result = cv::Rect(leftTop, rightBottom);
		}
		
		return result;
	}
	
	bool areNosePointsCorrect(std::vector<std::vector<double>>& points)
	{
		bool result = true;
		
		if(points.size() == bodyElementsAmount)
		{
			for(size_t i = 0; i < framePartsNoseDistance.size(); i++)
			{
				std::vector<double> value = points.at(static_cast<int>(framePartsNoseDistance.at(i)));
				
				if(value[0] == -1)
				{
					result = false;
					break;
				}
				
				if(value[1] == -1)
				{
					result = false;
					break;
				}
			}
		}
		else
		{
			result = false;
		}
		
		return result;
	}
	
	bool isFrameCorrect(const cv::Rect& frame, const cv::Mat& image)
	{
		bool result = true;
		
		if( (frame.x < 0) || (frame.y < 0) || ((frame.x + frame.width) >= image.cols) || ((frame.y + frame.height) >= image.rows) || (frame.width == 0) || (frame.height == 0) )
		{
			result = false;
		}
		
		return result;
	}
	
	cv::Mat extractFace(const cv::Mat& image, const  cv::Rect frame)
	{
		cv::Mat result;
		
		if( (FCF::isFrameCorrect(frame, image) == true) && (image.empty() == false) )
		{
			cv::Mat copiedImage = image.clone();
			result = copiedImage(frame);
		}
		
		return result;
	}
	
	bool prepareCvFace(cv::Mat& faceImage, int imageSize)
	{
		bool result = true;
		
		if( (faceImage.empty() == false) && (imageSize > 0) )
		{
			cv::resize(faceImage, faceImage, cv::Size(imageSize, imageSize));
			cv::cvtColor(faceImage, faceImage, cv::COLOR_BGRA2RGB);
		}
		else
		{
			result = false;
		}
		
		return result;
	}
	
	std::vector<cv::Mat> extractAllFaces(cv::Mat cv_image, std::vector<std::vector<std::vector<double>>> objects, int imageSize)
	{
        std::vector<cv::Mat> faces;
		
		for (size_t i = 0; i < objects.size(); i++) 
		{
			bool arePointsOk = FCF::areNosePointsCorrect(objects.at(i));
			
			if(arePointsOk == true)
			{
				cv::Rect frame = FCF::getFrameEarNoseDistance(objects.at(i));
				
				bool isFrameOK = FCF::isFrameCorrect(frame, cv_image);
				
				if(isFrameOK == true)
				{
					cv::Mat faceImage = FCF::extractFace(cv_image, frame);
					bool preparedFace = FCF::prepareCvFace(faceImage, imageSize);
					
					if(preparedFace == true)
					{
						faces.push_back(faceImage);
					}
				}
			}
		} 
		
		return faces;
	} 
}
