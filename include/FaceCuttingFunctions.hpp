#ifndef FACE_CUTTING_FUNCTIONS_HPP
#define FACE_CUTTING_FUNCTIONS_HPP

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#include <BodyPartEnum.hpp>

namespace FCF
{	
	const size_t bodyElementsAmount = 18;
	
	double distance(std::vector<double>& vec1, std::vector<double>& vec2);
	
	static std::vector<BODY_PARTS> framePartsNoseDistance = {
								BODY_PARTS::LEFT_EAR,
								BODY_PARTS::RIGHT_EAR,
								BODY_PARTS::NOSE };
								
	cv::Rect getFrameMinMax(std::vector<std::vector<double>>& points);
	cv::Rect getFrameEarNoseDistance(std::vector<std::vector<double>>& points);
	
	bool areNosePointsCorrect(std::vector<std::vector<double>>& points);
	bool isFrameCorrect(const cv::Rect& frame, const cv::Mat& image);
	cv::Mat extractFace(const cv::Mat& image, const cv::Rect frame);
	bool prepareCvFace(cv::Mat& faceImage, int imageSize);
	
	std::vector<cv::Mat> extractAllFaces(cv::Mat cv_image, std::vector<std::vector<std::vector<double>>> objects, int imageSize = 150);
}

#endif
