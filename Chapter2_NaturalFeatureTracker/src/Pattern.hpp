/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#pragma once

////////////////////////////////////////////////////////////////////
// File includes:
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

#include <opencv2/opencv.hpp>
#include <array>

/**
 * Store the image data and computed descriptors of target pattern
 */
struct Pattern
{
	cv::Size                  size;
	cv::Mat                   frame;
	cv::Mat                   grayImg;

	std::vector<cv::KeyPoint> keypoints;
	cv::Mat                   descriptors;

	std::vector<cv::Point2f>  points2d;
	std::vector<cv::Point3f>  points3d;
};

/**
 * Intermediate pattern tracking info structure
 */
struct PatternTrackingInfo
{	
	/// Coordinates of the image corners on the current frame.
	std::vector<cv::Point2f>  points2d;
	
	/// Computed homography transformation
	cv::Mat                   homography;

	/// Computed rigid transformation
	Transformation            pose3d;

	void draw2dContour(cv::Mat& image, cv::Scalar color) const;

	/// Compute pattern pose using PnP algorithm
	void computePose(const Pattern& pattern, const CameraCalibration& calibration);
};

