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
#include "PatternDetector.hpp"
#include "CameraCalibration.hpp"
#include "GeometryTypes.hpp"

#include <opencv2/opencv.hpp>

// Alias. TODO: Delete me
using ImageTemplate = Pattern;

struct TrackStatus
{
	bool    isValid = false;

	std::vector<uint8_t>        inliersMask;
	std::vector<cv::Point2f>    prevInspectionPoints;
	std::vector<cv::Point2f>    currInspectionPoints;

	cv::Mat currInspectionH;
};

class ARPipeline
{
public:
	ARPipeline(const CameraCalibration& camera);

	///
	/// @brief Builds a ImageTemplate from the image frame.
	///
	void setTemplateFromImage(const cv::Mat& referenceImage);

	///
	/// @brief Tries to locate image template on the inspection image.
	///
	/// @inspectionImage - Current image
	/// @tmpl - Template data for the tracking
	///
	bool matchTemplate(const cv::Mat& inspectionImage);

	///
	/// @brief Tries to track image template from previous to current (inspection) frame.
	///
	bool trackTemplate(const cv::Mat& inspectionImage);

	const ImageTemplate& getTemplate()  const { return mTemplate; }
	const TrackStatus& getTrackStatus() const { return mTrackStatus; }

protected:
	cv::Ptr<cv::Feature2D>         mFeatureExtractor;
	cv::Ptr<cv::DescriptorMatcher> mFeatureMatcher;
	CameraCalibration			   mCameraCalibartion;

	bool checkHomographyValid(const cv::Mat& h) const;

	///
	/// @brief Builds a ImageTemplate from the image frame.
	///
	bool buildTemplateFromImage(ImageTemplate& tmpl, const cv::Mat& referenceImage) const;

private:
	cv::Mat                     mCurrInspectionImageGray;
	cv::Mat                     mPrevInspectionImageGray;

	std::vector<cv::KeyPoint>   mInspectionKeypoints;
	cv::Mat                     mInspectionDescriptors;

	ImageTemplate				mTemplate;
	TrackStatus                 mTrackStatus;

	size_t mMinNumberTrackedPoints;
	size_t mMinNumberPoseInliers;
	size_t mMaxIterationsECC = 10;
	double mTerminationEpsECC = 1e-3;
};
