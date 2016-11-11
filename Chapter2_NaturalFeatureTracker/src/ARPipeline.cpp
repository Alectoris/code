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

////////////////////////////////////////////////////////////////////
// File includes:
#include "ARPipeline.hpp"

#include <numeric>

ARPipeline::ARPipeline(const CameraCalibration& camera)
	: mMinNumberTrackedPoints(16)
	, mMinNumberPoseInliers(8)
{
	mCameraCalibartion = camera;
	mFeatureExtractor = cv::AKAZE::create();
	mFeatureMatcher = cv::BFMatcher::create("BruteForce-Hamming");
}

void ARPipeline::setTemplateFromImage(const cv::Mat& referenceImage)
{
	buildTemplateFromImage(mTemplate, referenceImage);
	mTrackStatus = TrackStatus();
}

bool ARPipeline::buildTemplateFromImage(ImageTemplate& tmpl, const cv::Mat& referenceImage) const
{
	cv::cvtColor(referenceImage, tmpl.grayImg, cv::COLOR_BGR2GRAY);
	tmpl.size = referenceImage.size();

	// Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
	tmpl.points2d.resize(4);
	tmpl.points3d.resize(4);

	// Image dimensions
	const float w = referenceImage.cols;
	const float h = referenceImage.rows;

	// Normalized dimensions:
	const float maxSize = std::max(w, h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;

	tmpl.points2d[0] = cv::Point2f(0, 0);
	tmpl.points2d[1] = cv::Point2f(w, 0);
	tmpl.points2d[2] = cv::Point2f(w, h);
	tmpl.points2d[3] = cv::Point2f(0, h);

	tmpl.points3d[0] = cv::Point3f(-unitW, -unitH, 0);
	tmpl.points3d[1] = cv::Point3f(unitW, -unitH, 0);
	tmpl.points3d[2] = cv::Point3f(unitW, unitH, 0);
	tmpl.points3d[3] = cv::Point3f(-unitW, unitH, 0);

	// Compute feature points for template matching
	mFeatureExtractor->detectAndCompute(tmpl.grayImg, cv::noArray(), tmpl.keypoints, tmpl.descriptors);

	// Compute points for tracking
	cv::goodFeaturesToTrack(tmpl.grayImg, tmpl.pointsForTracking, 100, 0.01, 10, cv::noArray(), 5, true);

	return tmpl.keypoints.size() > mMinNumberTrackedPoints &&
		tmpl.pointsForTracking.size() > mMinNumberTrackedPoints;
}

bool ARPipeline::matchTemplate(const cv::Mat& inspectionImage)
{
	/// Detect keypoints and extract descriptors from the image
	cv::cvtColor(inspectionImage, mCurrInspectionImageGray, cv::COLOR_BGR2GRAY);
	mFeatureExtractor->detectAndCompute(mCurrInspectionImageGray, cv::noArray(), mInspectionKeypoints, mInspectionDescriptors);

	/// Match observed descriptors with descriptors from template to obtain matches
	std::vector<cv::DMatch> matches;
	mFeatureMatcher->match(mInspectionDescriptors, mTemplate.descriptors, matches);

	if (matches.size() < 8)
	{
		mTrackStatus.isValid = false;
		return false;
	}

	std::vector<cv::Point2f> referencePoints(matches.size()), inspectionPoints(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		referencePoints[i] = mTemplate.keypoints[matches[i].trainIdx].pt;
		inspectionPoints[i] = mInspectionKeypoints[matches[i].queryIdx].pt;
	}

	mTrackStatus.currInspectionH = cv::findHomography(referencePoints, inspectionPoints, cv::RANSAC, 3, mTrackStatus.inliersMask);
	size_t numInliers = std::count(mTrackStatus.inliersMask.begin(), mTrackStatus.inliersMask.end(), 1);
	if (numInliers < mMinNumberPoseInliers)
	{
		mTrackStatus.isValid = false;
		return false;
	}

	mTrackStatus.isValid = checkHomographyValid(mTrackStatus.currInspectionH);

	if (mTrackStatus.isValid)
	{
		// Initialize the points for tracking
		cv::perspectiveTransform(mTemplate.pointsForTracking, mTrackStatus.prevInspectionPoints, mTrackStatus.currInspectionH);
	}
	return mTrackStatus.isValid;
}

bool ARPipeline::trackTemplate(const cv::Mat& inspectionImage)
{
	mCurrInspectionImageGray.copyTo(mPrevInspectionImageGray);
	cv::cvtColor(inspectionImage, mCurrInspectionImageGray, cv::COLOR_BGR2GRAY);

	if (mTrackStatus.isValid)
	{
		/// KLT tracking
		{			cv::calcOpticalFlowPyrLK(
				mPrevInspectionImageGray,
				mCurrInspectionImageGray,
				mTrackStatus.prevInspectionPoints,
				mTrackStatus.currInspectionPoints,
				mTrackStatus.inliersMask,
				cv::noArray(),
				cv::Size(13, 13),
				3);
			size_t trackedPoints = std::count(mTrackStatus.inliersMask.begin(), mTrackStatus.inliersMask.end(), 1);
			mTrackStatus.isValid = trackedPoints >= mMinNumberTrackedPoints;
		}
	}

	cv::Mat hdelta;

	if (mTrackStatus.isValid)
	{
		std::vector<cv::Point2f> prevInspectionPoints;
		std::vector<cv::Point2f> currInspectionPoints;
		for (int i = 0; i < mTrackStatus.inliersMask.size(); i++)
		{
			if (mTrackStatus.inliersMask[i])
			{
				prevInspectionPoints.push_back(mTrackStatus.prevInspectionPoints[i]);
				currInspectionPoints.push_back(mTrackStatus.currInspectionPoints[i]);
			}
		}

		hdelta = cv::findHomography(prevInspectionPoints, currInspectionPoints, cv::RANSAC, 3, mTrackStatus.inliersMask);
		size_t numInliers = std::count(mTrackStatus.inliersMask.begin(), mTrackStatus.inliersMask.end(), 1);
		mTrackStatus.isValid = numInliers >= mMinNumberPoseInliers;
	}

	if (mTrackStatus.isValid)
	{
		mTrackStatus.currInspectionH = mTrackStatus.currInspectionH * hdelta;
		mTrackStatus.isValid = checkHomographyValid(mTrackStatus.currInspectionH);
	}

	/// Refine a pose using image registration
	if (mTrackStatus.isValid)
	{
		try
		{
			cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, mMaxIterationsECC, mTerminationEpsECC);

			cv::Mat h;
			mTrackStatus.currInspectionH.convertTo(h, CV_32F);
			cv::findTransformECC(mTemplate.grayImg, mCurrInspectionImageGray, h, cv::MOTION_HOMOGRAPHY, criteria);
			h.convertTo(mTrackStatus.currInspectionH, mTrackStatus.currInspectionH.type());

			cv::perspectiveTransform(mTemplate.pointsForTracking, mTrackStatus.prevInspectionPoints, mTrackStatus.currInspectionH);

			mTrackStatus.isValid = checkHomographyValid(mTrackStatus.currInspectionH);
		}
		catch (cv::Exception& e)
		{
			mTrackStatus.isValid = false;
		}
	}

	return mTrackStatus.isValid;
}


bool ARPipeline::checkHomographyValid(const cv::Mat& h) const
{
	assert(h.rows == 3);
	assert(h.cols == 3);

	if (std::abs(cv::determinant(h)) > 10000)
	{
		return false;
	}

	return true;
}