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

ARPipeline::ARPipeline()
{
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
	cv::cvtColor(referenceImage, tmpl.grayscaleImage, cv::COLOR_BGR2GRAY);
	tmpl.frameSize = referenceImage.size();
	mFeatureExtractor->detectAndCompute(tmpl.grayscaleImage, cv::noArray(), tmpl.keypoints, tmpl.descriptors);
	return tmpl.keypoints.size() > 0;
}

bool ARPipeline::matchTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl)
{
	/// Detect keypoints and extract descriptors from the image
	cv::cvtColor(inspectionImage, mInspectionImageGray, cv::COLOR_BGR2GRAY);
	mFeatureExtractor->detectAndCompute(mInspectionImageGray, cv::noArray(), mInspectionKeypoints, mInspectionDescriptors);

	/// Match observed descriptors with descriptors from template to obtain matches
	std::vector<cv::DMatch> matches;
	mFeatureMatcher->match(mInspectionDescriptors, tmpl.descriptors, matches);

	if (matches.size() < 8)
	{
		mTrackStatus.isValid = false;
		return false;
	}

	std::vector<cv::Point2f> referencePoints(matches.size()), inspectionPoints(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		referencePoints[i] = tmpl.keypoints[matches[i].trainIdx].pt;
		inspectionPoints[i] = tmpl.keypoints[matches[i].queryIdx].pt;
	}

	mTrackStatus.currInspectionPose = cv::findHomography(referencePoints, inspectionPoints, cv::RANSAC, 3, mTrackStatus.inliersMask);
	size_t numInliers = std::count(inliersMask.begin(), inliersMask.end(), 1);
	if (numInliers < mMinNumberPoseInliers)
	{
		mTrackStatus.isValid = false;
		return false;
	}

	mTrackStatus.isValid = checkHomographyValid(h);
	return mTrackStatus.isValid;
}

bool ARPipeline::trackTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl)
{
	cv::cvtColor(inspectionImage, mInspectionImageGray, cv::COLOR_BGR2GRAY);

	if (mTrackStatus.isValid)
	{
		/// KLT tracking
		{
			cv::calcOpticalFlowPyrLK(mPrevInspectionImage,
				mInspectionImageGray,
				mTrackStatus.prevInspectionPoints,
				mTrackStatus.currInspectionPoints,
				mTrackStatus.inliersMask,
				cv::noArray());
			size_t trackedPoints = std::count(inliersMask.begin(), inliersMask.end(), 1);
			mTrackStatus.isValid = ttrackedPoints >= mMinNumberTrackedPoints;
		}

		/// TM tracking
		/*
		{
		cv::warpPerspective(tmpl.grayscaleImage, expectedImage, mTrackStatus.currInspectionPose, inspectionImage.size());
		cv::perspectiveTransform(tmpl.referencePoints, expectedPoints, mTrackStatus.currInspectionPose);

		for (int i = 0; i < expectedPoints.size(); i++)
		{
		cv::Mat patch = expectedImage();
		cv::matchTemplate(expectedImage, inspectionImage, );
		}
		}*/
	}

	if (mTrackStatus.isValid)
	{
		cv::Mat hdelta = cv::findHomography(mTrackStatus.prevInspectionPoints, mTrackStatus.currInspectionPoints, cv::RANSAC, 3, mTrackStatus.inliersMask);
		size_t numInliers = std::count(mTrackStatus.inliersMask.begin(), mTrackStatus.inliersMask.end(), 1);
		mTrackStatus.isValid = numInliers >= mMinNumberPoseInliers;
	}

	if (mTrackStatus.isValid)
	{
		std::swap(mTrackStatus.mCurrInspectionPoints, mTrackStatus.prevInspectionPoints);
		mTrackStatus.currInspectionPose = mTrackStatus.currInspectionPose * hdelta;
		mTrackStatus.isValid = checkHomographyValid(mTrackStatus.currInspectionPose);
	}

	/// Refine a pose using image registration
	if (mTrackStatus.isValid)
	{
		try
		{
			cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, mMaxIterationsECC, mTerminationEpsECC);
			cv::findTransformECC(tmpl.grayscaleImage, mInspectionImageGray, mTrackStatus.currInspectionPose, cv::MOTION_HOMOGRAPHY, criteria);
			mTrackStatus.isValid = checkHomographyValid(mTrackStatus.currInspectionPose);
		}
		catch (cv::Exception)
		{
			mTrackStatus.isValid = false;
		}
	}

	return mTrackStatus.isValid;
}

const cv::Mat& ARPipeline::getPose() const
{
	std::vector<cv::Point2f> corners2d;
	cv::perspectiveTransform(tmpl.corners2d, corners2d, mTrackStatus.currInspectionPose);

	cv::Mat Rvec;
	cv::Mat_<float> Tvec;
	cv::Mat raux, taux;

	cv::solvePnP(tmpl.corners3d, corners2d, camMatrix, distCoeff, raux, taux);
	raux.convertTo(Rvec, CV_32F);
	taux.convertTo(Tvec, CV_32F);

	cv::Mat_<float> rotMat(3, 3);
	cv::Rodrigues(Rvec, rotMat);

	// Copy to transformation matrix
	for (int col = 0; col<3; col++)
	{
		for (int row = 0; row<3; row++)
		{
			m.transformation.r().mat[row][col] = rotMat(row, col); // Copy rotation component
		}
		m.transformation.t().data[col] = Tvec(col); // Copy translation component
	}

	return mTrackStatus.currInspectionPose;
}

bool ARPipeline::checkHomographyValid(const cv::Mat& h) const
{
	assert(h.rows == 3);
	assert(h.cols == 3);

	if (std::abs(h.det()) > 10000)
	{
		return false;
	}

	return true;
}