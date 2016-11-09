#pragma once

struct ImageTemplate
{
	cv::Mat                   grayscaleImage;
	cv::Size 				  frameSize;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat 				  descriptors;	
};

struct TrackStatus
{
    bool    isValid;

    std::vector<uint8_t>        inliersMask;
    std::vector<cv::Point2f>    prevInspectionPoints;
    std::vector<cv::Point2f>    currInspectionPoints;

    cv::Mat prevInspectionImage;

    cv::Mat currInspectionPose;
    cv::Mat prevInspectionPose;

};

class NFTPipeline
{
public:
	NFTPipeline();

    ///
    /// Builds a ImageTemplate from the image frame.
    ///
	void buildTemplateFromImage(ImageTemplate& tmpl, cv::Mat referenceImage);

    ///
    /// Tries to locate image template on the inspection image.
    /// @inspectionImage - Current image
    /// @tmpl - Template data for the tracking
    ///
	bool matchTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl);

    ///
    /// Tries to track image template from previous to current (inspection) frame.
    ///
	bool trackTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl);


    ///
    /// Returns last known pose
    ///
    const cv::Mat& getPose() const 
    {
        return mTrackStatus.currInspectionPose;
    }

protected:
	cv::Ptr<cv::Feature2D>      mFeatureExtractor;
    cv::Ptr<cv::FeatureMatcher> mFeatureMatcher;

    bool checkHomographyValid(const cv::Mat& h) const;

private:
    cv::Mat                     mInspectionImageGray;
    std::vector<cv::KeyPoint>   mInspectionKeypoints;
    cv::Mat                     mInspectionDescriptors;

    TrackStatus                 mTrackStatus;

    size_t mMinNumberTrackedPoints;
    size_t mMinNumberPoseInliers;
    size_t mMaxIterationsECC = 5000;
    double mTerminationEpsECC = 1e-10; 

};

NFTPipeline::NFTPipeline()
{
    mFeatureExtractor = cv::AKAZE::create();
    mFeatureMatcher = cv::BruteForceMatcher::create();
}

bool NFTPipeline::buildTemplateFromImage(ImageTemplate& tmpl, const cv::Mat& referenceImage)
{
    cv::cvtColor(referenceImage, tmpl.grayscaleImage, cv::COLOR_BGR2GRAY);
    tmpl.frameSize = referenceImage.size();
    mFeatureExtractor->detectAndCompute(tmpl.grayscaleImage, tmpl.keypoints, tmpl.descriptors);
    return tmpl.keypoints.size() > 0;
}

bool NFTPipeline::matchTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl)
{
    /// Detect keypoints and extract descriptors from the image
    cv::cvtColor(inspectionImage, mInspectionImageGray, cv::COLOR_BGR2GRAY);
    mFeatureExtractor->detectAndCompute(mInspectionImageGray, mInspectionKeypoints, mInspectionDescriptors);

    /// Match observed descriptors with descriptors from template to obtain matches
    auto matcher = cv::BruteForceMatcher::create();
    std::vector<cv::DMatch> matches;
    matcher->match(queryDescriptors, tmpl.descriptors, matches);

    if (matches.size() < 8)
    {
        mTrackStatus.isValid = false;
        return false;
    }

    std::vector<cv::Point2f> referencePoints(matches.size()), inspectionPoints(matches.size());
    for (size_t i = 0; i < matches.size(); i++)
    {
        referencePoints[i] = tmpl.keypoints[matches[i].trainIdx];
        inspectionPoints[i] = tmpl.keypoints[matches[i].queryIdx];
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

bool NFTPipeline::trackTemplate(const cv::Mat& inspectionImage, const ImageTemplate& tmpl)
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

const cv::Mat& NFTPipeline::getPose() const 
{
    std::vector<cv::Point2f> corners2d;
    cv::perspectiveTransform(tmpl.corners2d, corners2d, mTrackStatus.currInspectionPose);

    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux,taux;

    cv::solvePnP(tmpl.corners3d, corners2d, camMatrix, distCoeff, raux, taux);
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);

    cv::Mat_<float> rotMat(3,3); 
    cv::Rodrigues(Rvec, rotMat);

    // Copy to transformation matrix
    for (int col=0; col<3; col++)
    {
        for (int row=0; row<3; row++)
        {        
            m.transformation.r().mat[row][col] = rotMat(row,col); // Copy rotation component
        }
        m.transformation.t().data[col] = Tvec(col); // Copy translation component
    }

    return mTrackStatus.currInspectionPose;
}

bool NFTPipeline::checkHomographyValid(const cv::Mat& h) const
{
    assert(h.rows == 3);
    assert(h.cols == 3);

    if (std::abs(h.det()) > 10000)
    {
        return false;
    }

    return true;
}

int main(int argc, const char * argv[])
{
    cv::Mat targetImage = cv::imread("target.jpg");

    NFTPipeline pipeline;
    VisualizationPipeline renderPipeline;

    cv::VideoCapture cap(0);

    cv::Mat currentFrame;

    ImageTemplate tmpl;
    pipeline.buildTemplateFromImage(tmpl, targetImage);

    bool isTracking = false;
    while (cap.isOpen())
    {
        cap >> currentFrame;
        renderPipeline.drawFrame(currentFrame);

        if (isTracking)
        {
            isTracking = pipeline.trackTemplate(currentFrame, tmpl);
        }
        else
        {
            isTracking = pipeline.matchTemplate(currentFrame, tmpl);
        }

        renderPipeline.renderCube(isTracking, pipeline.getPose());
    }

    return 0;
}