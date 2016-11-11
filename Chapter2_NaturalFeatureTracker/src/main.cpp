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
#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "DebugHelpers.hpp"

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <opencv2/opencv.hpp>

/**
 * Processes a recorded video or live view from web-camera and allows you to adjust homography refinement and 
 * reprojection threshold in runtime.
 */
void processVideo(const cv::Mat& patternImage, CameraCalibration& calibration, cv::VideoCapture& capture);

/**
 * Performs full detection routine on camera frame and draws the scene using drawing context.
 * In addition, this function draw overlay with debug information on top of the AR window.
 * Returns true if processing loop should be stopped; otherwise - false.
 */
bool processFrame(const cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);

int main(int argc, const char * argv[])
{
    // Change this calibration to yours:
    CameraCalibration calibration(526.58037684199849f, 524.65577209994706f, 318.41744018680112f, 202.96659047014398f);
    
    if (argc < 2)
    {
        std::cout << "Input image not specified" << std::endl;
        std::cout << "Usage: markerless_ar_demo <pattern image> [filepath to recorded video or image]" << std::endl;
        return 1;
    }

    // Try to read the pattern:
    cv::Mat patternImage = cv::imread(argv[1]);
    if (patternImage.empty())
    {
        std::cout << "Input image cannot be read" << std::endl;
        return 2;
    }

    if (argc == 2)
    {
		cv::VideoCapture cap(0);
        processVideo(patternImage, calibration, cap);
    }
	if (argc == 3)
	{
		cv::VideoCapture cap;
		cv::String filename = argv[2];
		cap.open(filename);
		processVideo(patternImage, calibration, cap);
	}
    else
    {
        std::cerr << "Invalid number of arguments passed" << std::endl;
        return 1;
    }

    return 0;
}

void processVideo(const cv::Mat& patternImage, CameraCalibration& calibration, cv::VideoCapture& capture)
{
    // Grab first frame to get the frame dimensions
    cv::Mat currentFrame;  
    capture >> currentFrame;

    // Check the capture succeeded:
    if (currentFrame.empty())
    {
        std::cout << "Cannot open video capture device" << std::endl;
        return;
    }

    cv::Size frameSize(currentFrame.cols, currentFrame.rows);

	ARPipeline pipeline(calibration);
	pipeline.setTemplateFromImage(patternImage);
	
    ARDrawingContext renderPipeline("Markerless AR", frameSize, calibration);

    bool shouldQuit = false;
	bool isTracking = false;
	Transformation t;

	do
    {
        capture >> currentFrame;
        if (currentFrame.empty())
        {
            shouldQuit = true;
            break;
        }

		renderPipeline.updateBackground(currentFrame);

		if (isTracking)
		{
			isTracking = pipeline.trackTemplate(currentFrame);
		}
		else
		{
			isTracking = pipeline.matchTemplate(currentFrame);
		}

		pipeline.getTemplate().computePose(t, pipeline.getTrackStatus().currInspectionH, calibration);
		renderPipeline.renderCube(isTracking, t);
		cv::waitKey(1);

    } while (!shouldQuit);
}


bool processFrame(const cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
    // Clone image used for background (we will draw overlay on it)
    cv::Mat img = cameraFrame.clone();

    // Draw information:
    //if (pipeline.m_patternDetector.enableHomographyRefinement)
    //    cv::putText(img, "Pose refinement: On   ('h' to switch off)", cv::Point(10,15), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));
    //else
    //    cv::putText(img, "Pose refinement: Off  ('h' to switch on)",  cv::Point(10,15), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));

    //cv::putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.homographyReprojectionThreshold) + "( Use'-'/'+' to adjust)", cv::Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0,200,0));
	

    // Set a new camera frame:
    drawingCtx.updateBackground(img);

	

    // Find a pattern and update it's detection status:
    //drawingCtx.isPatternPresent = pipeline.processFrame(cameraFrame);

    // Update a pattern pose:
    //drawingCtx.patternPose = pipeline.getTrackStatus().getPose();

    // Request redraw of the window:
    drawingCtx.updateWindow();

    // Read the keyboard input:
    int keyCode = cv::waitKey(5); 

    bool shouldQuit = false;
    if (keyCode == '+' || keyCode == '=')
    {
        //pipeline.m_patternDetector.homographyReprojectionThreshold += 0.2f;
        //pipeline.m_patternDetector.homographyReprojectionThreshold = std::min(10.0f, pipeline.m_patternDetector.homographyReprojectionThreshold);
    }
    else if (keyCode == '-')
    {
        //pipeline.m_patternDetector.homographyReprojectionThreshold -= 0.2f;
        //pipeline.m_patternDetector.homographyReprojectionThreshold = std::max(0.0f, pipeline.m_patternDetector.homographyReprojectionThreshold);
    }
    else if (keyCode == 'h')
    {
        //pipeline.m_patternDetector.enableHomographyRefinement = !pipeline.m_patternDetector.enableHomographyRefinement;
    }
    else if (keyCode == 27 || keyCode == 'q')
    {
        shouldQuit = true;
    }

    return shouldQuit;
}


