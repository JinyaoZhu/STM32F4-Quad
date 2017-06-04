#include "opencv2/opencv.hpp"
#include <windows.h>
#include <iostream>
#include <NuiApi.h>
#include "SkeletonStream.h"

using namespace std;
using namespace cv;

SkeletonStream::SkeletonStream()
{
	skeleton_id = 0;
	trackingState = NUI_SKELETON_NOT_TRACKED;
}

SkeletonStream::~SkeletonStream()
{
	NuiShutdown();
}

Point3f SkeletonStream::getSkeletonPositon(int skeleton_num)
{
	Point3f p = Point3f(0, 0, 0);

	if (getSkeletonPositionTrackedState(skeleton_num) == true) {
		p.x = skeletonData.SkeletonPositions[skeleton_num].x;
		p.y = skeletonData.SkeletonPositions[skeleton_num].y;
		p.z = skeletonData.SkeletonPositions[skeleton_num].z;
	}

	return p;
}

bool SkeletonStream::getSkeletonPositionTrackedState(int skeleton_num)
{
	return ((skeletonData.eSkeletonPositionTrackingState[skeleton_num] != NUI_SKELETON_POSITION_NOT_TRACKED) ? true : false);
}

NUI_SKELETON_TRACKING_STATE SkeletonStream::getSkeletonTrackedState()
{
	return trackingState;
}



HRESULT SkeletonStream::initialize(int kinect_elevation_angle)
{
	skeletonEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	/* initialize kinect sensor : skeleton  */
	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
	if (FAILED(hr))
	{
		cout << "NuiInitialize failed" << endl;
		return hr;
	}

	/* set kinect elevation angle */
	NuiCameraElevationSetAngle(kinect_elevation_angle);

	/* open kinect skeleton stream */
	hr = NuiSkeletonTrackingEnable(skeletonEvent, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
	if (FAILED(hr)) {
		cout << "Could not open skeleton image stream video" << endl;
		NuiShutdown();
		return hr;
	}

}

int SkeletonStream::countSkeletonTracked()
{
	int cnt = 0;
	for (int i = 2; i < SKELETON_COUNT; i++){
		if (skeletonData.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED)
			cnt++;
	}
	return cnt;
}

bool SkeletonStream::updata()
{
	NUI_SKELETON_FRAME skeletonFrame;
	if (WaitForSingleObject(skeletonEvent, INFINITE) == 0) {
		HRESULT hr = NuiSkeletonGetNextFrame(0, &skeletonFrame);
		if (SUCCEEDED(hr)) {
			NuiTransformSmooth(&skeletonFrame, NULL); // smooth skeleton frame

			// Show skeleton only if it is tracked, and the center-shoulder joint is at least inferred.
			if (skeletonFrame.SkeletonData[skeleton_id].eTrackingState == NUI_SKELETON_TRACKED &&
				skeletonFrame.SkeletonData[skeleton_id].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] == NUI_SKELETON_POSITION_TRACKED) {
				trackingState = NUI_SKELETON_TRACKED;
				skeletonData = skeletonFrame.SkeletonData[skeleton_id];
			}
			else {	// if the one tracked is gone,find another .
				for (int i = 0; i <= NUI_SKELETON_COUNT; i++) {
					if (i == NUI_SKELETON_COUNT) {
						trackingState = NUI_SKELETON_NOT_TRACKED;
					}
					if (skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED &&
						skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] == NUI_SKELETON_POSITION_TRACKED) {
						skeleton_id = i;
						trackingState = NUI_SKELETON_TRACKED;
						skeletonData = skeletonFrame.SkeletonData[i];
						break;
					}
				}
			}
			return true;
		}
		else
			return false;
	}
}

CvPoint SkeletonStream::getSkeletonOnDepth(int skeleton_num)
{
	CvPoint p;
	float fx, fy;
	NuiTransformSkeletonToDepthImage(skeletonData.SkeletonPositions[skeleton_num], &fx, &fy, NUI_IMAGE_RESOLUTION_320x240);
	p.x = fx;
	p.y = fy;
	return p;
}

void SkeletonStream::drawSkeletonPositionTrackingImage(Mat &image, int skeleton_num)
{
	image.setTo(0);

	if (trackingState != NUI_SKELETON_NOT_TRACKED) {
		CvPoint p;
		char str[10];
		p.x = image.cols / 2 + skeletonData.SkeletonPositions[skeleton_num].x / 1.5 * image.cols;
		p.y = image.rows / 2 - skeletonData.SkeletonPositions[skeleton_num].y / 1.5 * image.cols;
		circle(image, p, 3, cvScalar(255, 255, 0), -1, 8, 0);
		sprintf(str, "%7.3f", skeletonData.SkeletonPositions[skeleton_num].z);
		string fpsString("depth:");
		fpsString += str;
		putText(image, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
	}
}

/* draw seated skeleton image , 10 skeleton point */
void SkeletonStream::drawSkeleton(Mat &image)
{
	CvScalar color = cvScalar(255, 255, 0);
	CvPoint pointSet[SKELETON_COUNT];
	float fx, fy;

	image.setTo(0);

	if (trackingState != NUI_SKELETON_NOT_TRACKED) {
		for (int i = 0; i < SKELETON_COUNT; i++) {
			NuiTransformSkeletonToDepthImage(skeletonData.SkeletonPositions[i], &fx, &fy, NUI_IMAGE_RESOLUTION_320x240);
			pointSet[i].x = (int)fx;
			pointSet[i].y = (int)fy;
			circle(image, pointSet[i], 3, cvScalar(0, 255, 0), 1, 8, 0);
		}

		if ((pointSet[NUI_SKELETON_POSITION_HEAD].x != 0 || pointSet[NUI_SKELETON_POSITION_HEAD].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_HEAD], pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], color, 2);

		// left part
		if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_HAND_LEFT].x != 0 || pointSet[NUI_SKELETON_POSITION_HAND_LEFT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], pointSet[NUI_SKELETON_POSITION_HAND_LEFT], color, 2);

		// right part
		if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], color, 2);
		if ((pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y != 0) &&
			(pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].x != 0 || pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].y != 0))
			line(image, pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], pointSet[NUI_SKELETON_POSITION_HAND_RIGHT], color, 2);
	}
}
