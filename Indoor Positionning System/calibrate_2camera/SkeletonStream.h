#ifndef __SKELETONSTREAM_H_
#define __SKELETONSTREAM_H_

#include <windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
using namespace cv;

#define SKELETON_COUNT 12

class SkeletonStream {
private:
	HANDLE skeletonEvent;
	NUI_SKELETON_DATA skeletonData;
	NUI_SKELETON_TRACKING_STATE trackingState;
	int skeleton_id;
public:

	SkeletonStream();

	~SkeletonStream();

	HRESULT initialize(int kinect_elevation_angle);

	bool updata();

	int countSkeletonTracked();

	void drawSkeleton(Mat &image);

	void drawSkeletonPositionTrackingImage(Mat &image, int skeleton_num);

	NUI_SKELETON_TRACKING_STATE getSkeletonTrackedState();

	bool getSkeletonPositionTrackedState(int skeleton_num);

	Point3f getSkeletonPositon(int skeleton_num);

	CvPoint getSkeletonOnDepth(int skeleton_num);

};

#endif