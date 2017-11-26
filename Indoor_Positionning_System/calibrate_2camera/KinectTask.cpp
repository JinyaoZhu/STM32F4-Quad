#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "SkeletonStream.h"
#include "KinectRecognition.h"
#include <math.h>

using namespace std;
using namespace cv;


extern bool CameraInitFlag;
extern Point3f Kinect_HandPoistion;
mutex kinect_lock;

int KinectTask(void)
{
	int64 tick1;
	int64 tick2;
	double duration = 0;

	Sleep(2000);

	if (CameraInitFlag == false)
		return 0;

	ofstream fout;
	fout.open("trackData.txt");

	Mat skeletonImage(240, 320, CV_8UC3, Scalar::all(0));
	Mat skeletonPointTrackImage(240, 320, CV_8UC3, Scalar::all(0));

	KinectRecognition KinectSkeleton;

	HRESULT hr = KinectSkeleton.initialize(10);
	if (FAILED(hr)) {
		cout << "can't  initial the kinect" << endl;
		getchar();
		return 0;
	}

	/* display window */
	namedWindow("skeletonImage", CV_WINDOW_AUTOSIZE);
	imshow("skeletonImage", skeletonImage);

	while (1)
	{
		tick1 = getTickCount();

		KinectSkeleton.updata();

		KinectSkeleton.drawSkeleton(skeletonImage);

		KinectSkeleton.drawSkeletonPositionTrackingImage(skeletonPointTrackImage, NUI_SKELETON_POSITION_WRIST_RIGHT);

		imshow("skeletonImage", skeletonImage);
		imshow("skeletonPointTrackImage", skeletonPointTrackImage);

		if ((KinectSkeleton.getSkeletonTrackedState() == NUI_SKELETON_TRACKED) && (KinectSkeleton.countSkeletonTracked() >= 7)) {

			if (KinectSkeleton.Is_PutDownPose())
				imshow("result", Mat(300, 300, CV_8UC3, Scalar::all(255)));
			else
				imshow("result", Mat(300, 300, CV_8UC3, Scalar::all(0)));

			lock_guard<mutex> lk(kinect_lock);
			Kinect_HandPoistion = KinectSkeleton.calculateTargetPosition();

			//cout << Kinect_HandPoistion << endl;
			//cout << "tracked : " << KinectSkeleton.getSkeletonPositionTrackedState(NUI_SKELETON_POSITION_HAND_RIGHT) << ',' << KinectSkeleton.getSkeletonPositon(NUI_SKELETON_POSITION_HAND_RIGHT) << endl;
			//fout << "tracked : " << KinectSkeleton.getSkeletonPositionTrackedState(NUI_SKELETON_POSITION_HAND_RIGHT) << ',' << KinectSkeleton.getSkeletonPositon(NUI_SKELETON_POSITION_HAND_RIGHT) << endl;
		}
		

		if (cvWaitKey(20) == 27) break;

		tick2 = getTickCount();
		//duration = duration * 0.95 + 0.05 * (tick2 - tick1) / getTickFrequency();
		//printf("duration:%8.5f ms\r\n", duration * 1000); /* 1ms */
	}
	fout.close();
	return 0;
}