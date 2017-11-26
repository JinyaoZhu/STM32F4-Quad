#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include "opencv2/opencv.hpp"
#include "windows.h"

extern int KinectTask(void);
extern int CameraTask(void);

bool CameraInitFlag = false;

cv::Point3f Kinect_HandPoistion;

int main()
{
	std::thread t1(CameraTask);
	//std::thread t2(KinectTask);

	t1.join();
	//t2.join();

	//Beep(440, 3000);

	std::cout << "all routines are over!" << std::endl;

	return 0;
}
