#include "opencv2/opencv.hpp"
#include "JHCap.h"
#include <iostream>

using namespace cv;
using namespace std;

#define WINDOWS1_NAME "CAM1"
#define WINDOWS2_NAME "CAM2"
#define WINDOWS3_NAME "Merge"

#define SAMPLE_POINTS 5

API_STATUS JHcerameInit(int cam_id, int& width, int& height);
void makeVideo(string name, Mat& img);
void putFPS2Image(Mat& img);
vector<Point2f> findTargetPoint(Mat& img);

int main()
{
	Mat warpMat1;
	Mat warpMat2;

	int width;
	int height;

	int cam_count;
	int cam1_id = 0;
	int cam2_id = 1;
	API_STATUS api_status;

	CameraGetCount(&cam_count);

	if (cam_count != 2)
		return 0;

	/* Camera init */
	for (int i = 0; i < cam_count; i++){
		api_status = JHcerameInit(i, width, height);
		if (api_status == API_ERROR)
			return 0;
	}

	/* get camera serial numbers */
	char cam1_serial[13] = { '2', '8', '9', '2', '0', '0', '3', '3', '2', '0', '1', '7', '\0' };
	char cam2_serial[13] = { '2', '8', '5', '2', '0', '0', '3', '3', '2', '0', '1', '7', '\0' };
	char serial1[13] = { '\0' };
	char serial2[13] = { '\0' };

	if (CameraReadSerialNumber(0, serial1, 12) == API_ERROR)
		return 0;
	if (CameraReadSerialNumber(1, serial2, 12) == API_ERROR)
		return 0;

	if ((strcmp(serial1, cam1_serial) == 0) && (strcmp(serial2, cam2_serial) == 0)){
		cam1_id = 0;
		cam2_id = 1;
	}
	else if ((strcmp(serial2, cam1_serial) == 0) && (strcmp(serial1, cam2_serial) == 0)){
		cam1_id = 1;
		cam2_id = 0;
	}
	else{
		return 0; /* if no serial number pairs reinit cameras */
	}

	int len = width * height;

	Mat cam1_frame,cam2_frame;
	Mat bin_frame,disp_frame_dist;
	Mat warpImg1, warpImg2;
	Mat warp_target, warp_source;
	vector<Point2f> warp_point_src, warp_point_dst;

	cam1_frame.create(height, width, CV_8UC1);
	cam2_frame.create(height, width, CV_8UC1);

	FileStorage fs1("calibrate_perspection1.yml", FileStorage::READ);
	if (fs1.isOpened() == false) {
		cout << "can't find file:calibrate_perspection.yml\n";
		return 0;
	}
	fs1["warpMat"] >> warpMat1;
	fs1.release();

	namedWindow(WINDOWS1_NAME, WINDOW_AUTOSIZE);
	namedWindow(WINDOWS2_NAME, WINDOW_AUTOSIZE);

	warp_point_src.clear();
	warp_point_dst.clear();

	printf("Collect %d Points.\r\n",SAMPLE_POINTS);

	while (1)
	{
		CameraTriggerShot(cam1_id); /* Cam 1 trigger */
		CameraTriggerShot(cam2_id); /* Cam 2 trigger */

		api_status = CameraQueryImage(cam1_id, (unsigned char *)cam1_frame.data, &len, CAMERA_IMAGE_GRAY8); /* 18.5ms */
		api_status = CameraQueryImage(cam2_id, (unsigned char *)cam2_frame.data, &len, CAMERA_IMAGE_GRAY8); /* 18.5ms */

		if (api_status == API_OK) {
			
			Mat cam1_bin, cam2_bin;
			cam1_bin = cam1_frame.clone() > 100;
			cam2_bin = cam2_frame.clone() > 100;

			vector<Point2f> point_dst = findTargetPoint(cam1_bin);
			vector<Point2f> point_src = findTargetPoint(cam2_bin);/* cam2 point on image */

			if ((point_dst.size() == 1) && (point_src.size() == 1)){

				perspectiveTransform(point_dst, point_dst, warpMat1); /* cam1 measured point in world */

				if (warp_point_dst.size() < SAMPLE_POINTS){
					printf("src:[%5.2f,%5.2f], dis:[%5.3f,%5.3f], count:%d \r\n", point_src[0].x, point_src[0].y, point_dst[0].x, point_dst[0].y, warp_point_dst.size());

					if (waitKey(10) == ' '){
						warp_point_src.push_back(point_src[0]);
						warp_point_dst.push_back(point_dst[0]);
					}
				}
				else if (warp_point_dst.size() == SAMPLE_POINTS){

					warpMat2 = findHomography(warp_point_src, warp_point_dst, RANSAC);

					if (warpMat2.empty() != true){
						cout << warpMat2 << endl;

						FileStorage fs("calibrate_perspection2.yml", FileStorage::WRITE);
						fs << "warpMat" << warpMat2;
						fs.release();
						warp_point_dst.push_back(Point(-1, -1));
					}
					else{
						cout << "Calibration error! return to step one or recalibrate." << endl;
						while (waitKey(10) != 27);
						return 0;
					}
				}
				else{
					perspectiveTransform(point_src, point_src, warpMat2); /* cam2 measured point in world */
					printf("Real coordinate(Cam2 meas):[%6.3f,%6.3f]\r\n",point_src[0].x, point_src[0].y);
				}
			}
				cvtColor(cam1_frame, disp_frame_dist, CV_GRAY2BGR);		// display camera1 warp image
				imshow(WINDOWS1_NAME, disp_frame_dist); /* 0.5ms */
				cvtColor(cam2_frame, disp_frame_dist, CV_GRAY2BGR);		// display camera2 original image
				imshow(WINDOWS2_NAME, disp_frame_dist); /* 0.5ms */
		}
	}

	return 0;
}


API_STATUS JHcerameInit(int cam_id, int& width, int& height)
{
	if (CameraInit(cam_id) != API_OK)
		return API_ERROR;
	if (CameraReset(cam_id) != API_OK)
		return API_ERROR;
	if (CameraSetHighspeed(cam_id, false) != API_OK)
		return API_ERROR;
	if (CameraSetDelay(cam_id, 0) != API_OK)
		return API_ERROR;
	if (CameraSetGain(cam_id, 1) != API_OK)
		return API_ERROR;
	if (CameraSetExposure(cam_id, 100) != API_OK)
		return API_ERROR;
	if (CameraSetMirrorX(cam_id, true) != API_OK)
		return API_ERROR;
	if (CameraSetMirrorY(cam_id, true) != API_OK)
		return API_ERROR;
	if (CameraSetResolutionMode(cam_id, RESOLUTION_BINNING) != API_OK)
		return API_ERROR;
	if (CameraSetResolution(cam_id, 0, &width, &height) != API_OK)
		return API_ERROR;
	if (CameraSetSnapMode(cam_id, CAMERA_SNAP_TRIGGER) != API_OK)
		return API_ERROR;
	return API_OK;
}

void makeVideo(string name, Mat& img)
{
	static VideoWriter writer(name, CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(img.cols, img.rows), true);
	writer << img;
}

void putFPS2Image(Mat& img)
{
	static double t = 0;
	static double fps;
	char str[10];

	fps = 0.95 * fps + 0.05 * (getTickFrequency() / (double)(getTickCount() - t));
	t = (double)getTickCount();

	sprintf(str, "%3.1f", fps);
	string fpsString("FPS:");
	fpsString += str;
	putText(img, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
}



vector<Point2f> findTargetPoint(Mat& img)
{
	vector<vector<Point> >  contourts;
	vector<Vec4i> hierarchy;
	Mat image = img.clone();

	findContours(image, contourts, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point2f> center(contourts.size());
	vector<float>   radius(contourts.size());

	if (contourts.size() != 0){
		for (int index = 0; index >= 0; index = hierarchy[index][0])
			minEnclosingCircle(contourts[index], center[index], radius[index]);
	}
	else{
		center.push_back(Point2f(-1, -1));
	}
	return center;
}

