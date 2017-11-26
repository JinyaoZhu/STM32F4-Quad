#include "opencv2/opencv.hpp"
#include "stdio.h"
#include <iostream>
#include <iomanip>
#include "JHCap.h"

using namespace cv;
using namespace std;

#define WINDOWS_NAME "camera image"

API_STATUS JHcerameInit(int cam_id, int& width, int& height);
void putFPS2Image(Mat& img);
vector<Point2f> findTargetPoint(Mat& img);
void calibrate_perspection(Mat& img, vector<Point2f> p_src, const float l_real, Mat &warpMat);

int main()
{
	Mat img;
	Mat warp_frame;
	int cam_count;
	cout.precision(3);

	Mat warpMat;
	int width;
	int height;

   int cam1_id, cam2_id;

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

	Mat cam_frame, warpImg;
	Mat bin_frame, disp_frame_dist;
	vector<Point2f> warp_point_src, warp_point_dst;

	cam_frame.create(height, width, CV_8UC1);

	namedWindow(WINDOWS_NAME, WINDOW_AUTOSIZE);

	while (1)
	{
		CameraTriggerShot(cam1_id); /* Cam 1 trigger */
		api_status = CameraQueryImage(cam1_id, (unsigned char *)cam_frame.data, &len, CAMERA_IMAGE_GRAY8);

		if (api_status == API_OK) {

			bin_frame = cam_frame > 100;

			cvtColor(bin_frame, disp_frame_dist, CV_GRAY2BGR);		// display camera bin image
			putFPS2Image(disp_frame_dist);
			imshow(WINDOWS_NAME, disp_frame_dist); /* 0.5ms */

			vector<Point2f> point_src = findTargetPoint(bin_frame);

			if ((warp_point_src.size()<4)&&(waitKey(10) == ' '))
				warp_point_src.push_back(point_src[0]);

			if (warp_point_src.size() == 4){
				warp_point_src.push_back(Point2f(-1,-1));

				float l_square = 1.70f;
				cout << "Please input the length of the square:(uint m)\r\n";
				cin >> l_square;

				calibrate_perspection(bin_frame, warp_point_src, l_square, warpMat);

				cout << "warpM= " << warpMat << endl;

				FileStorage fs("calibrate_perspection1.yml", FileStorage::WRITE);
				fs << "warpMat" << warpMat;
				fs.release();

				while (waitKey(10) != ' ');
			}

			if (warp_point_src.size() < 4){
				cout << "Point: " << point_src[0] << " Count: " << warp_point_src.size() << endl;
			}
			else{
				vector<Point2f> point_dist;
				perspectiveTransform(point_src, point_dist, warpMat);
				cout << "Real coordinate: " << point_dist[0] << endl;
				if (waitKey(10) == 27)
					break;
			}
		}
	}
	return 0;
}

void calibrate_perspection(Mat& img, vector<Point2f> p_src,const float l_real, Mat &warpMat)
{
	float l = 200;
	Point2f src_p[4];
	Point2f dst_p[4];
	Point2f center_point = Point2f(img.cols / 2.0f, img.rows / 2.0f);


	src_p[0] = p_src[0];
	src_p[1] = p_src[1];
	src_p[2] = p_src[2];
	src_p[3] = p_src[3];

	l = l_real;

	dst_p[0] = Point2f(-l / 2, l / 2);
	dst_p[1] = Point2f(l / 2, l / 2);
	dst_p[2] = Point2f(l / 2, -l / 2);
	dst_p[3] = Point2f(-l / 2, -l / 2);

	warpMat = getPerspectiveTransform(src_p, dst_p);
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
