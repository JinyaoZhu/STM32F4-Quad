#include "opencv2/opencv.hpp"
#include "JHCap.h"
#include <iostream>

using namespace cv;
using namespace std;

#define WINDOWS1_NAME "CAM1"
#define WINDOWS2_NAME "CAM2"

API_STATUS JHcerameInit(int cam_id, int& width, int& height);
Point3d getIntersection(Point3d a1, Point3d b1, Point3d a2, Point3d b2);
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

	vector<Point2f> cam1_projected_point, cam2_projected_point;

	cam1_frame.create(height, width, CV_8UC1);
	cam2_frame.create(height, width, CV_8UC1);

	FileStorage fs1("calibrate_perspection1.yml", FileStorage::READ);
	if (fs1.isOpened() == false) {
		cout << "can't find file:calibrate_perspection.yml\n";
		return 0;
	}
	fs1["warpMat"] >> warpMat1;
	fs1.release();

	FileStorage fs2("calibrate_perspection2.yml", FileStorage::READ);
	if (fs2.isOpened() == false) {
		cout << "can't find file:calibrate_perspection.yml\n";
		return 0;
	}
	fs2["warpMat"] >> warpMat2;
	fs2.release();

	int numSamplePoints;

	cout << "Input number of sample Positions:" ;
	cin >> numSamplePoints;
	numSamplePoints = numSamplePoints * 2;

	while (1)
	{
		CameraTriggerShot(cam1_id); /* Cam 1 trigger */
		CameraTriggerShot(cam2_id); /* Cam 2 trigger */
		api_status = CameraQueryImage(cam1_id, (unsigned char *)cam1_frame.data, &len, CAMERA_IMAGE_GRAY8 ); /* 18.5ms */
		api_status = CameraQueryImage(cam2_id, (unsigned char *)cam2_frame.data, &len, CAMERA_IMAGE_GRAY8 ); /* 18.5ms */

	/*	imshow("camera1 image", cam1_frame);
		imshow("camera2 image", cam2_frame);*/

		if (api_status == API_OK) {
			Mat cam1_bin, cam2_bin;
			cam1_bin = cam1_frame > 100;
			cam2_bin = cam2_frame > 100;

			vector<Point2f> cam1_point = findTargetPoint(cam1_bin);
			vector<Point2f> cam2_point = findTargetPoint(cam2_bin);

			if ((cam1_point.size() == 1) && (cam2_point.size() == 1)){

				perspectiveTransform(cam1_point, cam1_point, warpMat1);
				perspectiveTransform(cam2_point, cam2_point, warpMat2);

				cout << "cam1: " << cam1_point[0] << "cam2: " << cam2_point[0] << "count:" << cam2_projected_point.size() << endl;

				if (waitKey(10) == ' '){
					cam1_projected_point.push_back(cam1_point[0]);
					cam2_projected_point.push_back(cam2_point[0]);
				}

				if (cam2_projected_point.size() == numSamplePoints){

					Point3f cam1_pos, cam2_pos;
					vector<Point3f> cam1_p, cam2_p;
					float tmp = 0;
					float stick_h = 0;

					cout << "Please input the height of stick: (uint: m)\r\n";
					cin >> stick_h;

					for (int i = 0; i < numSamplePoints; i += 2){
						cam1_p.push_back(Point3f((cam1_projected_point[i].x + cam2_projected_point[i].x) / 2, (cam1_projected_point[i].y + cam2_projected_point[i].y) / 2, stick_h));
						cam1_p.push_back(Point3f(cam1_projected_point[i + 1].x, cam1_projected_point[i + 1].y, 0));

						cam2_p.push_back(Point3f((cam1_projected_point[i].x + cam2_projected_point[i].x) / 2, (cam1_projected_point[i].y + cam2_projected_point[i].y) / 2, stick_h));
						cam2_p.push_back(Point3f(cam2_projected_point[i + 1].x, cam2_projected_point[i + 1].y, 0));
					}

					for (int i = 0; i < numSamplePoints - 2; i += 2){
						for (int j = i + 2; j < numSamplePoints; j += 2){
							cam1_pos += (Point3f)getIntersection(cam1_p[i], cam1_p[i + 1], cam1_p[j], cam1_p[j + 1]);
							cam2_pos += (Point3f)getIntersection(cam2_p[i], cam2_p[i + 1], cam2_p[j], cam2_p[j + 1]);
							tmp += 1;
						}
					}

					cam1_pos = cam1_pos / tmp;
					cam2_pos = cam2_pos / tmp;

					cout << "cam1_pos: " << cam1_pos << endl << "cam2_pos: " << cam2_pos << endl;

					FileStorage fs("CameraInfo.yml", FileStorage::WRITE);
					fs << "camera1_warpMat" << warpMat1;
					fs << "camera2_warpMat" << warpMat2;
					fs << "camera1_position" << cam1_pos;
					fs << "camera2_position" << cam2_pos;
					fs.release();

					cout << "done... Press ESC to quit(on image)...\r\n";

					while (waitKey(10) != 27);
					return 0;
				}
			}
			cvtColor(cam1_frame, disp_frame_dist, CV_GRAY2BGR);		// display camera1 image
			imshow(WINDOWS1_NAME, disp_frame_dist); /* 0.5ms */
			cvtColor(cam2_frame, disp_frame_dist, CV_GRAY2BGR);		// display camera2 image
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


#define VECTOR_T Point3d
//Point3d getIntersection(Point3d a1, Point3d b1, Point3d a2, Point3d b2)
//{
//	Point3d  p;
//	VECTOR_T n1 = a1 - b1;
//	VECTOR_T n2 = a2 - b2;
//
//	p.y = (a2.x - a1.x) / (n1.x / n1.y - n2.x / n2.y) + a1.y / (1 - n1.y*n2.x / n1.x / n2.y) - a2.y / (n1.x*n2.y / n1.y / n2.x - 1);
//	//p.z = (a2.x-a1.x) / (n1.x/n1.z - n2.x/n2.z) + a1.z / (1 - n1.z*n2.x/n1.x/n2.z) - a2.z / (n1.x*n2.z/n1.z/n2.x - 1);
//	p.z = (p.y - a1.y) / n1.y*n1.z + a1.z;
//	p.x = (p.y - a1.y) / n1.y*n1.x + a1.x;
//	return p;
//}


Point3d getIntersection(Point3d a1, Point3d b1, Point3d a2, Point3d b2)
{
	VECTOR_T N2 = a2 - b2;  //calculate unit dirction vector of vector A2B2
	N2 = N2 / sqrt(N2.x * N2.x + N2.y * N2.y + N2.z * N2.z);

	Point3d p1, p2, q1, q2, m1, m2;

	p1 = (a1 - b1)*9999.0 + a1;
	q1 = (b1 - a1)*9999.0 + b1;

	m1 = (p1 + q1) / 2;

	for (int i = 0; i<1000; i++){

		VECTOR_T A2M1 = m1 - a2;
		VECTOR_T A2M2 = N2 * (A2M1.x * N2.x + A2M1.y * N2.y + A2M1.z * N2.z);
		VECTOR_T M1M2 = A2M2 - A2M1;
		double dis_m1m2 = M1M2.x * M1M2.x + M1M2.y * M1M2.y + M1M2.z * M1M2.z;
		m2 = M1M2 + m1;

		VECTOR_T P1Q1 = q1 - p1;
		if (sqrt(P1Q1.x * P1Q1.x + P1Q1.y * P1Q1.y) < 0.0001)	break;

		VECTOR_T A2P1 = p1 - a2;
		VECTOR_T A2P2 = N2 * (A2P1.x * N2.x + A2P1.y * N2.y + A2P1.z * N2.z);
		VECTOR_T P1P2 = A2P2 - A2P1;
		double dis_p1p2 = P1P2.x * P1P2.x + P1P2.y * P1P2.y + P1P2.z * P1P2.z;
		p2 = P1P2 + p1;

		VECTOR_T A2Q1 = q1 - a2;
		VECTOR_T A2Q2 = N2 * (A2Q1.x * N2.x + A2Q1.y * N2.y + A2Q1.z * N2.z);
		VECTOR_T Q1Q2 = A2Q2 - A2Q1;
		double dis_q1q2 = Q1Q2.x * Q1Q2.x + Q1Q2.y * Q1Q2.y + Q1Q2.z * Q1Q2.z;
		q2 = Q1Q2 + q2;

		if ((dis_m1m2 <= dis_q1q2) && (dis_m1m2 <= dis_p1p2)){
			if (dis_p1p2 > dis_q1q2)
				p1 = m1;
			else
				q1 = m1;

			m1 = (p1 + q1) / 2;
			p1 = (p1 + m1) / 2;
			q1 = (q1 + m1) / 2;
		}
		else if (dis_p1p2 > dis_q1q2){
			p1 = m1;
			m1 = q1;
			q1 = q1 + (m1 - p1);
		}
		else {
			q1 = m1;
			m1 = p1;
			p1 = p1 + (m1 - q1);
		}
	}
	return (m1 + m2) / 2;
}