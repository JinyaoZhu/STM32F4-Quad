#include "opencv2/opencv.hpp"
#include "JHCap.h"
#include <iostream>
#include "com_driver.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <windows.h>

using namespace cv;
using namespace std;

#define WINDOWS1_NAME "CAM1"
#define WINDOWS2_NAME "CAM2"
#define WINDOWS3_NAME "BIN1"
#define WINDOWS4_NAME "BIN2"

#define PI (3.1415926535f)

extern Point3f Kinect_HandPoistion;
extern bool CameraInitFlag;

API_STATUS JHcerameInit(int cam_id, int& width, int& height);
void putFPS2Image(Mat& img);
vector<Point2f> findTargetPoint(Mat& img);
void indexPoint(Mat& img, Point pos, int index);
bool Com_sendPoint(HANDLE fd, Point3f p, float yaw, float pitch, float roll, Point3f target);
Point3d getIntersection(Point3d a1, Point3d b1, Point3d a2, Point3d b2);


int CameraTask()
{
	int width;
	int height;

	long t1, t2;

	int cam_count;
	int cam1_id = 0;
	int cam2_id = 1;
	int cam_trigger_align_cnt = 0;
	API_STATUS api_status;

	/* Init cameras */
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
	char cam1_serial[13] = { '2', '8', '9', '2', '0', '0', '3', '3', '2', '0', '1', '7','\0' };
	char cam2_serial[13] = { '2', '8', '5', '2', '0', '0', '3', '3', '2', '0', '1', '7','\0' };
	char serial1[13] = {'\0'};
	char serial2[13] = {'\0'};

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

	CameraInitFlag = true;

	int len = width * height;

	Mat cam1_frame, cam2_frame;
	Mat cam1_frame_tmp, cam2_frame_tmp;
	Mat bin_frame1, bin_frame2;
	vector<Point2f> cam1_point, cam2_point;

	/* data to return */
	Point3f object_center = { 0, 0, 0 };
	float object_yaw = 0;
	float object_pitch = 0;
	float object_roll = 0;

	Mat cam1_warpMat;
	Mat cam2_warpMat;

	Mat disp_frame_dist;
	Point3f cam1_pos, cam2_pos;

	cam1_frame.create(height, width, CV_8UC1);
	cam2_frame.create(height, width, CV_8UC1);
	bin_frame1.create(height, width, CV_8UC1);
	bin_frame2.create(height, width, CV_8UC1);

	FileStorage fs("CameraInfo.yml", FileStorage::READ);
	if (fs.isOpened() == false) {
		cout << "can't find file:cameraInfo.yml\n";
		return 0;
	}
	fs["camera1_warpMat"] >> cam1_warpMat;
	fs["camera2_warpMat"] >> cam2_warpMat;
	fs["camera1_position"] >> cam1_pos;
	fs["camera2_position"] >> cam2_pos;
	fs.release();

	//FileStorage fs_traj("trajectory.yml", FileStorage::WRITE);

	HANDLE hCom;
	hCom = Com_Open();
	if (hCom == NULL){
		cout << "can't open bluetooth com...\n";
		return 0;
	}
	CameraTriggerShot(cam1_id);
	CameraTriggerShot(cam2_id);
	while (1) {

		t1 = (double)getTickCount();
		api_status = CameraQueryImage(cam1_id, (unsigned char *)cam1_frame.data, &len, CAMERA_IMAGE_GRAY8);
		api_status = CameraQueryImage(cam2_id, (unsigned char *)cam2_frame.data, &len, CAMERA_IMAGE_GRAY8);
		t2 = (double)getTickCount();

		if (rand() % 10000 > 5000){
			CameraTriggerShot(cam1_id);
			CameraTriggerShot(cam2_id);
		}
		else{
			CameraTriggerShot(cam2_id);
			CameraTriggerShot(cam1_id);
		}

		if (api_status == API_OK) {

			//Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
			//morphologyEx(cam1_frame, cam1_frame_tmp, MORPH_ERODE, kernel);
			//kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			//   morphologyEx(cam2_frame, cam2_frame_tmp, MORPH_ERODE, kernel);

			cam1_frame_tmp = cam1_frame.clone();
			cam2_frame_tmp = cam2_frame.clone();

			bin_frame1 = cam1_frame_tmp.clone()>100;
			bin_frame2 = cam2_frame_tmp.clone()>100;

			vector<Point2f> point_on_img_1 = findTargetPoint(bin_frame1);
			vector<Point2f> point_on_img_2 = findTargetPoint(bin_frame2);

			//cout << "1:"<<point_on_img_1[0] << point_on_img_1[1] << point_on_img_1[2]<<endl;
			//cout <<"2:"<< point_on_img_2[0] << point_on_img_2[1]<<point_on_img_2[2] << endl<<endl;

			if ((point_on_img_1.size() == point_on_img_2.size()) && (point_on_img_1.size() == 3)){

				/* index the points */
				for (unsigned int i = 0; i < point_on_img_1.size(); i++){
					indexPoint(bin_frame1, point_on_img_1[i], i + 1);
					indexPoint(bin_frame2, point_on_img_2[i], i + 1);
				}

				vector<Point2f> point_warp2ground_1, point_warp2ground_2;
				/* project markers to the ground ,get their coordinates on the ground */
				perspectiveTransform(point_on_img_1, point_warp2ground_1, cam1_warpMat);
				perspectiveTransform(point_on_img_2, point_warp2ground_2, cam2_warpMat);

				/* convert from ground-image to ground-real axis ; image center is center of camera 1 */
				Point3f point_on_ground_1, point_on_ground_2;/* uint meter */
				vector<Point3d> marker;


				marker.clear();
				for (unsigned int i = 0; i < point_warp2ground_1.size(); i++){

					point_on_ground_1.x = point_warp2ground_1[i].x;
					point_on_ground_1.y = point_warp2ground_1[i].y;
					point_on_ground_1.z = 0;

					point_on_ground_2.x = point_warp2ground_2[i].x;
					point_on_ground_2.y = point_warp2ground_2[i].y;
					point_on_ground_2.z = 0;

					marker.push_back(getIntersection(cam1_pos, point_on_ground_1, cam2_pos, point_on_ground_2));
				}

				Point3d v1, v2, v3, m;
				Point3d n_v1, n_v2, n_v3;/* normalized vectors */

				v1 = marker[1] - marker[2];
				n_v1 = v1 / sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
				v2 = marker[0] - marker[2];
				n_v2 = v2 / sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);

				m = (v2.x*n_v1.x + v2.y*n_v1.y + v2.z*n_v1.z)*n_v1;

				v3 = v2 - m;
				n_v3 = v3 / sqrt(v3.x*v3.x + v3.y*v3.y + v3.z*v3.z);

				object_center = (marker[1] + marker[2]) / 2;

				/* get euler */
				object_yaw = atan2(v1.y, v1.x);
				object_pitch = asin(-n_v1.z);
				object_roll = asin(n_v3.z / cos(object_pitch));

				Com_sendPoint(hCom, object_center, object_yaw, object_pitch, object_roll, Kinect_HandPoistion);
			}
			else
				;//Com_sendPoint(hCom, object_center, object_yaw, object_pitch, object_roll);

		}

		/*display*/
		static double t = 0;
		static double fps;

		fps = 0.9 * fps + 0.1 * (getTickFrequency() / (double)(getTickCount() - t));
		t = (double)getTickCount();

		printf("[Pos:%8.3f,%8.3f,%8.3f], [Att:%8.3f,%8.3f,%8.3f,] %3.1f fps\r\n", object_center.x, object_center.y, object_center.z, object_yaw*57.3, object_pitch*57.3, object_roll*57.3, fps);

		//Mat disp_frame_dist;
		//putFPS2Image(cam1_frame);
		//cvtColor(cam1_frame, disp_frame_dist, CV_GRAY2BGR);   // display camera1 warp image
		//imshow(WINDOWS1_NAME, disp_frame_dist); /* 0.5ms */

		//cvtColor(cam2_frame, disp_frame_dist, CV_GRAY2BGR);   // display camera2 original image
		//imshow(WINDOWS2_NAME, disp_frame_dist); /* 0.5ms */

		//cvtColor(bin_frame1, disp_frame_dist, CV_GRAY2BGR);
		//imshow(WINDOWS3_NAME, disp_frame_dist); /* 0.5ms */

		//cvtColor(bin_frame2, disp_frame_dist, CV_GRAY2BGR);
		//imshow(WINDOWS4_NAME, disp_frame_dist); /* 0.5ms */
		waitKey(10);
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

/*
***********************************************
findTargetPoint()
***********************************************
*/
vector<Point2f> findTargetPoint(Mat& img)
{
	vector<vector<Point> >  contourts;
	vector<Vec4i> hierarchy;
	Mat image = img.clone();

	findContours(image, contourts, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point2f> center(contourts.size());
	vector<float>   radius(contourts.size());

	Point2f center_tmp;
	float radius_tmp;

	if ((contourts.size()>0) && (contourts.size()<6)) {

		/* store the center and radius */
		for (int index = 0; index >= 0; index = hierarchy[index][0]){
			minEnclosingCircle(contourts[index], center[index], radius[index]);
		}

		/* sort in radius decending order */
		for (unsigned int i = 0; i < contourts.size(); i++){
			for (unsigned int j = i + 1; j < contourts.size(); j++){
				/* exchange */
				if (radius[j]>radius[i]){
					center_tmp = center[i];
					radius_tmp = radius[i];
					center[i] = center[j];
					radius[i] = radius[j];
					center[j] = center_tmp;
					radius[j] = radius_tmp;
				}
			}
		}
	}
	else{
		center.clear();
		center.push_back(Point2f(-1, -1));
	}

	return center;
}


/*
***********************************************
indexPoint()
index a point in the picture
***********************************************
*/
void indexPoint(Mat& img, Point pos, int index)
{
	char str[10];
	circle(img, pos, 10, Scalar(255, 255, 255));
	sprintf(str, "%d", index);
	putText(img, str, pos + Point(0, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
}

/*
***********************************************
getDistToLine()
la:start point   lb:end point
***********************************************
*/
float getDistToLine(Point2f p_, Point2f la, Point2f lb)
{
	Point2f l; /* vector along the line */
	Point2f n_l; /* normalized vector of the line */
	Point2f p; /* vector of the point */

	p = p_ - la;
	l = la - lb;
	n_l = l / sqrt(l.x * l.x + l.y * l.y);

	return sqrt((p.x * p.x + p.y * p.y) - pow(fabs(p.x * n_l.x + p.y * n_l.y), 2));
}

/*
***********************************************
findSmallerThan()
return the value that smaller than the threshold
***********************************************
*/
vector<int> findSmallerThan(vector<double> x, double threshold)
{
	vector<int> index;
	index.clear();
	for (unsigned int i = 0; i < x.size(); i++)
	if (x[i] < threshold)
		index.push_back(i);
	return index;
}

/*
***********************************************
findMin()
return the index of the minimum data
***********************************************
*/
int findMin(vector<double> x)
{
	int min_index = 0;
	double min = x[0];

	if (x.size() == 1)
		min_index = 0;
	else
	for (unsigned int i = 1; i < x.size(); i++) {

		if (x[i] < min) {
			min = x[i];
			min_index = i;
		}
	}

	return min_index;
}

/*
***********************************************
getSSD()
***********************************************
*/
double getSSD(Mat& img1, Mat& img2)
{
	double result = 0;

	for (unsigned int i = 0; i < (unsigned int)img1.rows; i++)
	for (unsigned int j = 0; j < (unsigned int)img1.cols; j++)
		result += pow((img1.at<char>(i, j) - img2.at<char>(i, j)), 2);
	return result;
}


double constraint(double x, double lower, double upper)
{
	if (x>upper)
		return upper;
	else if (x < lower)
		return lower;
	else
		return x;
}



UINT8 checksum(UINT8 *data, UINT16 len)
{
	UINT32 sum = 0;
	UINT8 i = 0;

	for (; len > 1; len -= 2)
	{
		sum += *data++;
		if (sum & 0x80000000)
			sum = (sum & 0xffff) + (sum >> 16);
	}

	if (len == 1)
	{
		*(UINT8 *)(&i) = *(UINT8 *)data;
		sum += i;
	}

	while (sum >> 16)
		sum = (sum & 0xffff) + (sum >> 16);

	return ((sum == 0xffff) ? sum : (~sum));
}

typedef union{
	float f;
	BYTE b[4];
}F2B_T;

typedef union{
	WORD w;
	BYTE b[2];
}W2B_T;

#define COM_BUF_LENGTH 21
bool Com_sendPoint(HANDLE fd, Point3f p, float yaw, float pitch, float roll, Point3f target)
{
	BYTE data[COM_BUF_LENGTH];
	W2B_T w2b_x, w2b_y, w2b_z, w2b_yaw, w2b_pitch, w2b_roll;
	W2B_T w2b_targ_x, w2b_targ_y, w2b_targ_z;

	w2b_x.w = 1000 * constraint(p.x, -10, 10) + 0.5;
	w2b_y.w = 1000 * constraint(p.y, -10, 10) + 0.5;
	w2b_z.w = 1000 * constraint(p.z, -10, 10) + 0.5;

	w2b_yaw.w = 1000 * constraint(yaw, -2 * PI, 2 * PI) + 0.5;
	w2b_pitch.w = 1000 * constraint(pitch, -2 * PI, 2 * PI) + 0.5;
	w2b_roll.w = 1000 * constraint(roll, -2 * PI, 2 * PI) + 0.5;

	w2b_targ_x.w = 1000 * constraint(target.x, -0.5, 0.5) + 0.5;
	w2b_targ_y.w = 1000 * constraint(target.y, -0.5, 0.5) + 0.5;
	w2b_targ_z.w = 1000 * constraint(target.z, 0, 1) + 0.5;

	data[0] = '$';
	data[1] = w2b_x.b[0];
	data[2] = w2b_x.b[1];

	data[3] = w2b_y.b[0];
	data[4] = w2b_y.b[1];

	data[5] = w2b_z.b[0];
	data[6] = w2b_z.b[1];

	data[7] = w2b_yaw.b[0];
	data[8] = w2b_yaw.b[1];

	data[9] = w2b_pitch.b[0];
	data[10] = w2b_pitch.b[1];

	data[11] = w2b_roll.b[0];
	data[12] = w2b_roll.b[1];

	data[13] = w2b_targ_x.b[0];
	data[14] = w2b_targ_x.b[1];

	data[15] = w2b_targ_y.b[0];
	data[16] = w2b_targ_y.b[1];

	data[17] = w2b_targ_z.b[0];
	data[18] = w2b_targ_z.b[1];

	data[19] = checksum(data + 1, COM_BUF_LENGTH - 3);
	data[20] = '*';

	DWORD dwExpectSend = COM_BUF_LENGTH, dwRealSend;
	bool state = Com_Send(fd, data, dwExpectSend, &dwRealSend);
	//cout << "send succeed! length: " << dwRealSend << endl;
	return state;
}



void nearest_point_of_skew_line(const double p1[3], const double p1_[3], const
	double p2[3], const double p2_[3], double p[3])
{
	double d2[3];
	int i;
	double n_idx_0;
	double n_idx_1;
	double n_idx_2;
	double n1[3];
	double n2[3];
	double y;
	double d0;
	double b_y;
	for (i = 0; i < 3; i++) {
		p[i] = p1_[i] - p1[i];
		d2[i] = p2_[i] - p2[i];
	}

	n_idx_0 = p[1] * d2[2] - p[2] * d2[1];
	n_idx_1 = p[2] * d2[0] - p[0] * d2[2];
	n_idx_2 = p[0] * d2[1] - p[1] * d2[0];
	n1[0] = p[1] * n_idx_2 - p[2] * n_idx_1;
	n1[1] = p[2] * n_idx_0 - p[0] * n_idx_2;
	n1[2] = p[0] * n_idx_1 - p[1] * n_idx_0;
	n2[0] = d2[1] * n_idx_2 - d2[2] * n_idx_1;
	n2[1] = d2[2] * n_idx_0 - d2[0] * n_idx_2;
	n2[2] = d2[0] * n_idx_1 - d2[1] * n_idx_0;
	y = 0.0;
	d0 = 0.0;
	for (i = 0; i < 3; i++) {
		y += (p2[i] - p1[i]) * n2[i];
		d0 += p[i] * n2[i];
	}

	b_y = y / d0;
	y = 0.0;
	d0 = 0.0;
	for (i = 0; i < 3; i++) {
		y += (p1[i] - p2[i]) * n1[i];
		d0 += d2[i] * n1[i];
	}

	y /= d0;
	for (i = 0; i < 3; i++) {
		p[i] = ((p1[i] + b_y * p[i]) + (p2[i] + y * d2[i])) / 2.0;
	}
}

Point3d getIntersection(Point3d a1, Point3d b1, Point3d a2, Point3d b2)
{
	double p1[3];
	double p1_[3];
	double p2[3];
	double p2_[3];
	double c[3];

	Point3d result;

	p1[0] = a1.x; p1[1] = a1.y; p1[2] = a1.z;
	p1_[0] = b1.x; p1_[1] = b1.y; p1_[2] = b1.z;

	p2[0] = a2.x; p2[1] = a2.y; p2[2] = a2.z;
	p2_[0] = b2.x; p2_[1] = b2.y; p2_[2] = b2.z;

	nearest_point_of_skew_line(p1, p1_, p2, p2_,c);

	result = Point3d(c[0], c[1], c[2]);

	return result;
}

