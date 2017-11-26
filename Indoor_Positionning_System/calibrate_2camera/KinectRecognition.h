#ifndef __KINECTRECOGNITION_H_
#define __KINECTRECOGNITION_H_

#define VECTOR Point3f
#define PI	3.1415926
#define RAD2DEG (180/PI)
#define DEG2RAD (PI/180)

#define SPACE_LIMIT_MAX_X	0.5	
#define SPACE_LIMIT_MIN_X	-0.5
#define SPACE_LIMIT_MAX_Y	0.5
#define SPACE_LIMIT_MIN_Y	-0.6
#define SPACE_LIMIT_MAX_Z	0.5
#define SPACE_LIMIT_MIN_Z	0.1

#include "SkeletonStream.h"
//using namespace cv;

class KinectRecognition: public SkeletonStream{
private :
	int body_center;
	const float scalar_x = 1.8;
	const float scalar_y = 2.0;
	const float scalar_z = 1.0;
public:
	KinectRecognition();
	bool Is_PutDownPose();
	Point3f convertAxisToBody(int skeletom_num);
	Point3f calculateTargetPosition ();
};

#endif