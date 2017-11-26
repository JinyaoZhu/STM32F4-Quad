#include "KinectRecognition.h"
using namespace std;

KinectRecognition::KinectRecognition()
{
	body_center = NUI_SKELETON_POSITION_SHOULDER_RIGHT; 
}

Point3f KinectRecognition::convertAxisToBody(int skeletom_num)
{
	return (getSkeletonPositon(skeletom_num) - getSkeletonPositon(body_center));
}

Point3f KinectRecognition::calculateTargetPosition()
{
	Point3f target_p;
	if (Is_PutDownPose() == false){
		Point3f body_p = convertAxisToBody(NUI_SKELETON_POSITION_HAND_RIGHT);
		target_p.x = -scalar_x*body_p.x;
		target_p.y = -scalar_y*(-body_p.z - 0.3);			// 0.3m is the length of a half of my arm.
		target_p.z = scalar_z*(body_p.y + 0.4);
	}

	else
		target_p = Point3f(0, 0, 0);

	target_p.x = target_p.x < SPACE_LIMIT_MAX_X ? target_p.x : SPACE_LIMIT_MAX_X;
	target_p.x = target_p.x > SPACE_LIMIT_MIN_X ? target_p.x : SPACE_LIMIT_MIN_X;
	target_p.y = target_p.y < SPACE_LIMIT_MAX_Y ? target_p.y : SPACE_LIMIT_MAX_Y;
	target_p.y = target_p.y > SPACE_LIMIT_MIN_Y ? target_p.y : SPACE_LIMIT_MIN_Y;
	target_p.z = target_p.z < SPACE_LIMIT_MAX_Z ? target_p.z : SPACE_LIMIT_MAX_Z;
	target_p.z = target_p.z > SPACE_LIMIT_MIN_Z ? target_p.z : SPACE_LIMIT_MIN_Z;

	return target_p;
}

bool KinectRecognition::Is_PutDownPose()
{
	Point3f p1 = getSkeletonPositon(NUI_SKELETON_POSITION_SHOULDER_RIGHT);
	Point3f p2 = getSkeletonPositon(NUI_SKELETON_POSITION_ELBOW_RIGHT);
	Point3f p3 = getSkeletonPositon(NUI_SKELETON_POSITION_WRIST_RIGHT);

	VECTOR a = p2 - p1;
	VECTOR b = p3 - p1;
	VECTOR c = Point3f(0, -1, 0);

	float cos_ac = -a.y / sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
	float cos_bc = -b.y / sqrt(b.x*b.x + b.y*b.y + b.z*b.z);

	return ((RAD2DEG*acos(cos_ac) < 35) && (RAD2DEG*acos(cos_bc) < 35)) ? true : false;
}