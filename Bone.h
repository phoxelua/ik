#pragma once
#include "includes.h"

class Bone
{
public:
	float length, currTheta, nextTheta, currPhi, nextPhi; //angle in radians
	Eigen::Vector3d currPos, nextPos;

	Bone(){};
	Bone(float l);
};