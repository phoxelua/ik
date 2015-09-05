#pragma once
#include "includes.h"
#include "Bone.h"

class Kinematics
{
public:
	float step, epsilon;
	Kinematics(float s, float e);

	Eigen::Vector3d solveFKTest(std::vector<Bone> & bones, int index, float dTheta, float dPhi);
	Eigen::Vector3d solveFK(std::vector<Bone> & bones, int index, float dTheta, float dPhi);
	Eigen::Vector3d solveFKReset(std::vector<Bone> & bones, float theta, float phi);

	Eigen::MatrixXd jacobian(std::vector<Bone> & bones, float step);
	Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd & jacobian);

	void solveIK(std::vector<Bone> & bones, Eigen::Vector3d goalPos);
};
