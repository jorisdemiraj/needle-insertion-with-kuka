#include "DeviceState.h"

using namespace std;
using namespace Eigen;

DeviceState::DeviceState()
{
	pos.setZero();
	vel.setZero();
	rot.setZero();
	T.setIdentity();
}


DeviceState::~DeviceState()
{
}

float* DeviceState::eigen2SimFloat(const Eigen::Vector3f eigen)
{
	float temp[3];
	temp[0] = eigen(0);
	temp[1] = eigen(1);
	temp[2] = eigen(2);

	return temp;
}

float** DeviceState::eigen2SimMatrix(const Eigen::Matrix3f eigen)
{
	float** temp;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			temp[i][j] = eigen(i, j);
		}
	}
	return temp;
}

void DeviceState::print(void)
{
	cout << "Position:" << endl;
	cout << pos << endl << endl;
	cout << "Velocity:" << endl;
	cout << vel << endl << endl;
	cout << "Rotation matrix:" << endl;
	cout << rot << endl << endl;
}

float* DeviceState::getEulerAngles(void)
{
	float euler_angles[3];
	euler_angles[0] = atan2(rot(2, 1), rot(2, 2));
	euler_angles[1] = asin(rot(2, 0));
	euler_angles[2] = -atan2(rot(1, 0), rot(0, 0));

	return euler_angles;
}

float* DeviceState::getSimTransformMatrix_Original(float resolution)
{
	// Original orientation of the Device
	float transform_matrix[12];

	transform_matrix[0] = rot(0, 0);
	transform_matrix[1] = rot(0, 1);
	transform_matrix[2] = rot(0, 2);
	transform_matrix[3] = resolution * pos(0);

	transform_matrix[4] = rot(1, 0);
	transform_matrix[5] = rot(1, 1);
	transform_matrix[6] = rot(1, 2);
	transform_matrix[7] = resolution * pos(1);

	transform_matrix[8]		= rot(2, 0);
	transform_matrix[9]		= rot(2, 1);
	transform_matrix[10]	= rot(2, 2);
	transform_matrix[11]	= resolution * pos(2);

	return transform_matrix;
}

float* DeviceState::getSimTransformMatrix(float resolution)
{
	float transform_matrix[12];

	transform_matrix[0]		= rot(2, 0);
	transform_matrix[1]		= rot(2, 1);
	transform_matrix[2]		= rot(2, 2);
	transform_matrix[3]		= resolution * pos(0);

	transform_matrix[4]		= rot(1, 0);
	transform_matrix[5]		= rot(1, 1);
	transform_matrix[6]		= rot(1, 2);
	transform_matrix[7]		= resolution * pos(1);

	transform_matrix[8]		= rot(0, 0);
	transform_matrix[9]		= rot(0, 1);
	transform_matrix[10]	= rot(0, 2);
	transform_matrix[11]	= resolution * pos(2);

	return transform_matrix;
}