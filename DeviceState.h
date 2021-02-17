#pragma once
#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <math.h>
class DeviceState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	DeviceState();
	~DeviceState();
	
	inline float* getSimPos(void) { return eigen2SimFloat(pos); };
	inline float* getSimVel(void) { return eigen2SimFloat(vel); };
	inline float** getSimRot(void) { return eigen2SimMatrix(rot); };

	float* getEulerAngles(void);
	float* getSimTransformMatrix_Original(float resolution = 1.0);
	float* getSimTransformMatrix(float resolution = 1.0);

	void print(void);

	Eigen::Vector3f pos;
	Eigen::Vector3f vel;
	Eigen::Matrix3f rot;
	Eigen::Matrix4f T;

protected:
	const int v_buffer_size = 4;
	float* eigen2SimFloat(const Eigen::Vector3f eigen);
	float** eigen2SimMatrix(const Eigen::Matrix3f eigen);

};

