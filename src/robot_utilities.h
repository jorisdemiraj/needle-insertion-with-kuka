#pragma once
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "luaFunctionData.h"
#include "v_repLib.h"
#include "chai3d.h"
#include "utility.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 7> Matrix6_7f;
typedef Eigen::Matrix<float, 7, 6> Matrix7_6f;
typedef Eigen::Matrix<float, 7, 7> Matrix7f;
using namespace Eigen;

Matrix4f linkCoordTransform(float a, float alpha, float d, float theta);
Matrix4f cinematicaDiretta(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord);
Matrix4f cinematicaDiretta(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord, int i_pos);

Matrix6_7f jacobianoGeometrico(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord);
void setDHParameter(Vector7f& a_vec, Vector7f& alpha_vec, Vector7f& d_vec);
Matrix6_7f LWRGeometricJacobian(const Vector7f lwr_joint_q);

Matrix7_6f pinv(const Matrix6_7f J);
Matrix<float, 7, 3> pinv(const Matrix<float, 3, 7> J);
Vector7f computeNSVel(Vector6f r_dot, Matrix6_7f J);

//void computeNullSpaceVelocity(Vector7f& config_q_dot,
//	const Vector6f& des_vel, const Matrix4f& des_T, const Matrix4f& curr_T,
//	const Matrix6_7f& J, const Matrix6f& Kp);

void computeNullSpaceVelocity(Vector7f& config_q_dot,
	Vector7f& q_0_dot,
	const Vector6f& des_vel,
	const Matrix4f& prev_des_T, const Matrix4f& prev_curr_T,
	const Matrix4f& des_T, const Matrix4f& curr_T,
	const Matrix6_7f& J, const Matrix6f& Kp);

void computeDLSVelocity(Vector7f& config_q_dot,
	const Vector6f& des_vel,
	const Matrix4f& prev_des_T, const Matrix4f& prev_curr_T,
	const Matrix4f& des_T, const Matrix4f& curr_T,
	const Matrix6_7f& J, const Matrix6f& Kp);