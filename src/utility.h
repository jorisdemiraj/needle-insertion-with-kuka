/* ----------------------------------------------------------- */
/* ----------------------------------------------------------- */
/* -------------------- CONVERSION SOURCE ---------------------*/
/* -------- Conversions from Eigen to simFloat/float* ---------*/
/* ----------------------------------------------------------- */
/* ----------------------------------------------------------- */

//! TUTTE TESTATE.
#pragma once

#include <signal.h>

#include <iostream>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "luaFunctionData.h"
#include "v_repLib.h"
#include "chai3d.h"

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 7> Matrix6_7f;
typedef Eigen::Matrix<float, 7, 6> Matrix7_6f;
typedef Eigen::Matrix<float, 7, 7> Matrix7f;


using namespace std;
using namespace Eigen;


int red();
int green();
int blue();
int yellow();
int magenta();
int cyan();
int reset();



// This function prints a float[12] array, in matrix style.
// TODO: fix last print
void printSimTransform(const float* sim_T);
// This function prints a generic float[r*c] array, in matrix style.
// TODO: fix last print
void printSimMatrix(const float* sim_M, const int r, const int c);


void eigen2SimTransf(const Matrix4f& in, float* out);
void sim2EigenTransf(const float* in, Matrix4f& out);

void eigen2SimRot(const Matrix3f& in, float* out);
void sim2EigenRot(const float* in, Matrix3f& out);

void eigen2SimVec3f(const Vector3f& in, float* out);
void sim2EigenVec3f(const float* in, Vector3f& out);

void eigen2SimVec6f(const Vector6f& in, float* out);
void sim2EigenVec6f(const float* in, Vector6f& out);

void eigen2SimVec7f(const Vector7f& in, float* out);
void sim2EigenVec7f(const float* in, Vector7f& out);

// This function given a R (3by3) and a P (3by1), builds a T (3by4).
void simComposeTransform(const float* in_rot, const float* in_pos, float* out_T);
// This function decompose a simFloat T into its rotational part (R 3by3)
// and its traslational one (P 3by1).
void simDecomposeTransform(const float* in_T, float* out_rot, float* out_pos);

float unboundAngle(const float prev_alpha, float curr_alpha);

void simMultiplyVec3fByScalar(float* v, float k);
VectorXf deg2radVec(VectorXf a);

inline float rad2deg(float b) { return (float)((b / M_PI * 180.0f)); };
inline float deg2rad(float a) { return (float)((a / 180.0f * M_PI)); };

inline void rotMatrix(Matrix2f& M, const float angle) { M << cos(angle), -sin(angle), sin(angle), cos(angle); };