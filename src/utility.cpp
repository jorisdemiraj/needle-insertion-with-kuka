#include "utility.h"


void printSimTransform(const float* sim_T)
{
	for (int i = 0; i < 12; i++)
	{
		if (i % 4 == 0 && i != 0)
			cout << "\t" << endl;
		cout << sim_T[i] << "\t";
	}
	cout << endl;
}


void printSimMatrix(const float* sim_M, const int r, const int c)
{
	for (int i = 0; i < r*c; i++)
	{
		if (i % c == 0 && i != 0)
			cout << "\t" << endl;
		cout << sim_M[i] << "\t";
	}
	cout << endl;
}

// This function takes a Eigen Matrix4f as input and returns a 
// float[12] array (since the last row [0 0 0 1] is not taken 
// in consideration).
void eigen2SimTransf(const Matrix4f& in, float* out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			out[4 * i + j] = in(i, j);
		}
	}
}

void sim2EigenTransf(const float* in, Matrix4f& out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			out(i, j) = in[4 * i + j];
		}
	}
	out.block<1, 4>(3, 0) << 0, 0, 0, 1;
}

void eigen2SimRot(const Matrix3f& in, float* out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out[3 * i + j] = in(i, j);
		}
	}
}

void sim2EigenRot(const float* in, Matrix3f& out)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out(i, j) = in[3 * i + j];
		}
	}
}

void eigen2SimVec3f(const Vector3f& in, float* out) 
{
	out[0] = in(0);
	out[1] = in(1);
	out[2] = in(2);
}
void sim2EigenVec3f(const float* in, Vector3f& out)
{
	out(0) = in[0];
	out(1) = in[1];
	out(2) = in[2];
}


void eigen2SimVec6f(const Vector6f& in, float* out)
{
	out[0] = in(0);
	out[1] = in(1);
	out[2] = in(2);
	out[3] = in(3);
	out[4] = in(4);
	out[5] = in(5);
}
void sim2EigenVec6f(const float* in, Vector6f& out)
{
	out(0) = in[0];
	out(1) = in[1];
	out(2) = in[2];
	out(3) = in[3];
	out(4) = in[4];
	out(5) = in[5];
}

void eigen2SimVec7f(const Vector7f& in, float* out)
{
	out[0] = in(0);
	out[1] = in(1);
	out[2] = in(2);
	out[3] = in(3);
	out[4] = in(4);
	out[5] = in(5);
	out[6] = in(6);
}
void sim2EigenVec7f(const float* in, Vector7f& out)
{
	out(0) = in[0];
	out(1) = in[1];
	out(2) = in[2];
	out(3) = in[3];
	out(4) = in[4];
	out(5) = in[5];
	out(6) = in[6];
}









void simComposeTransform(const float* in_rot, const float* in_pos, float* out_T)
{
	Matrix4f temp_T;
	Matrix3f temp_R;
	Vector3f temp_P;

	sim2EigenRot(in_rot, temp_R);
	sim2EigenVec3f(in_pos, temp_P);

	temp_T.setIdentity();
	temp_T.block<3, 3>(0, 0) = temp_R;
	temp_T.block<3, 1>(0, 3) = temp_P;

	eigen2SimTransf(temp_T, out_T);
}

void simDecomposeTransform(const float* in_T,  float* out_rot, float* out_pos)
{
	Matrix4f temp_T;
	Matrix3f temp_R;
	Vector3f temp_P;

	sim2EigenTransf(in_T, temp_T);
	temp_R = temp_T.block<3, 3>(0, 0);
	temp_P = temp_T.block<3, 1>(0, 3);

	eigen2SimRot(temp_R, out_rot);
	eigen2SimVec3f(temp_P, out_pos);
}

void simMultiplyVec3fByScalar(float* v, float k)
{
	v[0] = v[0] * k;
	v[1] = v[1] * k;
	v[2] = v[2] * k;
}

VectorXf deg2radVec(VectorXf a)
{
	VectorXf b = a;
	int size = a.size();
	for (int i = 0; i < size; i++)
	{
		b(i) = deg2rad(a(i));
	}
	return b;
}

float unboundAngle(const float prev_alpha, float curr_alpha)
{
	while ((curr_alpha - prev_alpha) < -6.10865)
		curr_alpha = curr_alpha + 2.0f * (float)M_PI;
	while ((curr_alpha - prev_alpha) > 6.10865)
		curr_alpha = curr_alpha - 2.0f * (float)M_PI;
	return curr_alpha;
}












// --------------------------------------------------- //
// ----------------- COLORE CONSOLE ------------------ //
//  Antonio Cifonelli, tutti i diritti sono riservati. //
// --------------------------------------------------- //

int red()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_INTENSITY);
	return 0;
}

int green()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_INTENSITY);
	return 0;
}

int blue()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_BLUE | FOREGROUND_INTENSITY);
	return 0;
}

int yellow()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
	return 0;
}

int magenta()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
	return 0;
}

int cyan()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
	return 0;
}

int reset()
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hStdout == INVALID_HANDLE_VALUE)
	{
		cout << "Error while getting input handle" << endl;
		return EXIT_FAILURE;
	}
	SetConsoleTextAttribute(hStdout, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
	return 0;
}