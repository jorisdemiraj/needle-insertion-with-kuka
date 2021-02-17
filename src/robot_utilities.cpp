#include "robot_utilities.h"

using namespace Eigen;

Matrix4f linkCoordTransform(float a, float alpha, float d, float theta)
{
	Matrix4f result;

	result(0, 0) = cos(theta);
	result(0, 1) = -sin(theta)*cos(alpha);
	result(0, 2) = sin(theta)*sin(alpha);
	result(0, 3) = a*cos(theta);

	result(1, 0) = sin(theta);
	result(1, 1) = cos(theta)*cos(alpha);
	result(1, 2) = -cos(theta)*sin(alpha);
	result(1, 3) = a*sin(theta);

	result(2, 0) = 0;
	result(2, 1) = sin(alpha);
	result(2, 2) = cos(alpha);
	result(2, 3) = d;

	result(3, 0) = 0;
	result(3, 1) = 0;
	result(3, 2) = 0;
	result(3, 3) = 1;
	return result;
}



//funzione che restituisce la cinematica diretta del manipolatore espressa con una matrice di trasformazione
//omogenea, in funzione del valore delle variabili di giunto, dei parametri cinematici e dei parametri
//della notazione di Denavit-Hartemberg
Matrix4f cinematicaDiretta(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord)
{
	Matrix4f result;

	//inizializzo la matrice con la matrice identità
	result.setIdentity();

	int joint_number = joint_coord.size();

	//questo vettore conterrà tutte le matrici di trasformazione di coordinate tra le terne associate a due link consecutivi.
	//Il primo elemento (in posizione 0) si riferisce alla matrice A^0_1, il secondo alla matrice A^1_2, e così via
	vector<Matrix4f> transform_matrices;

	for (int i = 0; i<joint_number; i++)
	{ //aggiunge elementi in coda al vettore
		transform_matrices.push_back(linkCoordTransform(a_vec(i), alpha_vec(i), d_vec(i), joint_coord(i)));
	}

	//effettuo il prodotto delle matrici di trasformazione per calcolare la cinematica diretta
	for (int i = 0; i<joint_number; i++)
	{
		Matrix4f temp;
		temp = result * transform_matrices.at(i);
		result = temp;
	}

	return result;
}



//funzione che restituisce la cinematica diretta del manipolatore CALCOLATA FINO AL GIUNTO i incluso (Ai-i,i inclusa),
//espressa con una matrice di trasformazione omogenea, in funzione del valore delle variabili di giunto,
//dei parametri cinematici e dei parametri della notazione di Denavit-Hartemberg
Matrix4f cinematicaDiretta(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord, int i_pos)
{
	// VEDERE PERCHE NON FUNZIONA
	Matrix4f result;

	//inizializzo la matrice con la matrice identità
	result.setIdentity();

	//questo vettore conterrà tutte le matrici di trasformazione di coordinate tra le terne associate a due link consecutivi.
	//Il primo elemento (in posizione 0) si riferisce alla matrice A^0_1, il secondo alla matrice A^1_2, e così via fino a Ai-1,i
	Matrix4f transform_matrices[7];
	for (int i = 0; i<i_pos; i++)
	{ //aggiunge elementi in coda al vettore
		transform_matrices[i] = (linkCoordTransform(a_vec(i), alpha_vec(i), d_vec(i), joint_coord(i)));
	}
	//effettuo il prodotto delle matrici di trasformazione per calcolare la cinematica diretta
	for (int i = 0; i<i_pos; i++)
	{
		result = result * transform_matrices[i];
	}
	return result;
}


//funzione che restituisce lo Jacobiano geometrico
Matrix6_7f jacobianoGeometrico(Vector7f a_vec, Vector7f alpha_vec, Vector7f d_vec, Vector7f joint_coord)
{/*lo Jacobiano geometrico per il Kuka LWR è una matrice 6x7 ed assume la seguente espressione:

 | z0 x (p-p0)  z1 x (p-p1)  z2 x (p-p2)  z3 x (p-p3)  z4 x (p-p4)  z5 x (p-p5)  z6 x (p-p6)  |
 J(q)=|    z0           z1           z2           z3           z4           z5           z6        |

 dove
 z0 = [0 0 1]^T
 p sono i primi 3 elementi della quarta colonna della matrice che esprime la cinematica diretta
 pi solo le prime 3 componenti dell'ultima colonna di  A0,1 * A1,2 * ... Ai-1,i
 Aj-1,j è la matrice di trasformazione in coordinate omogenee associata alla coppia di terne j-1, j
 zi = R0,1 * R1,2 * ... Ri-1,i * z0

 */
	Vector3f z0(0, 0, 1);
	//calcolo delle matrici di trasformazione
	Matrix4f A_0_1 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 1);
	Matrix4f A_0_2 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 2);
	Matrix4f A_0_3 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 3);
	Matrix4f A_0_4 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 4);
	Matrix4f A_0_5 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 5);
	Matrix4f A_0_6 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 6);
	Matrix4f A_0_7 = cinematicaDiretta(a_vec, alpha_vec, d_vec, joint_coord, 7);  //cinematica diretta completa

	Vector3f p = A_0_7.block<3, 1>(0, 3); // EE

	Vector3f p_1 = A_0_1.block<3, 1>(0, 3);
	Vector3f p_2 = A_0_2.block<3, 1>(0, 3);
	Vector3f p_3 = A_0_3.block<3, 1>(0, 3);
	Vector3f p_4 = A_0_4.block<3, 1>(0, 3);
	Vector3f p_5 = A_0_5.block<3, 1>(0, 3);
	Vector3f p_6 = A_0_6.block<3, 1>(0, 3);

	Vector3f z_1 = A_0_1.block<3, 3>(0, 0) * z0;
	Vector3f z_2 = A_0_2.block<3, 3>(0, 0) * z0;
	Vector3f z_3 = A_0_3.block<3, 3>(0, 0) * z0;
	Vector3f z_4 = A_0_4.block<3, 3>(0, 0) * z0;
	Vector3f z_5 = A_0_5.block<3, 3>(0, 0) * z0;
	Vector3f z_6 = A_0_6.block<3, 3>(0, 0) * z0;

	Matrix6_7f result;  //lo Jacobiano da restituire

							//blocco Jp1
	result.block<3, 1>(0, 0) = z0.cross(p);
	//blocco Jp2
	result.block<3, 1>(0, 1) = z_1.cross(p - p_1);
	//blocco Jp3
	result.block<3, 1>(0, 2) = z_2.cross(p - p_2);
	//blocco Jp4
	result.block<3, 1>(0, 3) = z_3.cross(p - p_3);
	//blocco Jp5
	result.block<3, 1>(0, 4) = z_4.cross(p - p_4);
	//blocco Jp6
	result.block<3, 1>(0, 5) = z_5.cross(p - p_5);
	//blocco Jp7
	result.block<3, 1>(0, 6) = z_6.cross(p - p_6);

	//blocco Jo1
	result.block<3, 1>(3, 0) = z0;
	//blocco Jo2
	result.block<3, 1>(3, 1) = z_1;
	//blocco Jo3
	result.block<3, 1>(3, 2) = z_2;
	//blocco Jo4
	result.block<3, 1>(3, 3) = z_3;
	//blocco Jo5
	result.block<3, 1>(3, 4) = z_4;
	//blocco Jo6
	result.block<3, 1>(3, 5) = z_5;
	//blocco Jo7
	result.block<3, 1>(3, 6) = z_6;

	return result;
}


//funzione che imposta i parametri di Denavit Hartenberg per il Kuka LWR
void setDHParameter(Vector7f& a_vec, Vector7f& alpha_vec, Vector7f& d_vec)
{ 	//imposto i parametri cinematici del robot
	float lung_0 = 115.0f / 1000.0f;
	float lung_1 = 200.0f / 1000.0f;
	float lung_2 = 200.0f / 1000.0f;
	float lung_3 = 200.0f / 1000.0f;
	float lung_4 = 200.0f / 1000.0f;
	float lung_5 = 190.0f / 1000.0f;
	float lung_7 = 112.0f / 1000.0f;
	float offset = 150.0f / 1000.0f;

	//inizializzazione dei parametri cinematici e dei parametri della notazione di Denavit-Hartenbger
	a_vec.setZero();

	alpha_vec(0) = (float)M_PI / 2;
	alpha_vec(1) = (float)-M_PI / 2;
	alpha_vec(2) = (float)M_PI / 2;
	alpha_vec(3) = (float)-M_PI / 2;
	alpha_vec(4) = (float)M_PI / 2;
	alpha_vec(5) = (float)-M_PI / 2;
	alpha_vec(6) = (float)0.0f;

	d_vec(0) = lung_0 + lung_1;
	d_vec(1) = 0;
	d_vec(2) = lung_2 + lung_3;
	d_vec(3) = 0;
	d_vec(4) = lung_4 + lung_5;
	d_vec(5) = 0;
	d_vec(6) = lung_7 + offset;

}

Matrix6_7f LWRGeometricJacobian(const Vector7f lwr_joint_q)
{ //strutture dati per i parametri di Denavit Hartenberg
	Vector7f a_vec;
	Vector7f alpha_vec;
	Vector7f d_vec;
	Vector7f joint_coord;

	for (int i = 0; i < 7; i++)
		joint_coord(i) = lwr_joint_q(i);

	setDHParameter(a_vec, alpha_vec, d_vec);

	Matrix6_7f A = jacobianoGeometrico(a_vec, alpha_vec, d_vec, joint_coord);
	return A;
}





void computeNullSpaceVelocity(Vector7f& config_q_dot,
	Vector7f& q_0_dot,
	const Vector6f& des_vel, 
	const Matrix4f& prev_des_T, const Matrix4f& prev_curr_T,
	const Matrix4f& des_T, const Matrix4f& curr_T,
	const Matrix6_7f& J, const Matrix6f& Kp)
{
	Vector3f error_pos, error_angle;
	Vector6f total_error;
	float sim_error_angle[3];

	Vector7f range_space_velocities, null_space_velocities;
	Matrix7f I;
	I.setIdentity();
	Vector6f r_dot;

	Matrix7_6f pinv_J = pinv(J);

	//! NULL PROJECTION
	Vector7f auxiliary_vector;
	for (int j = 0; j<7; j++)
		auxiliary_vector(j) = (float)rand() / (float)RAND_MAX;
	auxiliary_vector.setZero();

	//! POSITION ERROR
	error_pos = des_T.block<3, 1>(0, 3) - curr_T.block<3, 1>(0, 3);

	//! ORIENTATION ERROR
	Matrix4f angular_error_T;
	float sim_angular_error_T[12];

	Vector3f desired_angle, current_angle;
	Vector3f prev_desired_angle, prev_current_angle;
	float sim_des_angle[3];
	float sim_curr_angle[3];
	float sim_prev_des_angle[3];
	float sim_prev_curr_angle[3];
	float sim_prev_des_T[12];
	float sim_prev_curr_T[12];
	float sim_des_T[12];
	float sim_curr_T[12];
	Matrix3f error_R;

	Matrix3f tmp_1 = curr_T.block<3, 3>(0, 0).transpose();
	Matrix3f temp_R = tmp_1 * des_T.block<3, 3>(0, 0);

	angular_error_T.setZero();
	angular_error_T.block<3, 3>(0, 0) = temp_R;
	eigen2SimTransf(angular_error_T, sim_angular_error_T);

	simGetEulerAnglesFromMatrix(sim_angular_error_T, sim_error_angle);
	sim2EigenVec3f(sim_error_angle, error_angle);

	//! Unbound the error
	eigen2SimTransf(des_T, sim_des_T);
	eigen2SimTransf(curr_T, sim_curr_T);
	eigen2SimTransf(prev_des_T, sim_prev_des_T);
	eigen2SimTransf(prev_curr_T, sim_prev_curr_T);

	simGetEulerAnglesFromMatrix(sim_des_T, sim_des_angle);
	simGetEulerAnglesFromMatrix(sim_curr_T, sim_curr_angle);
	simGetEulerAnglesFromMatrix(sim_prev_des_T, sim_prev_des_angle);
	simGetEulerAnglesFromMatrix(sim_prev_curr_T, sim_prev_curr_angle);

	sim2EigenVec3f(sim_curr_angle, current_angle);
	sim2EigenVec3f(sim_des_angle, desired_angle);
	sim2EigenVec3f(sim_prev_curr_angle, prev_current_angle);
	sim2EigenVec3f(sim_prev_des_angle, prev_desired_angle);

	for (int i = 0; i < 3; i++)
	{
		current_angle(i) = unboundAngle(prev_current_angle(i), current_angle(i));
		desired_angle(i) = unboundAngle(prev_desired_angle(i), desired_angle(i));
	}

	//! Calculating T_XYZ matrix
	Matrix3f T_xyz;
	T_xyz <<	1,	0,						sin(current_angle(1)),
				0,	cos(current_angle(0)),	-cos(current_angle(1))*sin(current_angle(0)),
				0,	sin(current_angle(0)),	cos(current_angle(0))*cos(current_angle(1));

	error_angle = T_xyz * error_angle;

	//error_angle.setZero();
	total_error << error_pos, error_angle;

	//! R_DOT
	Vector6f temp_dv;
	temp_dv.setZero();
	temp_dv.block<3, 1>(0, 0) = des_vel.block<3, 1>(0, 0);

	r_dot = des_vel + Kp * total_error;
	//r_dot = temp_dv + Kp * total_error;

	//! Composing the final q_dot that will be imposed.
	range_space_velocities = pinv_J * r_dot;
	null_space_velocities = (I - pinv_J * J) * q_0_dot;

	config_q_dot = range_space_velocities + null_space_velocities;
}

Matrix7_6f pinv(const Matrix6_7f J)
{
	Matrix7_6f pinv_J;
	Matrix7_6f J_T;
	J_T = J.transpose();
	Matrix6f temp = J * J_T;
	temp = temp.inverse();

	pinv_J = J_T * temp;
	return pinv_J;
}

Matrix<float,7,3> pinv(const Matrix<float, 3, 7> J)
{
	Matrix<float, 7, 3> pinv_J;
	Matrix<float, 7, 3> J_T;
	J_T = J.transpose();
	Matrix3f temp = J * J_T;
	temp = temp.inverse();

	pinv_J = J_T * temp;
	return pinv_J;
}



Vector7f computeNSVel(Vector6f r_dot, Matrix6_7f J)
{
	Matrix7_6f pinv_J = pinv(J);
	Vector7f q_dot = pinv_J * r_dot;
	return q_dot;
}


void computeDLSVelocity(Vector7f& config_q_dot,
	const Vector6f& des_vel,
	const Matrix4f& prev_des_T, const Matrix4f& prev_curr_T,
	const Matrix4f& des_T, const Matrix4f& curr_T,
	const Matrix6_7f& J, const Matrix6f& Kp)
{
	Vector3f error_pos, error_angle;
	Vector6f total_error;
	float sim_error_angle[3];

	Vector7f range_space_velocities;
	Matrix6f I;
	I.setIdentity();
	Vector6f r_dot;

	//! POSITION ERROR
	error_pos = des_T.block<3, 1>(0, 3) - curr_T.block<3, 1>(0, 3);

	//! ORIENTATION ERROR
	Matrix4f angular_error_T;
	float sim_angular_error_T[12];

	Vector3f desired_angle, current_angle;
	Vector3f prev_desired_angle, prev_current_angle;
	float sim_des_angle[3];
	float sim_curr_angle[3];
	float sim_prev_des_angle[3];
	float sim_prev_curr_angle[3];
	float sim_prev_des_T[12];
	float sim_prev_curr_T[12];
	float sim_des_T[12];
	float sim_curr_T[12];
	Matrix3f error_R;

	Matrix3f tmp_1 = curr_T.block<3, 3>(0, 0).transpose();
	Matrix3f temp_R = tmp_1 * des_T.block<3, 3>(0, 0);

	angular_error_T.setZero();
	angular_error_T.block<3, 3>(0, 0) = temp_R;
	eigen2SimTransf(angular_error_T, sim_angular_error_T);

	simGetEulerAnglesFromMatrix(sim_angular_error_T, sim_error_angle);
	sim2EigenVec3f(sim_error_angle, error_angle);

	//! Unbound the error
	eigen2SimTransf(des_T, sim_des_T);
	eigen2SimTransf(curr_T, sim_curr_T);
	eigen2SimTransf(prev_des_T, sim_prev_des_T);
	eigen2SimTransf(prev_curr_T, sim_prev_curr_T);

	simGetEulerAnglesFromMatrix(sim_des_T, sim_des_angle);
	simGetEulerAnglesFromMatrix(sim_curr_T, sim_curr_angle);
	simGetEulerAnglesFromMatrix(sim_prev_des_T, sim_prev_des_angle);
	simGetEulerAnglesFromMatrix(sim_prev_curr_T, sim_prev_curr_angle);

	sim2EigenVec3f(sim_curr_angle, current_angle);
	sim2EigenVec3f(sim_des_angle, desired_angle);
	sim2EigenVec3f(sim_prev_curr_angle, prev_current_angle);
	sim2EigenVec3f(sim_prev_des_angle, prev_desired_angle);

	for (int i = 0; i < 3; i++)
	{
		current_angle(i) = unboundAngle(prev_current_angle(i), current_angle(i));
		desired_angle(i) = unboundAngle(prev_desired_angle(i), desired_angle(i));
	}

	//! Calculating T_XYZ matrix
	Matrix3f T_xyz;
	T_xyz << 1, 0, sin(current_angle(1)),
		0, cos(current_angle(0)), -cos(current_angle(1))*sin(current_angle(0)),
		0, sin(current_angle(0)), cos(current_angle(0))*cos(current_angle(1));

	error_angle = T_xyz * error_angle;
	total_error << error_pos, error_angle;

	//! R_DOT
	r_dot = des_vel + Kp * total_error;

	//! Composing the final q_dot that will be imposed.
	float mu = 0.5f;

	Matrix7_6f J_T = J.transpose();
	Matrix6f temp = J * J_T + mu * mu * I;
	temp = temp.inverse();

	config_q_dot = J_T * temp * r_dot;
}



