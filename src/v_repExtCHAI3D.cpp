//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Sebastien Grange
\version   3.1.1 $Rev: 1941 $
*/

//==============================================================================


#include <algorithm>
#include <iostream>
#include <chrono>
#include <ctime>
#include <ratio>
#include <math.h>
#include <set>
#include <queue>
#include <vector>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Extreme_points_traits_adapter_3.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/Extreme_points_traits_adapter_3.h>
//#include <CGAL/convex_hull_3.h>
//#include <CGAL/Vector_3.h>
//typedef CGAL::Exact_predicates_inexact_constructions_kernel      K;
//typedef K::Point_3                                               Point_3;
//typedef CGAL::Surface_mesh<Point_3>                              Mesh;
//typedef K::Vector_3                                              Vector_3;
using namespace std;

#include "v_repExtCHAI3D.h"
#include "luaFunctionData.h"
#include "v_repLib.h"

#include "chai3d.h"
using namespace chai3d;

#ifdef _WIN32
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif

#ifdef __APPLE__
#include <unistd.h>
#endif

#define PLUGIN_VERSION 1

#include "../DeviceState.h" //! Improve
#include "Tissue.h"
#include "utility.h"
#include "robot_utilities.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

/** \addtogroup V-REP
*  @{
*/



//---------------------------------------------------------------------------
//
//  globals
//
//---------------------------------------------------------------------------



// forward declarations
class  Object;
struct Device;

///  \brief Haptic thread control flag.
bool SimulationRunning;

///  \brief Haptic thread handle.
cThread *HapticThread;

///  \brief Haptic thread stop synchronization lock.
cMutex *SimulationLock;

///  \brief Haptic \ref scene "scene" synchronization lock.
cMutex *SceneLock;

///  \brief Haptic \ref Object "objects" synchronization lock.
cMutex *ObjectLock;

///  \brief Haptic \ref Object "objects" list.
vector<Object*> Objects;

///  \brief CHAI3D haptic device handler.
cHapticDeviceHandler *DeviceHandler;

///  \brief Haptic \ref scene "scene" container.
cWorld *World;

///  \brief CHAI3D haptic device list.
map<int, cGenericHapticDevicePtr> Devices;

///  \brief Haptic \ref Device "device" list.
map<int, Device*> Cursors;

cPrecisionClock Clock;
double BiggestRadius = 0.0;
double LowestStiffness = -1.0;

// ------------------------------------------------------------------------- //														From here is written by them
// ------------------------------------------------------------------------- //
// -------------------- Definizioni globali del Device ----------------------//
// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
typedef std::vector<Eigen::Vector3f> Vector3fVector;

bool ABILITA_LE_FORZE = true;
std::chrono::high_resolution_clock::time_point t_0, t_1;

DeviceState device_state;
#define DEVICE_IDX 0
#define TOOL_RADIUS 0.005
#define WS_RADIUS 0.2

Tissue tis;

simFloat tool_size[3] = {
	(float)0.03,
	(float)0.03,
	(float)0.1};
simInt dummy_handler;
simInt tool_handler;
simInt tool_tip_handler;
simInt lwr_target_handler;
simInt lwr_tip_handler;
simInt lwr_needle_handler;
simFloat resolution = 0.75f;
simInt force_displ_graph_handler;
simInt force_graph_handler;
simInt ext_force_graph_handler;
simInt UI_handler;
simInt LayersUI_handler;
std::vector<simInt> lwr_joint_handlers;
simInt aux_val[2] = { NULL, NULL };
simInt button_handler;
simChar* current_controller = "Position/Position-Force Controller";

bool want_to_print = false;
int button = 0, prev_button = 0;
bool first_press = true;
int global_cnt = 0;

Vector3d global_device_force(0.0, 0.0, 0.0);
Vector3f lwr_tip_external_F(0.0, 0.0, 0.0);


// Tissue params
VectorXf K;
VectorXf p_thick;
VectorXf thick;

Matrix4f offset_transform;
Vector3f lin_offset;
Matrix3f rot_offset;
Matrix3f force_offset_rot;
Vector3f F_mc;
Vector3f tool_vel, tool_omega;
Vector3f lwr_tip_vel, lwr_tip_omega;
Vector7f lwr_init_q;
Vector7f lwr_q_dot;
Matrix4f lwr_tip_T, target_T;
Matrix4f prev_lwr_tip_T, prev_target_T;

std::vector<float> lwr_curr_q_vector;

// Contact point
int objects_in_contact[2];
Vector3f contact_point;
Vector3f contact_force;
Vector3f contact_normal;
Vector3fVector contact_points;
Vector3fVector contact_normals;

// For penetration
Vector3f tool_tip_init_dir;
Vector3f tool_tip_init_pos;
Matrix4f lwr_tip_init_T;
Matrix4f dummy_T;

// UI parameters
int controller_ID = 1;
Matrix3f K_m, B_m;
Vector4f UI_thick_vec, UI_K_vec, UI_B_vec, UI_p_t_p_vec;
std::vector<std::string> UI_layers_names = { "Skin", "Fat", "Muscle", "Bone"};
int UI_layer_idx = 0;
bool use_default_tissue_values = false;

// FILE for exporting data on matlab
std::ofstream file_DOP_force;
std::ofstream file_time_forces;
std::ofstream file_perforation_error;
std::ofstream file_contacts_error;


//! Velocity Filtering													What is this??????
const int buffer_vel_size = 3;
Vector3f device_LPF_vel(0.0, 0.0, 0.0); //! ATTENZIONE questo deve essere tool_tip
std::vector<Eigen::Vector3f> device_vel_vector (buffer_vel_size);
std::vector<Eigen::Vector3f> device_mean_vel_vector(buffer_vel_size);

Vector3f tool_tip_LPF_vel(0.0, 0.0, 0.0); //! ATTENZIONE questo deve essere lwr_tip
std::vector<Eigen::Vector3f> tool_tip_vel_vector(buffer_vel_size);
std::vector<Eigen::Vector3f> tool_tip_mean_vel_vector(buffer_vel_size);

Vector3f tool_tip_LPF_omega(0.0, 0.0, 0.0); //! ATTENZIONE questo deve essere lwr_tip
std::vector<Eigen::Vector3f> tool_tip_omega_vector(buffer_vel_size);
std::vector<Eigen::Vector3f> tool_tip_mean_omega_vector(buffer_vel_size);

Vector3f lwr_tip_LPF_vel(0.0, 0.0, 0.0);
std::vector<Eigen::Vector3f> lwr_tip_vel_vector(buffer_vel_size);
std::vector<Eigen::Vector3f> lwr_tip_mean_vel_vector(buffer_vel_size);

Vector3f lwr_tip_LPF_omega(0.0, 0.0, 0.0);
std::vector<Eigen::Vector3f> lwr_tip_omega_vector(buffer_vel_size);
std::vector<Eigen::Vector3f> lwr_tip_mean_omega_vector(buffer_vel_size);

float time_step;

// Fake movement parameters
bool device_found = false;
bool first_press_fake = true;
float initial_dev_pos[3];

// ------------------------------------------------------------------------- //														From here is written by them
// ------------------------------------------------------------------------- //
// --------------------------- Global Variables -----------------------------//
// -------------------------- Peter Cook Bulukin---------------------------- //
// ------------------------------------------------------------------------- //


// Handles
int dummyHandle;									// This is the handle of the dummy device. That is the virtual 
													// representation of the position of the haptic device in the scene.
int dummyToolTipHandle;								// The tool tip dummy that is used to move the robot.
int phantomHandle;
int needleHandle;
int needleTipHandle;
int forceGraphHandle;
int lwrTipHandle;									// A dummy that is always connected to the needle tip.
int needleForceGraphHandle;
int penetrationLengthGraphHandle;

// CoppeliaSim object parameter IDs
const int RESPONDABLE = 3004;                       // Object parameter id for toggling respondable.
const int RESPONDABLE_MASK = 3019;                  // Object parameter id for toggling respondable mask.

// Coefficients for bidirectional Karnopp friction model
const float D_p = 18.45;                            // Positive static friction coefficient. Unit: N/m 
const float D_n = -18.23;                           // Negative static friction coefficient. Unit: N/m
const float b_p = 212.13;                           // Positive damping coefficient. Unit: N-s/m²
const float b_n = -293.08;                          // Negative damping coefficient. Unit: N-s/m²
const float C_p = 10.57;                            // Positive dynamic friction coefficient. Unit: N/m
const float C_n = -11.96;                           // Negative dynamic friction coefficient. Unit: N/m
const float zeroThreshold = 5.0e-6;                 // (delta v/2 in paper) Threshold on static and dynamic fricion. Unit: m/s

// Config variables: Use these to configurate the details of the execution.
const float engineForceScalar = 1.0;				// How much of the v-rep engine force should be counted.
const float modelForceScalar = 1.0;					// How much of the calculated force should be used.
const std::string forceModel = "kelvin-voigt";		// Which model should be used to model the forces.
const bool useOnlyZForceOnEngine = false;			// Should only z direction be used, or should the full magnitude of the force engine be used in calculations.
													// When changed, force thresholds has to be tweaked to get similar functionailty.
const bool enableExitTissue = true;					// Enable that tissues are enabeled when exiting the tissue. If false, severeal insertions in the same tissue are not possible in one simulation. 
const bool constantPunctureThreshold = false;		// Use the same puncture threshold for all tissues.
const float punctureThreshold = 1.0e-2;				// Set constant puncture threshold (only used if constantPunctureThreshold==true)
const float needleRadius = 2e-3;					// Used in calculating Kelvin Voigt model

// Kelvin-Voigt Params
map<string, float> kelvinMu = {
	{ "Fat", 1.0e6f },
	{ "muscle", 4.0e6f },
	{ "lung", 2.0e6f },
	{ "bone", 30.0e6f },
	{ "default", 3.0e6f }
};

map<string, float> kelvinKThresh = {
	{ "Fat", 1.0e-2 },
	{ "muscle", 1.0e-3 },
	{ "lung", 1.0e-3 },
	{ "bone", 1.0 },
	{ "default", 1.0e-3 }
};


// State variables
bool virtualFixture = false;									// This will be true when the needle is in the tissue.
float needleVelocityMagnitude;											// Velocity of the needle, calculated from the LWR_tip in V-REP.
float fullPenetrationLength;									// The total penetration length (length of needle covered by tissue)
float externalForceMagnitude;									// Magnitude of all external forces on the needle. (Magnitude of externalForce)
float lwrTipPhysisEngineForceMagnitude = 0;						// Magnitude of external forces on the Needle_tip created by the physics engine.
Vector3f lwrTipPhysicsEngineForce = Vector3f(0.0, 0.0, 0.0);	// Force vector of external forces on the needle_tip created by the physics engine.
Vector3f externalForce;											// Total forces on the needle created by the physics engine AND the modeled forces, relative to the dummy.
Vector3f toolTipPos;											// Position of the needle tool tip. Calculated from the LWR_tip object in V-REP.
Vector3f needleDirection;										// Direction of the needle calculated from the LWR_tip in V-REP.
Vector3f needleVelocity;
Vector3f needleAngularVelocity;

// Utility structures
struct sPuncture {
	int handle;
	Vector3f position;
	Vector3f direction;
	std::string name;
	float penetrationLength;

	void printPuncture(bool puncture) {
		if (puncture) {
			std::cout << "New puncture: " << name << std::endl;
		}
		else {
			std::cout << "Exit puncture: " << name << std::endl;
		}
		std::cout << "Position: " << position(0) << ", " << position(1) << ", " << position(2) << std::endl;
		std::cout << "Direction: " << direction(0) << ", " << direction(1) << ", " << direction(2) << std::endl;
	}
};

map<int, string> handle2Name;
std::vector<sPuncture> punctures;

// ---------------------------------------------------------------- //
// ---------------------------------------------------------------- //
// ------------------- FUNCTIONS DECLARATIONS ----------------------//
// ----------------------Peter Cook Bulukin------------------------ //
// ---------------------------------------------------------------- //


void addPuncture(int handle);
void checkContacts();
void checkPunctures();
void modelExternalForces(std::string forceModel);
void reactivateTissues();
void initAllTissueNames();
void setAllTissuesRespondable();
void setGraphs();
void setRespondable(int handle);
void setUnRespondable(int handle);
void updateNeedleDirection();
void updateNeedleState();
void updateNeedleTipPos();
void updateNeedleVelocity();
int checkSinglePuncture(sPuncture puncture);
float mu(sPuncture puncture);
float A(float x);
float distance3d(Vector3f point1, Vector3f point2);
float generalForce2NeedleTipZ(Vector3f force);
float KThresh(std::string tissueName);
float karnoppModel();
float kelvinVoigtModel();
float sgn(float x);
Vector3f changeBasis(const float* objectMatrixReferenceFrame, Vector3f vector);
Vector3f simContactInfo2EigenForce(const float* contactInfo);
Vector3f simObjectMatrix2EigenDirection(const float* objectMatrix);



// ---------------------------------------------------------------- //
// ---------------------------------------------------------------- //
//! ------------------ FUNCTIONS DECLARATIONS ----------------------//
// ---------------------------------------------------------------- //
// ---------------------------------------------------------------- //

// Pose calculation
void updateRot(void);
void updatePosPenetration(void); // o.O
void updatePose(void);
void getOffset(void);
void updateRobotPose(int target_handler, Vector3f target_lin_vel, Vector3f target_ang_vel);
void manageContact(void);

// Force functions
void computeGlobalForce(void);

// Filtering
void filterVelocity(Vector3fVector& v_vector, 
	Vector3f& new_vel, 
	Vector3f& mean_vel, 
	bool init);
void LPFilter(Vector3fVector& v_vector, 
	Vector3f& new_vel, 
	Vector3fVector& mean_vel_vector, 
	Vector3f& LPF_vel, 
	bool init);

// UI Functions
void readUI(void);
void readLayersUI(void);
void checkAndSetValues(int ID, int UI_handler);
void checkValues(int init_val, 
	int fin_val, 
	float* param, 
	float default_value,
	int UI_handler);
void loadLayerParams(int layer_idx);
void readLayerParams(int layer_idx, float* tmp, int UI_handler);

// Fake movement - when device is not found																								Until here
void moveFakeDevice(DeviceState& state);



//---------------------------------------------------------------------------
//
//  objects
//
//---------------------------------------------------------------------------



///  \brief Types of haptic objects.
///
///  Each \ref Object in the \ref scene "scene" must be assigned one of the following types.

enum ObjectType
{
	///  \brief Arbitrary 3D mesh object.
	SHAPE,

	///  \brief Constraint that holds the haptic device at a given point.
	///  The point is defined by \ref State::Pos.
	CONSTRAINT_POINT,

	///  \brief Constraint that holds the haptic device on a given segment.
	///  The segment is defined by a point (\ref State::Pos) and a vector (\ref State::Dir).
	CONSTRAINT_SEGMENT,

	///  \brief Constraint that holds the haptic device on a given plane.
	///  The plane is defined by a point (\ref State::Pos) and a normal direction (\ref State::Dir).
	CONSTRAINT_PLANE
};



///  \brief Data structure that contains an object state.

struct State
{
	///  \brief Constructor.
	State()
	{
		Pos.set(0.0, 0.0, 0.0);
		Rot.identity();
		Dir.set(0.0, 0.0, 0.0);
		Kp = Kv = Fmax = 0.0;
	}

	///  \brief \ref SHAPE and constraints position.
	cVector3d Pos;

	///  \brief \ref SHAPE orientation matrix.
	cMatrix3d Rot;

	///  \brief Constraint direction.
	///  \note
	///  For \ref CONSTRAINT_POINT, this member is ignored.<br>
	///  For \ref CONSTRAINT_SEGMENT, this member describes the segment length and direction from the \ref Pos member.<br>
	///  For \ref CONSTRAINT_PLANE, this member describes the normal direction to the plane.
	cVector3d Dir;

	///  \brief Constraints stiffness factor, expressed as a factor (0.0 < Kp < 1.0) of the maximum allowed stiffness (which depends on the type of haptic devices connected to the scene).
	double Kp;

	///  \brief Constraints viscosity factor, expressed as a factor (0.0 < Kv < 1.0) of the maximum allowed damping (which depends on the type of haptic devices connected to the scene).
	double Kv;

	///  \brief Constraints maximum force generated by the \ref Kp member, expressed in N.
	double Fmax;
};



///  \brief Data structure that contains a haptic device handle and related state information.

struct Device
{
	///  \brief Constructor.
	Device(cWorld *world)
	{
		Tool = new cToolCursor(world);
		Pos.set(0.0, 0.0, 0.0);
		Vel.set(0.0, 0.0, 0.0);
		Force.set(0.0, 0.0, 0.0);
	}

	///  \brief Destructor.
	virtual ~Device()
	{
		delete Tool;
	}

	///  \brief Haptic device handle.
	cToolCursor *Tool;

	///  \brief Haptic device position.
	cVector3d Pos;

	///  \brief Haptic device velocity.
	cVector3d Vel;

	///  \brief Additional force to apply to haptic device.
	cVector3d Force;

	///  \brief Maximum stiffness allowed for this device.
	double MaxStiffness;

	///  \brief Maximum damping allowed for this device.
	double MaxDamping;
};



///  \brief Data structure that contains an object of type \ref ObjectType used to generate haptic feedback.

class Object
{
private:

	///  \brief Initial object timestamp.
	double t0;

	///  \brief Initial object state.
	State  S0;

	///  \brief Target state timestamp.
	double tT;

	///  \brief Target object state.
	State  ST;


	///  \brief Set new target state.
	///  This method is shared by \ref SetTarget(float pos[3], float rot[3], float stiffnessFactor)
	///  and \ref SetTarget(float pos[3], float dir[3], float Kp, float Kv, float Fmax).

	void SetTarget()
	{
		// update current position and timestep
		double t = Clock.getCPUTimeSeconds();
		double timestep = t - t0;
		Update(t);

		// set starting point as current position
		S0 = S;
		t0 = t;

		// store new target
		tT = t + timestep;
	}


public:

	///  \brief Object type.
	ObjectType Type;

	///  \brief Internal handle to a \ref Device assigned to this object.
	///  \note This member can be negative if the object is not assigned to any particular device (e.g. \ref SHAPE).
	///        All \ref constraints "constraint" objects are assigned to a device.
	int Device;

	///  \brief Object mesh that this object renders haptically in the scene.
	cMesh *Mesh;

	///  \brief Current object state.
	State S;


	///  \brief \ref SHAPE constructor.
	///
	///  This member creates a new \ref SHAPE and adds it to the \ref scene "scene".
	///
	///  \param pos               \ref SHAPE position in the scene, expressed in m.
	///  \param rot               \ref SHAPE orientation in the scene, expressed as ZYX extrinsic Euler angles.
	///  \param vertices          list of vertices that define the \ref Mesh member.
	///  \param indices           list of vertex indices that define the \ref Mesh triangles.
	///  \param stiffnessFactor   object stiffness factor (must be between 0.0 and 1.0).

	Object(float pos[3], float rot[3], vector<float>vertices, vector<int> indices, float stiffnessFactor)
	{
		Type = SHAPE;
		Device = -1;
		Mesh = new cMesh();

		for (int i = 0; i<int(indices.size()) / 3; i++)
		{
			// build mesh
			int ind[3] = { indices[3 * i + 0], indices[3 * i + 1], indices[3 * i + 2] };
			cVector3d v1 = cVector3d(vertices[3 * ind[0] + 0], vertices[3 * ind[0] + 1], vertices[3 * ind[0] + 2]);
			cVector3d v2 = cVector3d(vertices[3 * ind[1] + 0], vertices[3 * ind[1] + 1], vertices[3 * ind[1] + 2]);
			cVector3d v3 = cVector3d(vertices[3 * ind[2] + 0], vertices[3 * ind[2] + 1], vertices[3 * ind[2] + 2]);
			Mesh->newTriangle(v1, v2, v3);
		}

		// set default material properties
		Mesh->m_material->setDynamicFriction(0.0);
		Mesh->m_material->setStaticFriction(0.0);
		Mesh->m_material->setHapticTriangleSides(true, false);

		// compute mesh properties
		Mesh->computeAllNormals();
		Mesh->computeBoundaryBox(true);
		Mesh->createAABBCollisionDetector(BiggestRadius);

		// set origin parameters
		t0 = Clock.getCPUTimeSeconds();
		S0.Kp = stiffnessFactor;
		S0.Pos.set(pos[0], pos[1], pos[2]);
		S0.Rot.setExtrinsicEulerRotationRad(rot[0], rot[1], rot[2], C_EULER_ORDER_ZYX);

		// set target at origin for now
		tT = t0;
		ST = S0;

		// update current position
		S = S0;
	}


	///  \brief \ref constraints "Constraints" constructor.
	///
	///  This member creates a new \ref constraints "constraint" and adds it to the \ref scene "scene".
	///
	///  \param constraintType    type of constraint to create.
	///  \param device            device handle (returned by \ref hapticConnect()) to assign this constraint to.
	///  \param pos               constraint position in the scene, expressed in m.
	///  \param dir               constraint direction, as defined in \ref ObjectType.
	///  \param Kp                constraint stiffness factor (between 0.0 and 1.0).
	///  \param Kv                constraint viscosity factor (between 0.0 and 1.0).
	///  \param Fmax              constraint max force for Kp, expressed in N.

	Object(ObjectType constraintType, int device, float pos[3], float dir[3], float Kp, float Kv, float Fmax)
	{
		Type = constraintType;
		Device = device;
		Mesh = NULL;

		// set origin parameters
		t0 = Clock.getCPUTimeSeconds();
		S0.Kp = Kp;
		S0.Kv = Kv;
		S0.Fmax = Fmax;
		S0.Pos.set(pos[0], pos[1], pos[2]);
		if (dir)
		{
			S0.Dir.set(dir[0], dir[1], dir[2]);
		}
		else
		{
			S0.Dir.set(1.0, 1.0, 1.0);
		}

		// set target at origin for now
		tT = t0;
		ST = S0;

		// update current position
		S = S0;
	}


	///  \brief Destructor.

	virtual ~Object()
	{
		if (Mesh)
		{
			delete Mesh;
		}
	}


	///  \brief Set new \ref SHAPE target state.
	///
	///  \param pos               new target \ref SHAPE target position, expressed in m.
	///  \param rot               new target \ref SHAPE orientation in the scene, expressed as ZYX extrinsic Euler angles.
	///  \param stiffnessFactor   new target object stiffness factor (between 0.0 and 1.0).

	void SetTarget(float pos[3], float rot[3], float stiffnessFactor)
	{
		// update current position and timestep
		SetTarget();

		// store new target
		ST.Pos.set(pos[0], pos[1], pos[2]);
		ST.Rot.setExtrinsicEulerRotationRad(rot[0], rot[1], rot[2], C_EULER_ORDER_ZYX);
		ST.Kp = stiffnessFactor;
	}


	///  \brief Set new \ref constraints "constraint" target state.
	///
	///  \param pos   new constraint position in the scene, expressed in m.
	///  \param dir   new constraint direction, as defined in \ref ObjectType.
	///  \param Kp    new constraint stiffness factor (between 0.0 and 1.0).
	///  \param Kv    new constraint viscosity factor (between 0.0 and 1.0).
	///  \param Fmax  new constraint max force for Kp, expressed in N.

	void SetTarget(float pos[3], float dir[3], float Kp, float Kv, float Fmax)
	{
		// update current position and timestep
		SetTarget();

		// store new target
		ST.Pos.set(pos[0], pos[1], pos[2]);
		ST.Dir.set(dir[0], dir[1], dir[2]);
		ST.Kp = Kp;
		ST.Kv = Kv;
		ST.Fmax = Fmax;
	}


	///  \brief Interpolate all object parameters to a given timestamp.
	///
	///  The state at the timestamp is linearly interpolated between the initial state (\ref Object::S0) and
	///  the current target state (\ref Object::ST), and is available after computation in \ref Object::S.
	///
	///  \param t   timestamp, expressed in s.

	void Update(double t)
	{
		// safeguards
		if (tT <= t0 || t  <  t0)
		{
			S = S0;
		}
		else if (t  >  tT)
		{
			S = ST;
		}

		// interpolation case
		else
		{
			double ratio = (t - t0) / (tT - t0);

			// interpolate parameters
			S.Kp = S0.Kp + ratio * (ST.Kp - S0.Kp);
			S.Kv = S0.Kv + ratio * (ST.Kv - S0.Kv);
			S.Fmax = S0.Fmax + ratio * (ST.Fmax - S0.Fmax);

			// interpolate position
			S.Pos = S0.Pos + ratio * (ST.Pos - S0.Pos);

			// interpolate orientation based on object type
			switch (Type)
			{
			case SHAPE:
			{
				cVector3d axis;
				double angle;
				cMatrix3d s0t;
				S0.Rot.transr(s0t);
				(s0t * ST.Rot).toAxisAngle(axis, angle);
				cMatrix3d rot;
				rot.setAxisAngleRotationRad(axis, angle*ratio);
				S0.Rot.mulr(rot, S.Rot);
			}
			break;
			default:
				S.Dir = S0.Dir + ratio * (ST.Dir - S0.Dir);
				break;
			}
		}

		// apply to mesh if any
		if (Mesh)
		{
			Mesh->setLocalPos(S.Pos);
			Mesh->setLocalRot(S.Rot);
			Mesh->m_material->setStiffness(S.Kp * LowestStiffness);
		}
	}


	///  \brief Compute additional force contribution for a given \ref Device position and velocity.
	///
	///  This additional force provides the constraint defined by the relevant \ref ObjectType. The
	///  force is computed using the \ref Device handle and is added to the \ref Device::Force member.

	void ComputeForce()
	{
		cVector3d point;
		cVector3d force(0.0, 0.0, 0.0);

		// safeguard
		if (Device < 0 || Type == SHAPE || S.Fmax == 0.0)
		{
			return;
		}

		// compute interaction point based on constraint type
		switch (Type)
		{
		case CONSTRAINT_PLANE:
			point = cProjectPointOnPlane(Cursors[Device]->Pos, S.Pos, S.Dir);
			break;
		case CONSTRAINT_SEGMENT:
			point = cProjectPointOnSegment(Cursors[Device]->Pos, S.Pos, S.Pos + S.Dir);
			break;
		case CONSTRAINT_POINT:
		default:
			point = S.Pos;
			break;
		}

		// compute constraint force at interaction point
		force -= S.Kp * Cursors[Device]->MaxStiffness * (Cursors[Device]->Pos - point);

		// limit to force max
		double norm = force.length();
		if (norm > S.Fmax) force *= S.Fmax / norm;

		// add viscosity (only along constraints)
		force -= cProject(S.Kv * Cursors[Device]->MaxDamping * Cursors[Device]->Vel, force);

		// add force to device
		Cursors[Device]->Force.add(force);
	}

};



//---------------------------------------------------------------------------
//
//  object management
//
//---------------------------------------------------------------------------



///  \brief Add generic object to the \ref scene "scene".
///
///  \param object  Haptic \ref Object to add to the \ref scene "scene".
///
///  \return
///  A unique identifier for the object if successful,
///  -1 otherwise.

int addObject(Object *object)
{
	if (!object)
	{
		return -1;
	}

	// we cannot add/remove objects while the simulation needs them
	SceneLock->acquire();

	// add to world if necessary
	if (object->Mesh)
	{
		World->addChild(object->Mesh);
	}

	// identify free slot in object list
	int size = (int)Objects.size();
	int index = -1;
	for (int i = 0; i<size; i++)
	{
		if (Objects[i] == NULL)
		{
			Objects[i] = object;
			index = i;
			break;
		}
	}

	// otherwise, push at the back of the list
	if (index < 0)
	{
		Objects.push_back(object);
		index = (int)Objects.size() - 1;
	}

	// simulation may proceed
	SceneLock->release();

	return index;
}



///  \brief Update the parameters of a \ref SHAPE haptic \ref Object "object".
///
///  \param objectID          object identifier (returned by \ref addObject).
///  \param pos               new target \ref SHAPE target position, expressed in m.
///  \param rot               new target \ref SHAPE orientation in the scene, expressed as ZYX extrinsic Euler angles.
///  \param stiffnessFactor   new target object stiffness factor (between 0.0 and 1.0).
///
///  \return
///  \b true on success, \b false otherwise.

bool updateShape(int objectID, float pos[3], float rot[3], float stiffnessFactor)
{
	// safeguards
	if (objectID < 0 || objectID >= (int)Objects.size())        return false;
	if (!Objects[objectID] || Objects[objectID]->Type != SHAPE) return false;

	// safely update shape target position between simulation steps
	SceneLock->acquire();
	Objects[objectID]->SetTarget(pos, rot, stiffnessFactor);
	SceneLock->release();

	return true;
}



///  \brief Update the parameters of a \ref constraints "constraint" haptic \ref Object "object".
///
///  \param objectID  object identifier (returned by \ref addObject).
///  \param pos       new constraint position in the scene, expressed in m.
///  \param dir       new constraint direction, as defined in \ref ObjectType.
///  \param Kp        new constraint stiffness factor (between 0.0 and 1.0).
///  \param Kv        new constraint viscosity factor (between 0.0 and 1.0).
///  \param Fmax      new constraint max force for Kp, expressed in N.
///
///  \return
///  \b true on success, \b false otherwise.

bool updateConstraint(int objectID, float pos[3], float dir[3], float Kp, float Kv, float Fmax)
{
	// safeguards
	if (objectID < 0 || objectID >= (int)Objects.size())        return false;
	if (!Objects[objectID] || Objects[objectID]->Type == SHAPE) return false;

	// safely update constraint target position between simulation steps
	SceneLock->acquire();
	Objects[objectID]->SetTarget(pos, dir, Kp, Kv, Fmax);
	SceneLock->release();

	return true;
}



///  \brief Safely remove an \ref Object "object" from the \ref Objects "object list".
///
///  \param objectID    object identifier (returned by \ref addObject).
///
///  \return
///  \b true on success, \b false otherwise.

bool removeObject(int objectID)
{
	// safeguards
	if (objectID < 0 || objectID >= (int)Objects.size()) return false;
	if (!Objects[objectID])                              return false;

	// remove object between simulation steps
	SceneLock->acquire();

	if (Objects[objectID]->Mesh)
	{
		World->removeChild(Objects[objectID]->Mesh);
	}
	delete Objects[objectID];
	Objects[objectID] = NULL;

	SceneLock->release();

	return true;
}



///  \brief Safely remove an \ref Object "object" from the \ref Objects "object list".
///
///  \param object  pointer to \ref Object "object" to remove.
///
///  \return
///  \b true on success, \b false otherwise.

bool removeObject(Object *object)
{
	int size = (int)Objects.size();

	// identify the index of the object to remove
	for (int i = 0; i<size; i++)
	{
		if (object == Objects[i])
		{
			// remove object
			return removeObject(i);
		}
	}

	return false;
}



//---------------------------------------------------------------------------
//
//  haptic loop
//
//---------------------------------------------------------------------------



///  \brief Main haptic loop.
///
///  The haptic loop runs in the background (with high priority) and renders all
///  \ref Objects haptically to all \ref Devices "devices" in the \ref scene "scene". Use
///  \ref hapticStart() to start the haptic loop, and \ref hapticReset() to stop it.
///
///  \note
///  The hapticLoop is started and stopped automatically by \ref hapticConnect() and
///  \ref hapticDisconnect(), according to the number of devices connected.

void hapticLoop()
{
	// announce that we are running
	SimulationRunning = true;
	SimulationLock->acquire();


	// loop
	while (SimulationRunning)
	{
		// update each cursor
		for (const auto& cursor : Cursors)
		{
			// get device data
			cursor.second->Tool->updateFromDevice();

			// update cursor position and velocity
			cursor.second->Pos = cursor.second->Tool->getDeviceGlobalPos();
			cursor.second->Vel = cursor.second->Tool->getDeviceGlobalLinVel();

			// reset constraints force for that cursor
			cursor.second->Force.set(0.0, 0.0, 0.0);
		}

		// prevent modifications to the haptic scene during updates
		SceneLock->acquire();

		// update scene and constraint forces
		double t = Clock.getCPUTimeSeconds();
		for (const auto& object : Objects)
		{
			if (object)
			{
				// update all objects positions and parameter values (interpolation)
				object->Update(t);

				// compute additional constraint forces
				object->ComputeForce();
			}
		}

		// update world
		World->computeGlobalPositions(true);

		// compute interaction forces
		for (const auto& cursor : Cursors) cursor.second->Tool->computeInteractionForces();

		// we can now safely modify the haptic scene without affecting the interaction
		SceneLock->release();

		// add constraint forces and apply to each device
		for (const auto& cursor : Cursors)
		{
			//cursor.second->Tool->addDeviceGlobalForce(cursor.second->Force); // Global forces here!
			cursor.second->Tool->setDeviceGlobalForce(global_device_force);
			cursor.second->Tool->applyToDevice();
		}

		// there is no point going on if no devices are connected
		if (Cursors.size() == 0) SimulationRunning = false;
	}

	// let hapticStop() know that we are done
	SimulationLock->release();
}



//---------------------------------------------------------------------------
//
//  haptic loop management
//
//---------------------------------------------------------------------------



///  \brief Start the haptic loop.

void hapticStart()
{
	// start haptic thread
	if (!SimulationRunning)
	{
		HapticThread->start(hapticLoop, CTHREAD_PRIORITY_HAPTICS);
	}
}



///  \brief Stop the haptic loop.

void hapticStop()
{
	if (SimulationRunning)
	{
		SimulationRunning = false;

		// wait for the thread to exit
		SimulationLock->acquire();
		SimulationLock->release();
	}
}



///  \\brief Connect to a haptic device and add it to the \ref scene "scene".
///
///  If successful, the device is added to the \ref Devices "device list" and
///  will be controlled by the \ref hapticLoop().
///
///  \param deviceIndex       index of the device to open in the list of devices detected by the operating system.
///                           This is guaranteed to be the same between executions as long as no hardware changes occur.
///  \param toolRadius        radius of the haptic device proxy (tool) in the simulated \ref scene "scene", expressed in m.
///  \param workspaceRadius   radius of the simulated workspace that the physical device workspace must be scaled to, expressed in m.
///
///  \return
///  \b true on success, \b false otherwise.

bool hapticConnect(int deviceIndex, float toolRadius, float workspaceRadius)
{
	SceneLock->acquire();

	// update list of available devices
	DeviceHandler->update();

	// create haptic device
	cGenericHapticDevicePtr device = cGenericHapticDevice::create();

	// get access to the first available haptic device found
	if (!DeviceHandler->getDevice(device, deviceIndex))
	{
		SceneLock->release();
		return false;
	}

	// create a tool (cursor)
	Device *cursor = new Device(World);

	// connect the haptic device to the virtual tool
	cursor->Tool->setHapticDevice(device);

	// insert cursor into the world
	World->addChild(cursor->Tool);

	// define a radius for the virtual tool (sphere)
	cursor->Tool->setRadius(toolRadius);

	// scale the haptic device workspace as desired
	cursor->Tool->setWorkspaceRadius(workspaceRadius);

	// objects in the scene are going to rotate of translate
	cursor->Tool->enableDynamicObjects(true);

	// start the haptic tool
	device->calibrate();
	cursor->Tool->setUseForceRise(true);
	cursor->Tool->setRiseTime(1.0);
	cursor->Tool->start();

	// store biggest tool radius
	if (toolRadius > BiggestRadius)
	{
		BiggestRadius = toolRadius;

		// update collision detection
		for (const auto& object : Objects)
		{
			if (object && object->Mesh)
			{
				object->Mesh->createAABBCollisionDetector(BiggestRadius);
			}
		}
	}

	// add cursor to cursor list
	Devices[deviceIndex] = device;
	Cursors[deviceIndex] = cursor;

	// retrieve max stiffness and damping
	cHapticDeviceInfo info = device->getSpecifications();
	cursor->MaxStiffness = info.m_maxLinearStiffness;
	cursor->MaxDamping = info.m_maxLinearDamping;

	// determine world-wide max stiffness allowed
	double maxStiffness = cursor->MaxStiffness / cursor->Tool->getWorkspaceScaleFactor();
	if (maxStiffness < LowestStiffness || LowestStiffness < 0.0)
	{
		LowestStiffness = maxStiffness;

		// update all object stiffnesses
		for (const auto& object : Objects)
		{
			if (object)
			{
				object->Mesh->m_material->setStiffness(object->S.Kp * LowestStiffness);
			}
		}
	}

	SceneLock->release();

	// start haptic loop if necessary
	if (!SimulationRunning) hapticStart();

	return true;
}



///  \brief Disconnect from haptic device.
///
///  This will remove all objects associated with this particular device,
///  and will remove the device from the \ref scene "scene".
///
///  \param index   index of the device to disconnect. This should be the same as the \c index given to \ref hapticConnect().
///
///  \return
///  \b true on success, \b false otherwise.

bool hapticDisconnect(int index)
{
	// make sure the device exists
	if (Cursors.find(index) == Cursors.end())
	{
		return false;
	}

	// we cannot remove a device while the haptic thread is running
	if (SimulationRunning) hapticStop();

	// delete constraint objects associated with device
	for (const auto& object : Objects)
	{
		if (object && object->Device == index)
		{
			removeObject(object);
		}
	}

	// stop the haptic tool
	Cursors[index]->Tool->stop();

	// remove tool (cursor) from the world
	World->removeChild(Cursors[index]->Tool);

	// delete device and cursor
	delete Cursors[index];
	Devices[index].reset();
	Cursors.erase(index);
	Devices.erase(index);

	return true;
}



///  \brief Clean up the \ref scene "scene".
///
///  This function disconnects all devices and removes all objects from the \ref scene "scene".

void hapticReset()
{
	// disconnect from all devices
	for (const auto& cursor : Cursors)
	{
		hapticDisconnect(cursor.first);
	}

	// delete all objects
	for (const auto& object : Objects)
	{
		removeObject(object);
	}
	Objects.clear();
}



//---------------------------------------------------------------------------
//
//  LUA interface
//
//---------------------------------------------------------------------------



LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#define CONCAT(x,y,z)     x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)



				 // definitions for LUA_START_COMMAND
#define LUA_START_COMMAND "simExtCHAI3D_start"
const int inArgs_START[] = {
	3,
	sim_lua_arg_int,   0,
	sim_lua_arg_float, 0,
	sim_lua_arg_float, 0,
};

///  \brief Connect to a haptic device.
///
///  This LUA callback is a wrapper for \ref hapticConnect().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref hapticConnect(int deviceIndex, float toolRadius, float workspaceRadius).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    number simExtCHAI3D_start(number deviceIndex, number toolRadius, number workspaceRadius)
///  \endcode
///
///  \return
///  1 on success, -1 otherwise.

int chai3DStart(int deviceIndex, float toolRadius, float workspaceRadius)
{
	int commandResult = -1;

	if (hapticConnect(deviceIndex, toolRadius, workspaceRadius))
		commandResult = 1;
	else
		simSetLastError("chai3DStart", "Initialization failed.");

	return commandResult;
}

void LUA_START_CALLBACK(SLuaCallBack* p)
{
	int commandResult = -1;

	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_START, inArgs_START[0], LUA_START_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();
		int   deviceIndex = inData->at(0).intData[0];
		float toolRadius = inData->at(1).floatData[0];
		float workspaceRadius = inData->at(2).floatData[0];

		if (hapticConnect(deviceIndex, toolRadius, workspaceRadius)) commandResult = 1;
		else simSetLastError(LUA_START_COMMAND, "Initialization failed.");
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem(commandResult));
	data.writeDataToLua(p);
}



// definitions for LUA_RESET_COMMAND
#define LUA_RESET_COMMAND "simExtCHAI3D_reset"
const int inArgs_RESET[] = {
	0
};

///  \brief Disconnect from a haptic device and reset the \ref scene "scene".
///
///  This LUA callback is a wrapper for \ref hapticReset().
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_reset()
///  \endcode

void chai3DReset()
{
	hapticReset();
}
void LUA_RESET_CALLBACK(SLuaCallBack* p)
{
	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_RESET, inArgs_RESET[0], LUA_RESET_COMMAND))
	{
		hapticReset();
	}

	// populate reply
	p->outputArgCount = 0;
	data.writeDataToLua(p);
}



// definitions for LUA_ADD_SHAPE_COMMAND
#define LUA_ADD_SHAPE_COMMAND "simExtCHAI3D_addShape"
const int inArgs_ADD_SHAPE[] = {
	5,
	sim_lua_arg_float | sim_lua_arg_table, 9,
	sim_lua_arg_int | sim_lua_arg_table,   3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0
};

///  \brief Add a \ref SHAPE object to the \ref scene "scene".
///
///  This LUA callback is a wrapper for \ref addObject() and \ref Object::Object().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref Object::Object(float pos[3], float rot[3], vector<float>vertices, vector<int> indices, float stiffnessFactor).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_addShape(table_3 pos, table_3 rot, table vertices, table indices, number stiffnessFactor)
///  \endcode


int chai3DAddShape(simFloat** vertices, simFloat** indices, simFloat* position, simFloat* rotation, simFloat stiffness)
{
	int objectID = -1;
	// bound stiffness factor to a valid range
	if (stiffness > 1.0)
		stiffness = 1.0;
	if (stiffness < 0.0)
		stiffness = 0.0;

	vector<float> v_vector;
	vector<int>	idx_vector;

	//objectID = addObject(new Object(position, rotation, v_vector, idx_vector, stiffness));
	return -1; //objectID placeholder
}

void LUA_ADD_SHAPE_CALLBACK(SLuaCallBack* p)
{
	int objectID = -1;

	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_ADD_SHAPE, inArgs_ADD_SHAPE[0], LUA_ADD_SHAPE_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		vector<float> vertices(inData->at(0).floatData);
		vector<int>   indices(inData->at(1).intData);

		float pos[3];
		pos[0] = inData->at(2).floatData[0];
		pos[1] = inData->at(2).floatData[1];
		pos[2] = inData->at(2).floatData[2];

		float rot[3];
		rot[2] = inData->at(3).floatData[0];
		rot[1] = inData->at(3).floatData[1];
		rot[0] = inData->at(3).floatData[2];

		float stiffnessFactor = inData->at(4).floatData[0];

		// bound stiffness factor to a valid range
		if (stiffnessFactor > 1.0) stiffnessFactor = 1.0;
		if (stiffnessFactor < 0.0) stiffnessFactor = 0.0;

		objectID = addObject(new Object(pos, rot, vertices, indices, stiffnessFactor));
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem(objectID));
	data.writeDataToLua(p);
}



// definitions for LUA_ADD_CONSTRAINT_POINT_COMMAND
#define LUA_ADD_CONSTRAINT_POINT_COMMAND "simExtCHAI3D_addConstraintPoint"
const int inArgs_ADD_CONSTRAINT_POINT[] = {
	5,
	sim_lua_arg_int,                     0,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0
};

///  \brief Add a \ref CONSTRAINT_POINT object to the \ref scene "scene".
///
///  This LUA callback is a wrapper for \ref addObject() and \ref Object::Object().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref Object::Object(ObjectType constraintType, int device, float pos[3], float dir[3], float Kp, float Kv, float Fmax).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_addConstraintPoint(number deviceIndex, table_3 pos, number Kp, number Kv, number Fmax)
///  \endcode


//! TODO (Virtual Fixtures)

void LUA_ADD_CONSTRAINT_POINT_CALLBACK(SLuaCallBack* p)
{
	int objectID = -1;

	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_ADD_CONSTRAINT_POINT, inArgs_ADD_CONSTRAINT_POINT[0], LUA_ADD_CONSTRAINT_POINT_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		float pos[3];
		pos[0] = inData->at(1).floatData[0];
		pos[1] = inData->at(1).floatData[1];
		pos[2] = inData->at(1).floatData[2];

		float dir[3];
		dir[0] = 0.0;
		dir[1] = 0.0;
		dir[2] = 1.0;

		float Kp = inData->at(2).floatData[0];
		float Kv = inData->at(3).floatData[0];
		float Fmax = inData->at(4).floatData[0];

		// bound stiffness factor to a valid range
		if (Kp > 1.0) Kp = 1.0;
		if (Kp < 0.0) Kp = 0.0;

		// bound damping factor to a valid range
		if (Kv > 1.0) Kv = 1.0;
		if (Kv < 0.0) Kv = 0.0;

		objectID = addObject(new Object(CONSTRAINT_POINT, deviceIndex, pos, dir, Kp, Kv, Fmax));
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem(objectID));
	data.writeDataToLua(p);
}



// definitions for LUA_ADD_CONSTRAINT_SEGMENT_COMMAND
#define LUA_ADD_CONSTRAINT_SEGMENT_COMMAND "simExtCHAI3D_addConstraintSegment"
const int inArgs_ADD_CONSTRAINT_SEGMENT[] = {
	6,
	sim_lua_arg_int,                     0,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0
};

///  \brief Add a \ref CONSTRAINT_SEGMENT object to the \ref scene "scene".
///
///  This LUA callback is a wrapper for \ref addObject() and \ref Object::Object().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref Object::Object(ObjectType constraintType, int device, float pos[3], float dir[3], float Kp, float Kv, float Fmax).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_addConstraintSegment(table_3 posA, table_3 posB, number Kp, number Kv, number Fmax)
///  \endcode


//! TODO (Virtual Fixtures)

void LUA_ADD_CONSTRAINT_SEGMENT_CALLBACK(SLuaCallBack* p)
{
	int objectID = -1;

	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_ADD_CONSTRAINT_SEGMENT, inArgs_ADD_CONSTRAINT_SEGMENT[0], LUA_ADD_CONSTRAINT_SEGMENT_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		float pos[3];
		pos[0] = inData->at(1).floatData[0];
		pos[1] = inData->at(1).floatData[1];
		pos[2] = inData->at(1).floatData[2];

		float dir[3];
		dir[0] = inData->at(2).floatData[0];
		dir[1] = inData->at(2).floatData[1];
		dir[2] = inData->at(2).floatData[2];

		float Kp = inData->at(3).floatData[0];
		float Kv = inData->at(4).floatData[0];
		float Fmax = inData->at(5).floatData[0];

		// bound stiffness factor to a valid range
		if (Kp > 1.0) Kp = 1.0;
		if (Kp < 0.0) Kp = 0.0;

		// bound damping factor to a valid range
		if (Kv > 1.0) Kv = 1.0;
		if (Kv < 0.0) Kv = 0.0;

		objectID = addObject(new Object(CONSTRAINT_SEGMENT, deviceIndex, pos, dir, Kp, Kv, Fmax));
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem(objectID));
	data.writeDataToLua(p);
}



// definitions for LUA_ADD_CONSTRAINT_PLANE_COMMAND
#define LUA_ADD_CONSTRAINT_PLANE_COMMAND "simExtCHAI3D_addConstraintPlane"
const int inArgs_ADD_CONSTRAINT_PLANE[] = {
	6,
	sim_lua_arg_int,                     0,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0
};

///  \brief Add a \ref CONSTRAINT_PLANE object to the \ref scene "scene".
///
///  This LUA callback is a wrapper for \ref addObject() and \ref Object::Object().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref Object::Object(ObjectType constraintType, int device, float pos[3], float dir[3], float Kp, float Kv, float Fmax).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_addConstraintPlane(table_3 pos, table_3 normal, number Kp, number Kv, number Fmax)
///  \endcode

//! TODO (Virtual Fixtures)

void LUA_ADD_CONSTRAINT_PLANE_CALLBACK(SLuaCallBack* p)
{
	int objectID = -1;

	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_ADD_CONSTRAINT_PLANE, inArgs_ADD_CONSTRAINT_PLANE[0], LUA_ADD_CONSTRAINT_PLANE_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		float pos[3];
		pos[0] = inData->at(1).floatData[0];
		pos[1] = inData->at(1).floatData[1];
		pos[2] = inData->at(1).floatData[2];

		float dir[3];
		dir[0] = inData->at(2).floatData[0];
		dir[1] = inData->at(2).floatData[1];
		dir[2] = inData->at(2).floatData[2];

		float Kp = inData->at(3).floatData[0];
		float Kv = inData->at(4).floatData[0];
		float Fmax = inData->at(5).floatData[0];

		// bound stiffness factor to a valid range
		if (Kp > 1.0) Kp = 1.0;
		if (Kp < 0.0) Kp = 0.0;

		// bound damping factor to a valid range
		if (Kv > 1.0) Kv = 1.0;
		if (Kv < 0.0) Kv = 0.0;

		objectID = addObject(new Object(CONSTRAINT_PLANE, deviceIndex, pos, dir, Kp, Kv, Fmax));
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem(objectID));
	data.writeDataToLua(p);
}



// definitions for LUA_UPDATE_SHAPE_COMMAND
#define LUA_UPDATE_SHAPE_COMMAND "simExtCHAI3D_updateShape"
const int inArgs_UPDATE_SHAPE[] = {
	4,
	sim_lua_arg_int,                     0,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0
};

///  \brief Update the parameters of a \ref SHAPE haptic \ref Object "object".
///
///  This LUA callback is a wrapper for \ref updateShape().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref updateShape(int objectID, float pos[3], float rot[3], float stiffnessFactor).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_updateShape(number objectID, table_3 pos, table_3 rot, number stiffnessFactor)
///  \endcode


void chai3DUpdateShape(simInt objectID, simFloat* pos, simFloat* rot, simFloat stiffness)
{
	// bound stiffness factor to a valid range
	if (stiffness > 1.0)
		stiffness = 1.0;
	if (stiffness < 0.0)
		stiffness = 0.0;

	if (!updateShape(objectID, pos, rot, stiffness))
	{
		simSetLastError("chai3DUpdateShape", "Invalid shape ID.");
	}
}

void LUA_UPDATE_SHAPE_CALLBACK(SLuaCallBack* p)
{
	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_UPDATE_SHAPE, inArgs_UPDATE_SHAPE[0], LUA_UPDATE_SHAPE_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int objectID = inData->at(0).intData[0];

		float pos[3];
		pos[0] = inData->at(1).floatData[0];
		pos[1] = inData->at(1).floatData[1];
		pos[2] = inData->at(1).floatData[2];

		float rot[3];
		rot[2] = inData->at(2).floatData[0];
		rot[1] = inData->at(2).floatData[1];
		rot[0] = inData->at(2).floatData[2];

		float stiffnessFactor = inData->at(3).floatData[0];

		// bound stiffness factor to a valid range
		if (stiffnessFactor > 1.0) stiffnessFactor = 1.0;
		if (stiffnessFactor < 0.0) stiffnessFactor = 0.0;

		if (!updateShape(objectID, pos, rot, stiffnessFactor))
		{
			simSetLastError(LUA_UPDATE_SHAPE_COMMAND, "Invalid shape ID.");
		}
	}

	// populate reply
	p->outputArgCount = 0;
	data.writeDataToLua(p);
}



// definitions for LUA_UPDATE_CONSTRAINT_COMMAND
#define LUA_UPDATE_CONSTRAINT_COMMAND "simExtCHAI3D_updateConstraint"
const int inArgs_UPDATE_CONSTRAINT[] = {
	6,
	sim_lua_arg_int,                     0,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0,
	sim_lua_arg_float,                   0
};

///  \brief Update the parameters of a \ref constraints "constraint" haptic \ref Object "object".
///
///  This LUA callback is a wrapper for \ref updateShape().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref updateConstraint(int objectID, float pos[3], float dir[3], float Kp, float Kv, float Fmax).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_updateConstraint(number objectID, table_3 pos, table_3 dir, number Kp, number Kv, number Fmax)
///  \endcode


//! TODO?

void LUA_UPDATE_CONSTRAINT_CALLBACK(SLuaCallBack* p)
{
	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_UPDATE_CONSTRAINT, inArgs_UPDATE_CONSTRAINT[0], LUA_UPDATE_CONSTRAINT_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int objectID = inData->at(0).intData[0];

		float pos[3];
		pos[0] = inData->at(1).floatData[0];
		pos[1] = inData->at(1).floatData[1];
		pos[2] = inData->at(1).floatData[2];

		float dir[3];
		dir[0] = inData->at(2).floatData[0];
		dir[1] = inData->at(2).floatData[1];
		dir[2] = inData->at(2).floatData[2];

		float Kp = inData->at(3).floatData[0];
		float Kv = inData->at(4).floatData[0];
		float Fmax = inData->at(5).floatData[0];

		// bound stiffness factor to a valid range
		if (Kp > 1.0) Kp = 1.0;
		if (Kp < 0.0) Kp = 0.0;

		// bound damping factor to a valid range
		if (Kv > 1.0) Kv = 1.0;
		if (Kv < 0.0) Kv = 0.0;

		if (!updateConstraint(objectID, pos, dir, Kp, Kv, Fmax))
		{
			simSetLastError(LUA_UPDATE_CONSTRAINT_COMMAND, "Invalid shape ID.");
		}
	}

	// populate reply
	p->outputArgCount = 0;
	data.writeDataToLua(p);
}



// definitions for LUA_REMOVE_OBJECT_COMMAND
#define LUA_REMOVE_OBJECT_COMMAND "simExtCHAI3D_removeObject"
const int inArgs_REMOVE_OBJECT[] = {
	1,
	sim_lua_arg_int, 0
};

///  \brief Safely remove an \ref Object "object" from the \ref Objects "object list".
///
///  This LUA callback is a wrapper for \ref removeObject().<br>
///  The arguments parsed from the \c p buffer are passed on to \ref removeObject(int objectID).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    simExtCHAI3D_removeObject(number objectID)
///  \endcode

void chai3DRemoveObject(simInt objectID)
{
	if (!removeObject(objectID))
	{
		simSetLastError("chai3DRemoveObject", "Invalid shape ID.");
	}
}

void LUA_REMOVE_OBJECT_CALLBACK(SLuaCallBack* p)
{
	CLuaFunctionData data;

	// validate argument count and types
	if (data.readDataFromLua(p, inArgs_REMOVE_OBJECT, inArgs_REMOVE_OBJECT[0], LUA_REMOVE_OBJECT_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int objectID = inData->at(0).intData[0];

		if (!removeObject(objectID))
		{
			simSetLastError(LUA_REMOVE_OBJECT_COMMAND, "Invalid shape ID.");
		}
	}

	// populate reply
	p->outputArgCount = 0;
	data.writeDataToLua(p);
}



// definitions for LUA_READ_POSITION_COMMAND
#define LUA_READ_POSITION_COMMAND "simExtCHAI3D_readPosition"
const int inArgs_READ_POSITION[] = {
	1,
	sim_lua_arg_int, 0
};

///  \brief Retrieve the current position of a haptic device (identified by its \c deviceIndex) in the \ref scene "scene".
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    table_3 simExtCHAI3D_readPosition(number deviceIndex)
///  \endcode
///
///  \return
///  An array containing the XYZ position Cartesian coordinates of the device end-effector in the simulated \ref scene "scene".

void LUA_READ_POSITION_CALLBACK(SLuaCallBack* p)
{
	std::vector<float> retPos(3, 0.0f);

	// validate argument count and types
	CLuaFunctionData data;
	if (data.readDataFromLua(p, inArgs_READ_POSITION, inArgs_READ_POSITION[0], LUA_READ_POSITION_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		if (Cursors.find(deviceIndex) != Cursors.end())
		{
			cVector3d pos = Cursors[deviceIndex]->Tool->getDeviceGlobalPos();
			retPos[0] = (float)pos.x();
			retPos[1] = (float)pos.y();
			retPos[2] = (float)pos.z();
		}
	}

	// populate reply
	p->outputArgCount = 0;
	if (retPos.size() >= 3)
	{
		data.pushOutData(CLuaFunctionDataItem(retPos));
	}
	data.writeDataToLua(p);
}



// definitions for LUA_READ_FORCE_COMMAND
#define LUA_READ_FORCE_COMMAND "simExtCHAI3D_readForce"
const int inArgs_READ_FORCE[] = {
	1,
	sim_lua_arg_int, 0
};

///  \brief Retrieve the force currently displayed by a haptic device (identified by its \c deviceIndex).
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    table_3 simExtCHAI3D_readForce(number deviceIndex)
///  \endcode
///
///  \return
///  An array containing the XYZ Cartesian force component at the device end-effector.

void LUA_READ_FORCE_CALLBACK(SLuaCallBack* p)
{
	std::vector<float> retForce(3, 0.0f);

	// validate argument count and types
	CLuaFunctionData data;
	if (data.readDataFromLua(p, inArgs_READ_FORCE, inArgs_READ_FORCE[0], LUA_READ_FORCE_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		if (Cursors.find(deviceIndex) != Cursors.end())
		{
			cVector3d force = Cursors[deviceIndex]->Tool->getDeviceGlobalForce();
			retForce[0] = (float)force.x();
			retForce[1] = (float)force.y();
			retForce[2] = (float)force.z();
		}
	}

	// populate reply
	p->outputArgCount = 0;
	if (retForce.size() >= 3)
	{
		data.pushOutData(CLuaFunctionDataItem(retForce));
	}
	data.writeDataToLua(p);
}



//! Read State function -> Position, Orientation, Velocity
void chai3DReadState(simInt device_idx, DeviceState& state)
{
	if (Cursors.find(device_idx) != Cursors.end())
	{
		cVector3d pos = Cursors[device_idx]->Tool->getDeviceGlobalPos();
		state.pos(0) = (float)pos.x();
		state.pos(1) = (float)pos.y();
		state.pos(2) = (float)pos.z();

		cMatrix3d rot = Cursors[device_idx]->Tool->getDeviceGlobalRot();
		Matrix3f temp;
		temp.setZero();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				temp(i, j) = (float)rot(i, j);
			}
		}
		state.rot = temp;

		cVector3d vel = Cursors[device_idx]->Tool->getDeviceGlobalLinVel();
		state.vel(0) = (float)vel.x();
		state.vel(1) = (float)vel.y();
		state.vel(2) = (float)vel.z();

		state.T.block<3, 3>(0, 0) = temp;
		state.T(0, 3) = state.pos(0);
		state.T(1, 3) = state.pos(1);
		state.T(2, 3) = state.pos(2);
		
	}
}


// definitions for LUA_READ_BUTTONS_COMMAND
#define LUA_READ_BUTTONS_COMMAND "simExtCHAI3D_readButtons"
const int inArgs_READ_BUTTONS[] = {
	1,
	sim_lua_arg_int, 0
};

///  \brief Retrieve a haptic device (identified by its \c deviceIndex) end-effector buttons status.
///
///  The buttons are returned in the form of an array of bit. A bit state of \c 1 indicates that the button
///  at the given bit array index is pressed, \c 0 means it is not.
///
///  \param p   SLuaCallBack I/O buffer.
///
///  The corresponding LUA function is:
///  \code
///    table_3 simExtCHAI3D_readButtons(number deviceIndex)
///  \endcode
///
///  \return
///  An array of bits containing the buttons state for the given device.


unsigned int chai3DGetButton(simInt device_idx)
{
	if (Cursors.find(device_idx) != Cursors.end())
	{
		return Cursors[device_idx]->Tool->getUserSwitches();
	}
	else
		return 0;
}

void LUA_READ_BUTTONS_CALLBACK(SLuaCallBack* p)
{
	unsigned int buttons = 0;

	// validate argument count and types
	CLuaFunctionData data;
	if (data.readDataFromLua(p, inArgs_READ_BUTTONS, inArgs_READ_BUTTONS[0], LUA_READ_BUTTONS_COMMAND))
	{
		vector<CLuaFunctionDataItem>* inData = data.getInDataPtr();

		int deviceIndex = inData->at(0).intData[0];

		if (Cursors.find(deviceIndex) != Cursors.end())
		{
			buttons = Cursors[deviceIndex]->Tool->getUserSwitches();
		}
	}

	// populate reply
	p->outputArgCount = 0;
	data.pushOutData(CLuaFunctionDataItem((int)buttons));
	data.writeDataToLua(p);
}


// This (funny) function, used in a LUA script is useful to pass quatities
// calculated into the LUA script to the C++ one. For example, here is used
// to carry all the contact informations (calculated automatically by the 
// Contact Managere in the scene) from the LUA script to this C++ plugin.
#define LUA_READ_CONTACT_INFO_COMMAND "simSalaBim"
const int inArgs_READ_CONTACT_INFO[] = {
	4,
	sim_lua_arg_int | sim_lua_arg_table, 2,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3,
	sim_lua_arg_float | sim_lua_arg_table, 3
};
void LUA_READ_CONTACT_INFO_CALLBACK(SLuaCallBack* p)
{
	CLuaFunctionData data;
	if (data.readDataFromLua(p, inArgs_READ_CONTACT_INFO, inArgs_READ_CONTACT_INFO[0], LUA_READ_CONTACT_INFO_COMMAND))
	{
		float sim_contact_point[3];
		float sim_contact_force[3];
		float sim_contact_normal[3];
		std::vector<CLuaFunctionDataItem>* outData = data.getInDataPtr();
		
		objects_in_contact[0] = outData->at(0).intData[0];
		objects_in_contact[1] = outData->at(0).intData[1];

		sim_contact_point[0] = outData->at(1).floatData[0];
		sim_contact_point[1] = outData->at(1).floatData[1];
		sim_contact_point[2] = outData->at(1).floatData[2];

		sim_contact_force[0] = outData->at(2).floatData[0];
		sim_contact_force[1] = outData->at(2).floatData[1];
		sim_contact_force[2] = outData->at(2).floatData[2];

		sim_contact_normal[0] = outData->at(3).floatData[0];
		sim_contact_normal[1] = outData->at(3).floatData[1];
		sim_contact_normal[2] = outData->at(3).floatData[2];

		sim2EigenVec3f(sim_contact_point, contact_point);
		sim2EigenVec3f(sim_contact_force, contact_force);
		sim2EigenVec3f(sim_contact_normal, contact_normal);

	}
	p->outputArgCount = 0;
	data.writeDataToLua(p);

}


///  \brief V-REP shared library initialization.

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt)
{
	// setup plumbing
	HapticThread = new cThread;
	SimulationLock = new cMutex;
	SceneLock = new cMutex;
	ObjectLock = new cMutex;
	DeviceHandler = new cHapticDeviceHandler;
	World = new cWorld;

	char curDirAndFile[1024];
#ifdef _WIN32
	GetModuleFileName(NULL, curDirAndFile, 1023);
	PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
	if (getcwd(curDirAndFile, sizeof(curDirAndFile)) == NULL) strcpy(curDirAndFile, "");
#endif
	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp += "\\v_rep.dll";
#elif defined (__linux)
	temp += "/libv_rep.so";
#elif defined (__APPLE__)
	temp += "/libv_rep.dylib";
#endif
	vrepLib = loadVrepLibrary(temp.c_str());
	if (vrepLib == NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'CHAI3D' plugin.\n";
		return(0);
	}
	if (getVrepProcAddresses(vrepLib) == 0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'CHAI3D' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0);
	}

	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
	if (vrepVer<30103)
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'CHAI3D' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0);
	}

	// register LUA commands

	std::vector<int> inArgs;

	// **********************************************************************//
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READ_CONTACT_INFO, inArgs);
	simRegisterCustomLuaFunction(LUA_READ_CONTACT_INFO_COMMAND, strConCat("", LUA_READ_CONTACT_INFO_COMMAND, "(table_2 objectInContact,table_3 contactPt,table_3 forceDirectionAndAmplitude, table_3 contactN)"), &inArgs[0], LUA_READ_CONTACT_INFO_CALLBACK);
	// **********************************************************************//

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_START, inArgs);
	simRegisterCustomLuaFunction(LUA_START_COMMAND, strConCat("number result=", LUA_START_COMMAND, "(number deviceIndex,number toolRadius,number workspaceRadius)"), &inArgs[0], LUA_START_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_RESET, inArgs);
	simRegisterCustomLuaFunction(LUA_RESET_COMMAND, strConCat("", LUA_RESET_COMMAND, "()"), &inArgs[0], LUA_RESET_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ADD_SHAPE, inArgs);
	simRegisterCustomLuaFunction(LUA_ADD_SHAPE_COMMAND, strConCat("number objectID=", LUA_ADD_SHAPE_COMMAND, "(table vertices,table indices,table_3 position,table_3 orientation,number stiffnessFactor)"), &inArgs[0], LUA_ADD_SHAPE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ADD_CONSTRAINT_POINT, inArgs);
	simRegisterCustomLuaFunction(LUA_ADD_CONSTRAINT_POINT_COMMAND, strConCat("number objectID=", LUA_ADD_CONSTRAINT_POINT_COMMAND, "(number deviceIndex,table_3 position,number Kp,number Kv,number Fmax)"), &inArgs[0], LUA_ADD_CONSTRAINT_POINT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ADD_CONSTRAINT_SEGMENT, inArgs);
	simRegisterCustomLuaFunction(LUA_ADD_CONSTRAINT_SEGMENT_COMMAND, strConCat("number objectID=", LUA_ADD_CONSTRAINT_SEGMENT_COMMAND, "(number deviceIndex,table_3 point,table_3 segment,number Kp,number Kv,number Fmax)"), &inArgs[0], LUA_ADD_CONSTRAINT_SEGMENT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ADD_CONSTRAINT_PLANE, inArgs);
	simRegisterCustomLuaFunction(LUA_ADD_CONSTRAINT_PLANE_COMMAND, strConCat("number objectID=", LUA_ADD_CONSTRAINT_PLANE_COMMAND, "(number deviceIndex,table_3 position,table_3 normal,number Kp,number Kv,number Fmax)"), &inArgs[0], LUA_ADD_CONSTRAINT_PLANE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_UPDATE_SHAPE, inArgs);
	simRegisterCustomLuaFunction(LUA_UPDATE_SHAPE_COMMAND, strConCat("", LUA_UPDATE_SHAPE_COMMAND, "(number objectID,table_3 position,table_3 orientation,number stiffnessFactor)"), &inArgs[0], LUA_UPDATE_SHAPE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_UPDATE_CONSTRAINT, inArgs);
	simRegisterCustomLuaFunction(LUA_UPDATE_CONSTRAINT_COMMAND, strConCat("", LUA_UPDATE_CONSTRAINT_COMMAND, "(number objectID,table_3 positionA,table_3 positionB,number Kp,number Kv,number Fmax)"), &inArgs[0], LUA_UPDATE_CONSTRAINT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_REMOVE_OBJECT, inArgs);
	simRegisterCustomLuaFunction(LUA_REMOVE_OBJECT_COMMAND, strConCat("", LUA_REMOVE_OBJECT_COMMAND, "(number objectID)"), &inArgs[0], LUA_REMOVE_OBJECT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READ_POSITION, inArgs);
	simRegisterCustomLuaFunction(LUA_READ_POSITION_COMMAND, strConCat("table_3 position=", LUA_READ_POSITION_COMMAND, "(number deviceIndex)"), &inArgs[0], LUA_READ_POSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READ_FORCE, inArgs);
	simRegisterCustomLuaFunction(LUA_READ_FORCE_COMMAND, strConCat("table_3 force=", LUA_READ_FORCE_COMMAND, "(number deviceIndex)"), &inArgs[0], LUA_READ_FORCE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READ_BUTTONS, inArgs);
	simRegisterCustomLuaFunction(LUA_READ_BUTTONS_COMMAND, strConCat("number buttons=", LUA_READ_BUTTONS_COMMAND, "(number deviceIndex)"), &inArgs[0], LUA_READ_BUTTONS_CALLBACK);

	return PLUGIN_VERSION;
}



///  \brief V-REP shared library disconnect.

VREP_DLLEXPORT void v_repEnd()
{
	// stop haptic thread and cleanup
	hapticReset();

	unloadVrepLibrary(vrepLib);
}



///  \brief V-REP shared library message processing callback.																				
///	 This represents the core callback of the VREP plugin.
//																																		From here it is them again																				
VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData)
{
	int   errorModeSaved;
	void *retVal = NULL;

	simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
	simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);


	// Reading values put by the user in the 2 UIs.
	readUI();
	readLayersUI();


	// ------------------------------------------------------------------------- //
	// ------------------ WHEN THE USER CLICKS THE PLAY BUTTON ------------------//
	// ------------------ this function is executed only once ------------------ //
	// ------------------------------------------------------------------------- //
	if (message == sim_message_eventcallback_simulationabouttostart)
	{
		
		t_0 = std::chrono::high_resolution_clock::now();

		// Initialize the device
		// NOTE: the reference frames of the real haptic device and of the virtual 
		// haptic device coincide. The relation between the real and virtual world 
		// is an identity matrix. Anyway it is possible to scale the dimension of 
		// the virtual WS setting the value WS_RADIUS.
		if (chai3DStart(DEVICE_IDX, (float)TOOL_RADIUS, (float)WS_RADIUS) == 1)
		{
			device_found = true;
			green();
			cout << "Device found. Everithing OK ;)" << endl << endl;
			reset();
		}
		else
		{
			device_found = false;
			first_press_fake = true;
			red();
			cerr << endl << "***************WARNING****************"
				<< endl << "******** Device not found! :( ********" << endl << endl;
			reset();
		}

		// Reading the scene and retreive all the object handlers

		// Dummy point graphically associated with the position of the 
		// haptic device in the scene.
		dummy_handler = simGetObjectHandle("Dummy_device");

		// Tool dummy, which is associated to the center a reference object used 
		// to calculate offset required to perform the clutching.
		tool_handler = simGetObjectHandle("Dummy_tool");

		// Retrieve the dummy attached to the tip of the tool; 
		// this is a reference object, and represents the DESIRED POSE
		// to which the lwr_EE will converge when the button 1 or 2 is pressed
		tool_tip_handler = simGetObjectHandle("Dummy_tool_tip");

		// Retrieve the dummy attached to the lwr_ee tip
		lwr_tip_handler = simGetObjectHandle("LWR_tip");

		// Retrieve the handler of the target dummy, which will be the
		// DESIRED POSE of the lwr_ee during penetration (so only when
		// button 3) is pressed
		lwr_target_handler = simGetObjectHandle("LWR_Target");

		lwr_needle_handler = simGetObjectHandle("Needle");


		// -------------------- Peter ------------------------
		// Handles
		dummyHandle = simGetObjectHandle("Dummy_device");
		dummyToolTipHandle = simGetObjectHandle("Dummy_tool_tip");
		phantomHandle = simGetObjectHandle("_Phantom");
		needleHandle = simGetObjectHandle("Needle");
		needleTipHandle = simGetObjectHandle("Needle_tip");
		lwrTipHandle = simGetObjectHandle("LWR_tip");

		forceGraphHandle = simGetObjectHandle("Force_Graph");
		needleForceGraphHandle = simGetObjectHandle("Needle_force_graph");
		penetrationLengthGraphHandle = simGetObjectHandle("Penetration_length_graph");


		// Init full penetration to 0
		fullPenetrationLength = 0.0;
		// Init all tissues to respondable
		setAllTissuesRespondable();
		// Retrieve all tissue names
		initAllTissueNames();

		// -------------------- Peter end --------------------

		//! In order to unbound the angular error
		lwr_tip_T.setZero();
		target_T.setZero();

		// Graphs handler
		force_displ_graph_handler	= simGetObjectHandle("Force_displ_graph");
		force_graph_handler			= simGetObjectHandle("Force_graph");
		ext_force_graph_handler		= simGetObjectHandle("Ext_force_graph");

		// If there is no device, this will move randomly the robot (WHY? Boh) 
		if (device_found)
		{
			// Read the state of the haptic device (position, rotation, velocity)
			chai3DReadState(DEVICE_IDX, device_state);
		}
		else
		{
			// IF THE DEVICE IS NOT FOUND, KEEP UPDATING THE DEVICE STATE AS THE DEVICE IS MOVING.
			// Simulate the movement of the device and initialize device_state
			// take the first device state position
			simGetObjectPosition(dummy_handler, -1, initial_dev_pos);
			initial_dev_pos[0] /= resolution;
			initial_dev_pos[1] /= resolution;
			initial_dev_pos[2] /= resolution;
			moveFakeDevice(device_state);
		}

		// Set Dummy device pose as that of the device state
		// Dummy device is just a representation of where the haptic interface is.
		Matrix4f temp = device_state.T;
		temp.block<3, 1>(0, 3) *= resolution;
		float scaled_device_T[12];
		eigen2SimTransf(temp, scaled_device_T);
		simSetObjectMatrix(dummy_handler, -1, scaled_device_T);

		// retrieve JOINT HANDLERS
		std::string temp_name;
		for (int i = 1; i < 8; i++)
		{
			temp_name = "LBR4p_joint" + std::to_string(i);
			yellow();
			cout << temp_name + " initialized" << endl;
			reset();
			lwr_joint_handlers.push_back(simGetObjectHandle(temp_name.c_str()));
		}

		//! get raw data for the null_space_velocity
		float sim_lwr_init_q[7];
		for (int i = 0; i < 7; i++)
			simGetJointPosition(lwr_joint_handlers[i], &sim_lwr_init_q[i]);
		sim2EigenVec7f(sim_lwr_init_q, lwr_init_q);

		file_DOP_force.open("00_GEOMAGIC_file_DOP_force.txt");
		file_time_forces.open("01_GEOMAGIC_file_time_forces.txt");
		file_perforation_error.open("02_GEOMAGIC_file_perforation_error.txt");
		file_contacts_error.open("03_GEOMAGIC_file_contacts_error.txt");

		file_DOP_force << "DOP, Force\n";
		file_time_forces << "Time, F.x, F.y, F.z, Key\n";
		file_perforation_error << "Time, Error [mm]\n";
		file_contacts_error << "Time, Error [mm]\n";

		green();
		cout << "Finish setup" << endl;
		reset();
	}




	// ------------------------------------------------------------------------- //
	// ------------------------------------------------------------------------- //
	// ---------------------- SIMULATION ADVANCING RUNNING ----------------------//
	// -------- this function is run at every graphics rendering loop ---------- //
	// ------------------------------------------------------------------------- //
	// ------------------------------------------------------------------------- //
	if (simGetSimulationState() == sim_simulation_advancing_running)
	{
		time_step = simGetSimulationTimeStep();

		if (device_found)
		{
			// Read the state of the haptic device 
			// (position, rotation, velocity in the virtual RF)
			chai3DReadState(DEVICE_IDX, device_state);
			//filterVelocity(device_vel_vector, device_state.vel, device_state.vel, false);
		}
		else
		{
			// Simulate the movement of the device and initialize device_state
			moveFakeDevice(device_state);
		}

		//! For exporting data into external files
		std::chrono::duration<double> delta_t;
		float perforation_error;
		float contact_error;
		float curr_DOP;
		int curr_layer_idx;
		float th = 0;
		float rel_th = 0;

		// Peter: Which objects in the simulation do these correspond to?
		float sim_lwr_tip_vel[3];
		float sim_lwr_tip_omega[3];
		float sim_tool_vel[3];
		float sim_tool_omega[3];
		float sim_dummy_T[12];



		// Scaling by resolution
		dummy_T = device_state.T;
		dummy_T.block<3, 1>(0, 3) *= resolution;
		eigen2SimTransf(dummy_T, sim_dummy_T);
		simSetObjectMatrix(dummy_handler, -1, sim_dummy_T);
		
		//! Buttons
		button = chai3DGetButton(DEVICE_IDX);

		float sim_lwr_tip_pos[3];
		Vector3f lwr_tip_pos;
		// Get position of dummy attached to tip --- Essentially: Get position of needle tip
		simGetObjectPosition(lwr_tip_handler, -1, sim_lwr_tip_pos);
		sim2EigenVec3f(sim_lwr_tip_pos, lwr_tip_pos);
		float sim_tool_tip_lin_vel[3];
		float sim_tool_tip_ang_vel[3];
		Vector3f tool_tip_lin_vel, tool_tip_ang_vel;

		// Se sei nel tessuto costringi il bottone ad essere 3;
		// If you are in the tissue, you force the button to be 3
		int current_layer_IDX = -1;
		if (contact_points.size() != 0)
		{
			//current_layer_IDX = tis.getLayerIDXFromDepth(contact_points[0], lwr_tip_pos, contact_normals[0]);
			//if (current_layer_IDX != -1)
			if (punctures.size() == 0)
				button = 3;
		}

		if (punctures.size() == 0)
		{
			fullPenetrationLength = 0.0;
			virtualFixture = false;
		}
		else
			virtualFixture = true;

		//Peter
		updateNeedleState();
		checkPunctures();
		checkContacts();
		modelExternalForces(forceModel);
		setGraphs();

		// If a new button is clicked and the last button was not 0
		// What does 0 mean? No button clicked?
		if (prev_button != button && prev_button != 0)
		{
			first_press = true;
			simSetObjectParent(tool_handler, -1, true);
			simSetObjectParent(tool_tip_handler, tool_handler, true);
			
			// Set lwr_target invisible and tool tip visible.
			simSetObjectIntParameter(lwr_target_handler, sim_objectproperty_selectinvisible, 0);
			simSetObjectIntParameter(tool_tip_handler, sim_objectproperty_selectinvisible, 1);

			// Se sei fuori dal tessuto allora puoi azzerare le cose e il bottone
			// If you are outside of the tissue you can reset things and the button

			// (old code): prev_button != button is already checked on last if
			// This means that if last button was 3 and now we click new button this happens
			// Apparently resets the tissue
			if (prev_button != button && prev_button == 3)
			{
				// clear contact points
				contact_points.clear();
				//tis.resetRendering();
				green();
				cout << "Reset tissue" << endl << endl;
				reset();
				simResetGraph(force_displ_graph_handler);

				// lwr target is the target of the needle (tip (it is a dummy)) during penetration.
				float lwr_target_T[12];
				simGetObjectMatrix(lwr_target_handler, -1, lwr_target_T);

				// Here the next 5 lines we revert the order of the tool and the tip, set the position to the sane as the lwr target.
				// Set tool tip orphan
				simSetObjectParent(tool_tip_handler, -1, true);
				// Set tool tip as parent of tool
				simSetObjectParent(tool_handler, tool_tip_handler, true);

				// Orients the tool tip to the same as the lwr target
				simSetObjectMatrix(tool_tip_handler, -1, lwr_target_T);
				// Revert back to normal
				simSetObjectParent(tool_handler, -1, true);
				simSetObjectParent(tool_tip_handler, tool_handler, true);

			}
		}

		// Here we have the funcitonality of the different buttons
		// 3 is apparently for penetration
		switch (button)
		{
		case 1:
			// If button was pressed for the first time (is set above)
			if (first_press)
			{
				contact_points.clear();

				simSetObjectParent(tool_handler, -1, true);
				simSetObjectParent(tool_tip_handler, tool_handler, true);
				getOffset();

				// clear velocity buffers
				Vector3f zero_tmp;
				zero_tmp.setZero();
				LPFilter(device_vel_vector, zero_tmp, device_mean_vel_vector, device_LPF_vel, true);
				LPFilter(lwr_tip_vel_vector, zero_tmp, lwr_tip_mean_vel_vector, lwr_tip_LPF_vel, true);
				LPFilter(lwr_tip_omega_vector, zero_tmp, lwr_tip_mean_omega_vector, lwr_tip_LPF_omega, true);

				LPFilter(tool_tip_vel_vector, zero_tmp, tool_tip_mean_vel_vector, tool_tip_LPF_vel, true);
				LPFilter(tool_tip_omega_vector, zero_tmp, tool_tip_mean_omega_vector, tool_tip_LPF_omega, true);

				first_press = false;
			}
			LPFilter(device_vel_vector, device_state.vel, device_mean_vel_vector, device_LPF_vel, false);

			// tool_handler is "Dummy_tool" in v-rep
			simGetObjectVelocity(tool_handler, sim_tool_vel, sim_tool_omega);
			sim2EigenVec3f(sim_tool_vel, tool_vel);
			sim2EigenVec3f(sim_tool_omega, tool_omega);

			LPFilter(tool_tip_vel_vector, tool_vel, tool_tip_mean_vel_vector, tool_tip_LPF_vel, false);
			LPFilter(tool_tip_omega_vector, tool_omega, tool_tip_mean_omega_vector, tool_tip_LPF_omega, false);

			//updatePose();
			//updateRobotPose(tool_tip_handler, tool_tip_LPF_vel, tool_tip_LPF_omega);
			computeGlobalForce();
			break;
		case 2:
			if (first_press)
			{
				contact_points.clear();

				simSetObjectParent(tool_tip_handler, -1, true);
				simSetObjectParent(tool_handler, tool_tip_handler, true);
				getOffset();
				first_press = false;
			}
			//updateRot();
			simGetObjectVelocity(tool_tip_handler, sim_tool_tip_lin_vel, sim_tool_tip_ang_vel);
			sim2EigenVec3f(sim_tool_tip_lin_vel, tool_tip_lin_vel);
			sim2EigenVec3f(sim_tool_tip_ang_vel, tool_tip_ang_vel);
			//updateRobotPose(tool_tip_handler, tool_tip_lin_vel, tool_tip_ang_vel);
			break;
		case 3:
			if (first_press)
			{
				simSetObjectIntParameter(lwr_target_handler, sim_objectproperty_selectinvisible, 1);
				simSetObjectIntParameter(tool_tip_handler, sim_objectproperty_selectinvisible, 0);
				float sim_lwr_tip_T[12];
				float sim_tool_tip_T[12];
				Matrix4f tool_tip_T;
				simGetObjectMatrix(lwr_tip_handler, -1, sim_lwr_tip_T);
				simGetObjectMatrix(tool_tip_handler, -1, sim_tool_tip_T);

				sim2EigenTransf(sim_lwr_tip_T, lwr_tip_init_T);
				sim2EigenTransf(sim_tool_tip_T, tool_tip_T);

				tool_tip_init_dir = tool_tip_T.block<3, 1>(0, 2);
				tool_tip_init_pos = tool_tip_T.block<3, 1>(0, 3);

				//! Velocity filtering: clear velocity buffers
				Vector3f zero_tmp;
				zero_tmp.setZero();
				LPFilter(device_vel_vector, zero_tmp, device_mean_vel_vector, device_LPF_vel, true);

				LPFilter(lwr_tip_vel_vector, zero_tmp, lwr_tip_mean_vel_vector, lwr_tip_LPF_vel, true);
				LPFilter(lwr_tip_omega_vector, zero_tmp, lwr_tip_mean_omega_vector, lwr_tip_LPF_omega, true);

				getOffset();
				first_press = false;

				//! to retreive data
				green();
				cout << "PRESS 'A' WHEN YOU FEEL THE CONTACT WITH A LAYER" << endl;
				cout << "PRESS 'S' WHEN YOU FEEL THE PERFORATION OF A LAYER" << endl;
				reset();
			}

			//! Velocity filtering
			LPFilter(device_vel_vector, device_state.vel, device_mean_vel_vector, device_LPF_vel, false);

			simGetObjectVelocity(lwr_tip_handler, sim_lwr_tip_vel, sim_lwr_tip_omega);
			sim2EigenVec3f(sim_lwr_tip_vel, lwr_tip_vel);
			sim2EigenVec3f(sim_lwr_tip_omega, lwr_tip_omega);

			LPFilter(lwr_tip_vel_vector, lwr_tip_vel, lwr_tip_mean_vel_vector, lwr_tip_LPF_vel, false);
			LPFilter(lwr_tip_omega_vector, lwr_tip_omega, lwr_tip_mean_omega_vector, lwr_tip_LPF_omega, false);

			//updatePose();
			//updatePosPenetration();
			//updateRobotPose(lwr_target_handler, lwr_tip_LPF_vel, lwr_tip_LPF_omega);
			manageContact();
			lwr_tip_external_F.setZero();
			if (contact_points.size() > 0)
			{
				modelExternalForces("kelvin-voigt");
				//computeExternalForce(lwr_tip_external_F, lwr_tip_pos, contact_points, contact_normals[0], lwr_tip_LPF_vel); //! QUI
				/*
				//! Retreive data to export in matlab
				curr_DOP = tis.getDOP(contact_points[0], lwr_tip_pos, contact_normals[0]);
				curr_layer_idx = tis.getLayerIDXFromDepth(contact_points[0], lwr_tip_pos, contact_normals[0]);
				th = 0;
				rel_th = 0;

				if ((GetAsyncKeyState('a') & 0x8000)
					|| (GetAsyncKeyState('A') & 0x8000))
				{
					//! write into contact file
					for (int i = 0; i < curr_layer_idx; i++)
						th += thick(i);
					contact_error = th - curr_DOP;
					t_1 = std::chrono::high_resolution_clock::now();
					delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t_1 - t_0);
					file_contacts_error << delta_t.count() << ", " << contact_error << "\n";

					//! Write into time-forces file
					file_time_forces << delta_t.count() << ", " <<
						lwr_tip_external_F.x() << ", " <<
						lwr_tip_external_F.y() << ", " <<
						lwr_tip_external_F.z() << ", " <<
						'A' << "\n";
				}
				else if ((GetAsyncKeyState('s') & 0x8000)
					|| (GetAsyncKeyState('S') & 0x8000))
				{
					//! write into perforation file
					for (int i = 0; i < curr_layer_idx; i++)
						rel_th += thick(i);
					rel_th += p_thick(curr_layer_idx);
					perforation_error = rel_th - curr_DOP;

					t_1 = std::chrono::high_resolution_clock::now();
					delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t_1 - t_0);

					file_perforation_error << delta_t.count() << ", " << perforation_error << "\n";

					//! Write into time-forces file
					file_time_forces << delta_t.count() << ", " <<
						lwr_tip_external_F.x() << ", " <<
						lwr_tip_external_F.y() << ", " <<
						lwr_tip_external_F.z() << ", " <<
						'S' << "\n";
				}
				else
				{
					t_1 = std::chrono::high_resolution_clock::now();
					delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t_1 - t_0);

					//! Write into time-forces file
					file_time_forces << delta_t.count() << ", " <<
						lwr_tip_external_F.x() << ", " <<
						lwr_tip_external_F.y() << ", " <<
						lwr_tip_external_F.z() << ", " <<
						'X' << "\n";
				}
				*/
			}
			computeGlobalForce();
			

			break;
		default:
			first_press = true;
			simSetObjectParent(tool_handler, -1, true);
			simSetObjectParent(tool_tip_handler, tool_handler, true);

			contact_points.clear();

			global_device_force.setZero();

			simSetObjectIntParameter(lwr_target_handler, sim_objectproperty_selectinvisible, 0);
			simSetObjectIntParameter(tool_tip_handler, sim_objectproperty_selectinvisible, 1);
		}

		prev_button = button;


		if (!device_found)
		{
			if (first_press_fake)
			{
				simSetObjectParent(tool_handler, -1, true);
				simSetObjectParent(tool_tip_handler, tool_handler, true);
				getOffset();

				first_press_fake = false;
			}
			//updatePose();
			computeGlobalForce();
		}


		// COUT
		//want_to_print = true;

		if (global_cnt % 100 == 0 && want_to_print)
		{
			cyan();
			cout << endl << "Epoch: \t" << global_cnt << endl;
			cout << "Button idx: \t" << button << endl;
			reset();

		}
		global_cnt++;
	}




	// ------------------------------------------------------------------------- //
	// ------------------------------------------------------------------------- //
	// ----------------- WHEN THE USER CLICKS THE STOP BUTTON -------------------//
	// ----------------- this function is executed only once ------------------- //
	// ------------------------------------------------------------------------- //
	// ------------------------------------------------------------------------- //
	if (message == sim_message_eventcallback_simulationended)
	{
		hapticReset();
		file_DOP_force.close();
		file_perforation_error.close();
		file_contacts_error.close();
		file_time_forces.close();
		red();
		cout << "*********************************" << endl;
		cout << "Sim stopped, device disconnected" << endl;
		reset();

		reactivateTissues();
	}
	return retVal;
}





// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
// ------------------------- FUNCTIONS DEFINITIONS --------------------------//
// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //

/**
 * Maps the rotation of the dummy to the tool tip.
 * Updates the rotation of the tool tip in v-rep.
 */
void updateRot(void)
{
	Matrix4f tool_tip_T, temp;
	Matrix4f dummy_T;
	float sim_tool_tip_T[12];
	float sim_tool_tip_angles[3];
	float sim_dummy_T[12];

	simGetObjectMatrix(dummy_handler, -1, sim_dummy_T);
	sim2EigenTransf(sim_dummy_T, dummy_T);

	tool_tip_T = dummy_T;
	tool_tip_T.block<3, 3>(0, 0) = tool_tip_T.block<3, 3>(0, 0) * rot_offset;

	eigen2SimTransf(tool_tip_T, sim_tool_tip_T);
	simGetEulerAnglesFromMatrix(sim_tool_tip_T, sim_tool_tip_angles);
	// Update tool tip orientation in v-rep
	simSetObjectOrientation(tool_tip_handler, -1, sim_tool_tip_angles);
}


void updatePosPenetration(void)
{
	Vector3f tool_tip_pos, lwr_target_proj_pos, tool_tip_relative_increment;
	float lwr_target_proj_magnitude;
	float sim_tool_tip_pos[3];
	float sim_lwr_tip_curr_T[12];
	Matrix4f lwr_tip_curr_T = lwr_tip_init_T;
	Vector3f lwr_tip_dir;
	Vector3f lwr_tip_init_pos;

	lwr_tip_dir = lwr_tip_init_T.block<3, 1>(0, 2);
	lwr_tip_init_pos = lwr_tip_init_T.block<3, 1>(0, 3);

	simGetObjectPosition(tool_tip_handler, -1, sim_tool_tip_pos);
	sim2EigenVec3f(sim_tool_tip_pos, tool_tip_pos); 

	tool_tip_relative_increment = tool_tip_pos - tool_tip_init_pos;
	lwr_target_proj_magnitude = tool_tip_relative_increment.dot(tool_tip_init_dir);
	lwr_target_proj_pos = lwr_tip_init_pos + lwr_target_proj_magnitude * lwr_tip_dir;

	lwr_tip_curr_T.block<3, 1>(0, 3) = lwr_target_proj_pos;
	eigen2SimTransf(lwr_tip_curr_T, sim_lwr_tip_curr_T);
	simSetObjectMatrix(lwr_target_handler, -1, sim_lwr_tip_curr_T);


	return; //placeholder
}

/**
 * Set pose of tool
 */
void updatePose(void)
{
	Vector3f tool_pos;
	Matrix3f tool_rot;
	Matrix4f tool_T, dummy_T;
	float sim_tool_T[12];
	float sim_dummy_T[12];

	simGetObjectMatrix(dummy_handler, -1, sim_dummy_T);
	sim2EigenTransf(sim_dummy_T, dummy_T);

	// tool_pos in this way is already multiplied by resolution
	// --> in advancing_running
	tool_pos = dummy_T.block<3,1>(0,3) + lin_offset; 
	tool_rot = dummy_T.block<3,3>(0, 0) * rot_offset;

	tool_T.block<3, 1>(0, 3) = tool_pos;
	tool_T.block<3, 3>(0, 0) = tool_rot;
	eigen2SimTransf(tool_T, sim_tool_T);
	simSetObjectMatrix(tool_handler, -1, sim_tool_T);
}

void getOffset(void)
{
	Matrix4f dummy_T;
	Matrix4f tool_T;
	Vector3f tool_pos;
	Vector3f dummy_pos;
	Matrix3f dummy_rot;
	Matrix3f tool_rot;
	
	float sim_dummy_T[12];
	float sim_tool_T[12];

	simGetObjectMatrix(dummy_handler, -1, sim_dummy_T);
	simGetObjectMatrix(tool_handler, -1, sim_tool_T);
	
	//! Position
	sim2EigenTransf(sim_dummy_T, dummy_T);
	sim2EigenTransf(sim_tool_T, tool_T);
	dummy_pos = dummy_T.block<3, 1>(0, 3);
	tool_pos = tool_T.block<3, 1>(0, 3);

	// already scaled by resolution
	lin_offset = tool_pos - dummy_pos;

	//! Orientation
	dummy_rot = dummy_T.block<3, 3>(0, 0);
	tool_rot = tool_T.block<3, 3>(0, 0);

	rot_offset = dummy_rot.transpose() * tool_rot;
	force_offset_rot = dummy_rot * tool_rot.transpose();
}


void computeGlobalForce(void)
{
	Vector3f tool_tip_F;
	Vector3d temp_v;
	Matrix4f dummy_T;
	Matrix3f dummy_R;

	float sim_dummy_T[12];

	simGetObjectMatrix(dummy_handler, -1, sim_dummy_T);
	sim2EigenTransf(sim_dummy_T, dummy_T);
	dummy_R = dummy_T.block<3, 3>(0, 0);

	tool_tip_F = dummy_R.col(0); // forza lungo l'asse x del dummy (sempre)	
	

	//! Because the Device dummy is mapped on the center of the needle
	float sim_lwr_needle_pos[3];
	Vector3f lwr_needle_pos;
	Vector3f x_md, x_md_dot;

	// Peter
	// To use my external force, use externalForce instead of lwr_tip_external_F it looks like.
	// A simpler way would maybe just to set the global force to the external force.
	// Why do they apply K and B again? They apply this when calculating external force
	
	//! TELEOPERATION
	switch (controller_ID)
	{
	case 1:
		// Pos/Force-Pos (Non-uniform matrix port)
		// lwr_tip_external force calculated in computeExternalForce
		// Diff device LPF and LWR Tip LPF is the LPFiltered velocity difference of the device and the lwr_tip
		
		F_mc = (K_m * lwr_tip_external_F) - (B_m * (device_LPF_vel - lwr_tip_LPF_vel));
		break;
	case 2:
		// Pos / Pos
		//! F_mc = -K_m(x_m - x_md) - B_m(x_m_DOT - x_md_DOT) con x_md = x_s - x_offset
		//! x_md -> proiezione di x_s nel 'device space'.
		simGetObjectPosition(lwr_needle_handler, -1, sim_lwr_needle_pos);
		sim2EigenVec3f(sim_lwr_needle_pos, lwr_needle_pos);

		x_md = (lwr_needle_pos - lin_offset)*(1 / resolution);
		x_md_dot = lwr_tip_LPF_vel * (1 / resolution);

		F_mc = -K_m * (device_state.pos - x_md) - B_m * (device_LPF_vel - x_md_dot);
		break;
	case 3:
		// Pos / Force
		F_mc = K_m * lwr_tip_external_F;
		break;
	default:
		// Null force (just in case)
		F_mc.setZero();
		break;
	}

	temp_v = F_mc.cast<double>();
	if(ABILITA_LE_FORZE)
		global_device_force = temp_v;

	simSetGraphUserData(force_graph_handler, "F_x", (float)global_device_force.x());
	simSetGraphUserData(force_graph_handler, "F_y", (float)global_device_force.y());
	simSetGraphUserData(force_graph_handler, "F_z", (float)global_device_force.z());
	simSetGraphUserData(force_graph_handler, "Magnitude", (float)global_device_force.norm());
}


void filterVelocity(Vector3fVector& v_vector, Vector3f& new_vel, Vector3f& mean_vel, bool init)
{
	if (init)
	{
		for (int i = 0; i < buffer_vel_size; i++)
			v_vector[i] = new_vel;
		mean_vel = new_vel;
	}
	else
	{
		for (int i = buffer_vel_size - 1; i > 0; i--)
			v_vector[i] = v_vector[i - 1];

		v_vector[0] = new_vel;

		mean_vel.setZero();
		for (int j = 0; j < buffer_vel_size; j++)
			mean_vel += v_vector[j];
		mean_vel = mean_vel / (const float)buffer_vel_size;
	}
}


void LPFilter(Vector3fVector& v_vector, Vector3f& new_vel, Vector3fVector& mean_vel_vector, Vector3f& LPF_vel, bool init)
{
	if (init)
	{
		if (v_vector.size() != mean_vel_vector.size())
		{
			red();
			cerr << "Error, different vectors size" << endl;
			reset();
			return;
		}

		for (int i = 0; i < buffer_vel_size; i++)
			v_vector[i] = new_vel;

		for (int i = 0; i < buffer_vel_size; i++)
			mean_vel_vector[i] = new_vel;

		LPF_vel.setZero();
	}
	else
	{
		//! Il filtraggio potrebbe dover essere fatto nell'haptic loop 
		//! poich� originariamente richiede una frequenza di aggiornamento di 1000Hz;
		float update_freq = 1000.0f;
		//! Questo parametro dipende dalla frequenza (come sopra)
		//! Modificarlo?!
		float alpha = (0.001f / (0.001f + 0.07f));


		Vector3f mean_vel;
		mean_vel = (0.2196f *(new_vel + v_vector[2]) +
			0.6588f * (v_vector[0] + v_vector[1])) / update_freq -
			(-2.7488f * mean_vel_vector[0] + 2.5282f * mean_vel_vector[1] - 0.7776f * mean_vel_vector[2]);

		// Input Vector Shift
		for (int i = buffer_vel_size - 1; i > 0; i--)
			v_vector[i] = v_vector[i - 1];
		v_vector[0] = new_vel;
		// Output Vector Shift
		for (int i = buffer_vel_size - 1; i > 0; i--)
			mean_vel_vector[i] = mean_vel_vector[i - 1];
		mean_vel_vector[0] = mean_vel;

		// LPF
		LPF_vel = LPF_vel + (mean_vel_vector[0] - LPF_vel)*alpha;
	}
}




void readUI(void)
{

	UI_handler = simGetUIHandle("UI");

	button_handler = simGetUIEventButton(UI_handler, aux_val);

	switch (button_handler)
	{
	case 3:
		controller_ID = 1;
		current_controller = simGetUIButtonLabel(UI_handler, button_handler);
		simSetUIButtonLabel(UI_handler, 27, current_controller, NULL);
		break;
	case 12:
		controller_ID = 2;
		current_controller = simGetUIButtonLabel(UI_handler, button_handler);
		simSetUIButtonLabel(UI_handler, 27, current_controller, NULL);
		break;
	case 21:
		controller_ID = 3;
		current_controller = simGetUIButtonLabel(UI_handler, button_handler);
		simSetUIButtonLabel(UI_handler, 27, current_controller, NULL);
		break;
	case 39:
		if (simGetSimulationState() == sim_simulation_advancing_running)
			simStopSimulation();

		cout << "Check Values" << endl;
		checkAndSetValues(controller_ID, UI_handler);
		simStartSimulation();
		break;
	default:
		break;
	}
}


void readLayersUI(void)
{

	int size = UI_layers_names.size();
	LayersUI_handler = simGetUIHandle("UI0");
	button_handler = simGetUIEventButton(LayersUI_handler, aux_val);

	float tmp[4] = {0.0f,0.0f,0.0f,0.0f};
	
	switch (button_handler)
	{
	case 14: 
		use_default_tissue_values = true;
		break;
	case 8:
		readLayerParams(UI_layer_idx, tmp, LayersUI_handler);
		UI_layer_idx = (UI_layer_idx + size -1) % size;
		yellow();
		cout << UI_layer_idx << endl;
		reset();
		loadLayerParams(UI_layer_idx);
		break;
	case 9:
		cout << "pre read" << endl;
		readLayerParams(UI_layer_idx, tmp, LayersUI_handler);
		UI_layer_idx = (UI_layer_idx + 1) % size;
		yellow();
		cout << UI_layer_idx << endl;
		reset();
		loadLayerParams(UI_layer_idx);
		break;
	default:
		break;
	}


}

void loadLayerParams(int layer_idx)
{
	simSetUIButtonLabel(LayersUI_handler, 3, UI_layers_names[layer_idx].c_str(), NULL);
	simSetUIButtonLabel(LayersUI_handler, 10, std::to_string(UI_thick_vec(layer_idx)).c_str(), NULL);
	simSetUIButtonLabel(LayersUI_handler, 11, std::to_string(UI_K_vec(layer_idx)).c_str(), NULL);
	simSetUIButtonLabel(LayersUI_handler, 12, std::to_string(UI_B_vec(layer_idx)).c_str(), NULL);
	simSetUIButtonLabel(LayersUI_handler, 13, std::to_string(UI_p_t_p_vec(layer_idx)).c_str(), NULL);
}

void readLayerParams(int layer_idx, float* tmp, int UI_handler)
{
	checkValues(10, 13, tmp, 0.50, UI_handler);
	green();
	cout << "check done" << endl;
	UI_thick_vec(layer_idx) = tmp[0];
	UI_K_vec(layer_idx) = tmp[1];
	UI_B_vec(layer_idx) = tmp[2];
	UI_p_t_p_vec(layer_idx) = tmp[3];

	reset();
}

void checkAndSetValues(int ID, int UI_handler)
{
	float tmp[3];
	K_m.setIdentity();
	B_m.setIdentity();

	if (ID == 1)
	{
		checkValues(6, 8, tmp, 1.0, UI_handler);
		K_m(0, 0) = tmp[0];
		K_m(1, 1) = tmp[1];
		K_m(2, 2) = tmp[2];

		checkValues(9, 11, tmp, 1.0, UI_handler);
		B_m(0, 0) = tmp[0];
		B_m(1, 1) = tmp[1];
		B_m(2, 2) = tmp[2];
	}
	else if (ID == 2)
	{
		checkValues(15, 17, tmp, 1.0, UI_handler);
		K_m(0, 0) = tmp[0];
		K_m(1, 1) = tmp[1];
		K_m(2, 2) = tmp[2];

		checkValues(18, 20, tmp, 1.0, UI_handler);
		B_m(0, 0) = tmp[0];
		B_m(1, 1) = tmp[1];
		B_m(2, 2) = tmp[2];
	}
	else if (ID == 3)
	{
		checkValues(24, 26, tmp, 1.0, UI_handler);
		K_m(0, 0) = tmp[0];
		K_m(1, 1) = tmp[1];
		K_m(2, 2) = tmp[2];
	}
	simSetUIButtonLabel(UI_handler, 27, current_controller, NULL);
}

void checkValues(int init_val, int fin_val, float* param, float default_value, int UI_handler) 
{
	simChar* label;

	char buff[256];
	for (int i = init_val; i <= fin_val; i++) 
	{
		label = simGetUIButtonLabel(UI_handler, i);

		string data(label);
		if(data == "") 
		{
			param[i - init_val] = default_value;
			sprintf(buff, "%.4f", default_value);
			simSetUIButtonLabel(UI_handler, i, buff, NULL);
		}
		else
		{
			string::size_type sz;
			float parsed_data = stof(data, &sz);
			sprintf(buff, "%.4f", parsed_data);
			simSetUIButtonLabel(UI_handler, i, buff, NULL);
			param[i - init_val] = parsed_data;
		}

	}

}


void manageContact(void)
{
	//objects_in_contact contact_point;
	if (objects_in_contact[1] == tis.getLayerHandler("Skin", true))
	{
		contact_points.push_back(contact_point);
		contact_normals.push_back(contact_normal);
		tis.removeDynamicLayer("Skin");
		tis.toggleTouched("Skin");

		//END
		objects_in_contact[1] = -1;
	}
	else if (objects_in_contact[1] == tis.getLayerHandler("Fat", true))
	{
		contact_points.push_back(contact_point);
		contact_normals.push_back(contact_normal);
		tis.removeDynamicLayer("Fat");
		tis.toggleTouched("Fat");

		//END
		objects_in_contact[1] = -1;
	}
	else if (objects_in_contact[1] == tis.getLayerHandler("Muscle", true))
	{
		contact_points.push_back(contact_point);
		contact_normals.push_back(contact_normal);
		tis.removeDynamicLayer("Muscle");
		tis.toggleTouched("Muscle");

		//END
		objects_in_contact[1] = -1;
	}
	else if (objects_in_contact[1] == tis.getLayerHandler("Bone", true))
	{
		contact_points.push_back(contact_point);
		contact_normals.push_back(contact_normal);
		tis.removeDynamicLayer("Bone");
		tis.toggleTouched("Bone");

		//END
		objects_in_contact[1] = -1;
	}
	else
		return;
		//tis.printTissue();
}

void moveFakeDevice(DeviceState& state)
{
	// update device_state as if it is moved by the device.
	// circular trajectory (roller coaster like)
	float oriz_radius = (float)0.04;
	float vert_radius = (float) 0.01;
	float T = 80; // period
	float t = simGetSimulationTime();
	float angle =  (float)(t * (2.0f * M_PI) / T);
	state.pos(0) = (float)initial_dev_pos[0] + oriz_radius * cos(angle);
	state.pos(1) = (float)initial_dev_pos[1] + oriz_radius * sin(angle);
	state.pos(2) = (float)initial_dev_pos[2] + vert_radius * sin(angle);

	state.vel(0) = (float)-oriz_radius * sin(angle);
	state.vel(1) = (float)oriz_radius * cos(angle);
	state.vel(2) = (float)oriz_radius * cos(angle);

	state.rot.setZero();
	state.rot(0, 0) = sin(angle);
	state.rot(1, 0) = -cos(angle);
	state.rot(0, 1) = cos(angle);
	state.rot(1, 1) = sin(angle);
	state.rot(2, 2) = 1;

	state.T.block<3, 3>(0, 0) = state.rot;
	state.T(0, 3) = state.pos(0);
	state.T(1, 3) = state.pos(1);
	state.T(2, 3) = state.pos(2);
}


void updateRobotPose(int target_handler, Vector3f target_lin_vel, Vector3f target_ang_vel)
{
	Vector6f target_r_dot_d;

	Vector7f lwr_current_q, lwr_q;
	Matrix6_7f lwr_J;

	//! GAIN for cartesian correction. 
	//! NB: block<3,3>(0,0) -> linear part;
	//! NB: block<3,3>(3,3) -> angular part;
	Matrix6f K_p;
	K_p.setIdentity();
	K_p.block<3, 3>(0, 0) *= 4.2f;
	K_p.block<3, 3>(3, 3) *= 2.5f;

	float sim_target_T[12];

	float sim_lwr_tip_T[12];
	float sim_lwr_current_q[7];
	float sim_lwr_q[7];
	
	//! In order to unbound the angular error
	prev_lwr_tip_T = lwr_tip_T;
	prev_target_T = target_T;

	//! get raw data
	for (int i = 0; i < 7; i++)
		simGetJointPosition(lwr_joint_handlers[i], &sim_lwr_current_q[i]);
	sim2EigenVec7f(sim_lwr_current_q, lwr_current_q);

	// target pose
	simGetObjectMatrix(target_handler, -1, sim_target_T);
	sim2EigenTransf(sim_target_T, target_T);


	// lwr_tip pose
	simGetObjectMatrix(lwr_tip_handler, -1, sim_lwr_tip_T);
	sim2EigenTransf(sim_lwr_tip_T, lwr_tip_T);

	target_r_dot_d << target_lin_vel, target_ang_vel;

	//! Jac
	lwr_J = LWRGeometricJacobian(lwr_current_q);
	
	//! Computing null-space velocity
	Vector7f lwr_q_0_dot = (lwr_init_q - lwr_current_q);

	//! Compute q_dot
	computeNullSpaceVelocity(lwr_q_dot, lwr_q_0_dot, target_r_dot_d,
		prev_target_T, prev_lwr_tip_T, target_T, lwr_tip_T, 
		lwr_J, K_p);
	//computeDLSVelocity(lwr_q_dot, target_r_dot_d,
	//	prev_target_T, prev_lwr_tip_T, target_T, lwr_tip_T,
	//	lwr_J, K_p);

	//! Linear integration
	lwr_q = lwr_current_q + lwr_q_dot * time_step;

	eigen2SimVec7f(lwr_q, sim_lwr_q);

	//! Set target joint pos
	for (int i = 0; i < 7; i++)
		simSetJointTargetPosition(lwr_joint_handlers[i], sim_lwr_q[i]);
}



// ------------------------------------------------------------------------- //
// ------------------------------------------------------------------------- //
// ----------------------- Function Definitions------------------------------//
// ------------------------ Peter Cook Bulukin ----------------------------- //
// ------------------------------------------------------------------------- //

/**
* @brief Retrieve puncture related to handle.
* @param handle: handle of object to retrieve.
* @return the puncture of the handle. Returns empty puncture if not found.
*/
sPuncture getPunctureFromHandle(int handle)
{
	for (sPuncture puncture : punctures)
	{
		if (puncture.handle == handle)
		{
			return puncture;
		}
	}
	sPuncture puncture;
	return puncture;
}

/**
* @brief Get puncture from tissue name
* @param name: name of tissue (as set in v-rep)
* @return puncture related to name. Empty puncture if not found.
*/
sPuncture getPunctureFromName(std::string name)
{
	for (sPuncture puncture : punctures)
	{
		if (puncture.name == name)
		{
			return puncture;
		}
	}
	sPuncture puncture;
	return puncture;
}

/**
* @brief Calculate length of a puncture
* @param puncture: puncture
* @return penetration distance of puncture. Value bellow zero means distance is "outside" of the tissue.
*/
float punctureLength(sPuncture puncture)
{
	Vector3f penetration = toolTipPos - puncture.position;
	if (puncture.direction.dot(penetration) > 0)
		return penetration.norm();
	else
		return 0;

}

/**
* @brief Check if the needle is still in the puncture. This is where fullPenetrationLength is incremented.
*/
void checkPunctures()
{
	// Iterate through punctures backwards, because if a puncture still is active, all punctures before will also still be active.
	for (auto it = punctures.rbegin(); it != punctures.rend(); ++it)
	{
		float puncture_length = punctureLength(*it);
		// If puncture length is above zero, all punctures before it in the vector will be unchanged.
		if (checkSinglePuncture(*it) > 0)
		{
			fullPenetrationLength -= it->penetrationLength;
			fullPenetrationLength += puncture_length;
			// This penetration length might have been updated, so update.
			it->penetrationLength = puncture_length;
			if (enableExitTissue) {
				// Set punctures to be from the first puncture up until the current.
				punctures = std::vector<sPuncture>(it, punctures.rend());
				std::reverse(punctures.begin(), punctures.end());
			}
			return;
		}
		else {
			// The needle isn't puncturing this tissue anymore. Set tissue respondable and print.
			if (enableExitTissue)
			{
				setRespondable(it->handle);
				fullPenetrationLength -= it->penetrationLength;
				it->printPuncture(false);
			}
			
		}

	}
	// No punctures had length > 0
	if(enableExitTissue)
		punctures.clear();
}



/**
* @brief Set object respondable
* @param handle: handle
*/
void setRespondable(int handle)
{
	simSetObjectIntParameter(handle, RESPONDABLE, 1);
}

/**
* @brief Set object unrespondable
* @param handle: handle
*/
void setUnRespondable(int handle)
{
	simSetObjectIntParameter(handle, RESPONDABLE, 0);
}

/**
* @brief Check if a puncture is still active.
* @param puncture: puncture
* @return 1 if still active, -1 if not.
*/
int checkSinglePuncture(sPuncture puncture) {
	Vector3f current_translation = toolTipPos - puncture.position;
	// Return 1 if the distance is close to 0 so we get stability close to the puncture.
	if (current_translation.norm() < 1e-4)
		return 1;
	if (puncture.direction.dot(current_translation) >= 0)
		return 1;
	return -1;
}

/**
* @brief Update needle velocity
*/
void updateNeedleVelocity()														// To add Low-pass filter, I think a good place to add it would be here.
{
	simFloat _tmpNeedleVelocity[3];
	simFloat _tmpAngVelocity[3];
	
	if (simGetObjectVelocity(lwrTipHandle, _tmpNeedleVelocity, _tmpAngVelocity) == -1)
		std::cerr << "Needle tip velocity retrieval failed" << std::endl;
	needleVelocity = Vector3f(_tmpNeedleVelocity[0], _tmpNeedleVelocity[1], _tmpNeedleVelocity[2]);
	needleAngularVelocity = Vector3f(_tmpAngVelocity[0], _tmpAngVelocity[1], _tmpAngVelocity[2]);
	needleVelocityMagnitude = needleVelocity.norm();

}

/**
* @brief Update needle direction
*/
void updateNeedleDirection()
{
	simFloat objectMatrix[12];
	simGetObjectMatrix(lwrTipHandle, -1, objectMatrix);
	needleDirection = simObjectMatrix2EigenDirection(objectMatrix).normalized();

}

/**
* @brief Update needle position
*/
void updateNeedleTipPos()
{
	float needleTipPos[3];
	simGetObjectPosition(lwrTipHandle, -1, needleTipPos);
	toolTipPos = Vector3f(needleTipPos[0], needleTipPos[1], needleTipPos[2]);
}

void updateNeedleState()
{
	updateNeedleDirection();
	updateNeedleTipPos();
	updateNeedleVelocity();
}

/**
* @brief Add new puncture to punctures
* @param handle: handle of tissue that was punctured
*/
void addPuncture(int handle)
{
	sPuncture puncture;
	puncture.position = toolTipPos;
	puncture.direction = needleDirection;
	puncture.handle = handle;
	puncture.name = handle2Name[handle];
	puncture.penetrationLength = punctureLength(puncture);
	fullPenetrationLength += puncture.penetrationLength;
	setUnRespondable(handle);
	punctures.push_back(puncture);
	puncture.printPuncture(true);
}

/**
* @brief Check which contacts will result in a puncture
*/
void checkContacts()
{

	lwrTipPhysisEngineForceMagnitude = 0.0;
	lwrTipPhysicsEngineForce.setZero();
	int steps;
	simGetIntegerParameter(sim_intparam_dynamic_step_divider, &steps);

	map<int, Vector3f> forces;
	// Iterate through all the steps of the dynamic engine
	for (int a = 0; a<steps; a++)
	{
		// Get contact info
		simInt contactHandles[2];
		simFloat contactInfo[6];
		int respondableValue;
		simGetContactInfo(sim_handle_all, needleHandle, a, contactHandles, contactInfo);
		simGetObjectIntParameter(contactHandles[1], RESPONDABLE, &respondableValue);
		// The forces are only valid if the object is a child of the phantom. Check also if contact handle is below 1000 because v-rep can return a random number on no contact.
		// Check if object is respondable
		if (contactHandles[1] < 1000 && simGetObjectParent(contactHandles[1]) == phantomHandle && respondableValue != 0)
		{
			// Turn the force into a Vector3f
			Vector3f force = simContactInfo2EigenForce(contactInfo);
			// Calculate force magnitude (maybe do this in the end?)
			map<int, Vector3f>::iterator forceIt = forces.find(contactHandles[1]);
			if (forceIt != forces.end())
				forceIt->second += force;
			else
				forces[contactHandles[1]] = force;

		}
	}

	// Init threshold to be the constant threshold
	float threshold = punctureThreshold;
	map<int, Vector3f>::iterator it;
	for (it = forces.begin(); it != forces.end(); it++)
	{
		int handle = it->first;
		Vector3f force = it->second;

		// Add forces to the total forces
		// Check if full force should be used in force magnitude
		float force_magnitude;
		if (useOnlyZForceOnEngine)
			force_magnitude = generalForce2NeedleTipZ(force);
		else
			force_magnitude = force.norm();
		lwrTipPhysicsEngineForce += force;
		lwrTipPhysisEngineForceMagnitude += force_magnitude;
		// If variable thresholds are used, set K threshold
		if (!constantPunctureThreshold)
		{
			string tissueName = handle2Name[handle];
			threshold = KThresh(tissueName);
		}
		if (force_magnitude > threshold)
		{
			addPuncture(handle);
			std::cout << "Force magnitude: " << force_magnitude << std::endl;
		}
		

	}
}

/**
* @brief Sign function
*/
float sgn(float x) {
	if (x > 0) return 1.0;
	if (x < 0) return -1.0;
	return 0.0;
}

/**
* @brief Model external forces that act upon the needle. Updates externalForceMagnitude and externalForce
* @param model: string describing the model that should be used for modeling the forces
*/
void modelExternalForces(std::string model) {

	if (model == "kelvin-voigt") {
		externalForceMagnitude = kelvinVoigtModel();
	}
	else if (model == "karnopp")
	{
		externalForceMagnitude = karnoppModel() * 0.1;
	}
	else
	{
		std::cout << "No valid friction model was chosen, automatically set Kelvin-Voigt" << std::endl;
		externalForceMagnitude = kelvinVoigtModel();
	}
	externalForceMagnitude *= modelForceScalar;
	// Obtain the forces in the z direction in the reference frame of the lwr needle tip.
	// Add the z force to the magnitude of the forces
	if (useOnlyZForceOnEngine)
		externalForceMagnitude += generalForce2NeedleTipZ(lwrTipPhysicsEngineForce) * engineForceScalar;
	else
		externalForceMagnitude += lwrTipPhysicsEngineForce.norm() * engineForceScalar;
	// Get direction of the dummy so that the forces get distributed on all the axis. (They did this in the other project, but is this correct?)
	// Shouldn't we rather map all the calculated forces onto the z direction of the needle? The other directions should be handled by the virtual fixture.
	float dummy_object_matrix[12];
	simGetObjectMatrix(dummyHandle, -1, dummy_object_matrix);
	Vector3f dummy_dir = simObjectMatrix2EigenDirection(dummy_object_matrix).normalized();
	externalForce = externalForceMagnitude * dummy_dir; // Should we normalize dir?				Peter: Multiplying with dummy dir creates equal force in all directions of the dummy. Is this right?
}

/**
* @brief Distance between two points
* @param point1: point 1
* @param point2: point 2
* @return distance between the points
*/
float distance3d(Vector3f point1, Vector3f point2) {
	return (point1 - point2).norm();
}

/**
* @brief Set the values of the graphs in v-rep
*/
void setGraphs()
{
	// Force graph
	simSetGraphUserData(forceGraphHandle, "external_force_magnitude", externalForceMagnitude);
	float _objectMatrixRefFrame[12];
	simGetObjectMatrix(lwrTipHandle, -1, _objectMatrixRefFrame);
	Vector3f extf = changeBasis(_objectMatrixRefFrame, lwrTipPhysicsEngineForce);
	simSetGraphUserData(forceGraphHandle, "x", extf(0));
	simSetGraphUserData(forceGraphHandle, "y", extf(1));
	simSetGraphUserData(forceGraphHandle, "z", extf(2));

	// Penetration Length Graph
	for (sPuncture puncture : punctures)
	{
		if (puncture.name == "Fat") {
			simSetGraphUserData(penetrationLengthGraphHandle, "fat_penetration", puncture.penetrationLength);
		}
		else if (puncture.name == "muscle")
		{
			simSetGraphUserData(penetrationLengthGraphHandle, "muscle_penetration", puncture.penetrationLength);
		}
		else if (puncture.name == "lung")
		{
			simSetGraphUserData(penetrationLengthGraphHandle, "lung_penetration", puncture.penetrationLength);
		}
		else if (puncture.name == "bronchus")
		{
			simSetGraphUserData(penetrationLengthGraphHandle, "bronchus_penetration", puncture.penetrationLength);
		}

	}
	simSetGraphUserData(penetrationLengthGraphHandle, "full_penetration", fullPenetrationLength);
}

/**
* @brief Karnopp friction model
* @return current friction value
*/
float karnoppModel()
{
	if (needleVelocityMagnitude <= -zeroThreshold) {
		return fullPenetrationLength*(C_n*sgn(needleVelocityMagnitude) + b_n*needleVelocityMagnitude);
	}
	else if (-zeroThreshold < needleVelocityMagnitude && needleVelocityMagnitude <= 0) {
		return fullPenetrationLength * D_n;
	}
	else if (0 < needleVelocityMagnitude && needleVelocityMagnitude < zeroThreshold) {
		return fullPenetrationLength * D_p;
	}
	else if (needleVelocityMagnitude >= zeroThreshold) {
		return fullPenetrationLength * (C_p*sgn(needleVelocityMagnitude) + b_p*needleVelocityMagnitude);
	}
	return -1;
}


/**
* @brief viscosity constant for the different tissues
* @param puncture: a puncture
* @return the value of the constant
*/
float mu(sPuncture puncture)
{
	map<string, float>::iterator muIt = kelvinMu.find(puncture.name);
	if (muIt != kelvinMu.end())
	{
		return muIt->second;
	}
	else
	{
		return kelvinMu["default"];
	}
}

/**
* @brief threshold for when the needle should penetrate the tissue
* @param name: name of the tissue (the one set in V-REP)
* @return the threshold value for that tissue
*/
float KThresh(std::string name)
{
	map<string, float>::iterator kIt = kelvinKThresh.find(name);
	if (kIt != kelvinKThresh.end())
	{
		return kIt->second;
	}
	else
	{
		return kelvinKThresh["default"];
	}
}

/**
* @brief calculate kelvin-voigt interaction force model
* @return the force magnitude of the interaction
*/
float kelvinVoigtModel() {
	float _forceMagnitude = 0.0;
	for (auto punctureIt = punctures.begin(); punctureIt != punctures.end(); punctureIt++)
	{
		_forceMagnitude += (mu(*punctureIt) * A(punctureIt->penetrationLength));
	}
	_forceMagnitude *= needleVelocityMagnitude;
	return _forceMagnitude;
}

Vector3f simObjectMatrix2EigenDirection(const float* objectMatrix)
{
	return Vector3f(objectMatrix[2], objectMatrix[6], objectMatrix[10]);
}


/**
* @brief reactivate all tissues
*/
void reactivateTissues()
{
	std::cout << punctures.size() << std::endl;
	for (sPuncture puncture : punctures) {
		simSetObjectIntParameter(puncture.handle, RESPONDABLE, 1);
		std::cout << "Reactivated respondable for object " <<puncture.name << std::endl;
	}
	punctures.clear();
}


/**
* @brief get the force as a Vector3f from the return of simGetContactInfo
* @param contactInfo: contact info retrieved from v-rep
* @return the force in Vector3f format
*/
Vector3f simContactInfo2EigenForce(const float* contactInfo)
{
	return Vector3f(contactInfo[3], contactInfo[4], contactInfo[5]);
}

/**
* @brief change the basis going from the global reference frame to a specific one
* @param objectMatrixReferenceFrame: object matrix retrieved from v-rep representing the reference frame you want to go to.
* @param vector: the vector you want to map to another reference frame
* @return a Vector3f representing the mapped vector
*/
Vector3f changeBasis(const float* objectMatrixReferenceFrame, Vector3f vector)
{
	float quat[4];
	simGetQuaternionFromMatrix(objectMatrixReferenceFrame, quat);
	return Quaternionf(quat[0], quat[1], quat[2], quat[3]) * vector;
}

/**
* @brief rotate a force in the general reference frame to the reference frame of the needle tip and retrieve only the z value.
* @param force: the force vector
* @return the force in the z direction of the needle
*/
float generalForce2NeedleTipZ(Vector3f force)
{
	float _lwrTipObjectMatrix[12];
	simGetObjectMatrix(lwrTipHandle, -1, _lwrTipObjectMatrix);
	Vector3f _lwrTipEngineForce = changeBasis(_lwrTipObjectMatrix, lwrTipPhysicsEngineForce);
	return _lwrTipEngineForce.z();
}

/**
* @brief the area of the needle covered by tissue given length of needle covered
* @return the area of the needle covered
*/
float A(float x)
{
	return M_PI * needleRadius * x;
}

/** 
* @brief set all tissues respondable
*/
void setAllTissuesRespondable()
{
	int idx = 0;
	int tissueHandle = simGetObjectChild(phantomHandle, idx);
	while (tissueHandle != -1)
	{
		setRespondable(tissueHandle);
		idx++;
		tissueHandle = simGetObjectChild(phantomHandle, idx);
	}
}

/**
* @brief initialise handle2Name with all tissue names
*/
void initAllTissueNames()
{
	int idx = 0;
	int tissueHandle = simGetObjectChild(phantomHandle, idx);
	while (tissueHandle != -1)
	{
		handle2Name[tissueHandle] = simGetObjectName(tissueHandle);
		idx++;
		tissueHandle = simGetObjectChild(phantomHandle, idx);
	}
}