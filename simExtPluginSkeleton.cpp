#include "simExtPluginSkeleton.h"
#include "stackArray.h"
#include "stackMap.h"
#include "simLib.h"
#include <math.h>
#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Extreme_points_traits_adapter_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Extreme_points_traits_adapter_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Vector_3.h>
#include <vector>
#include <fstream>
#include <algorithm>
typedef CGAL::Exact_predicates_inexact_constructions_kernel      K;
typedef K::Point_3                                               Point_3;
typedef CGAL::Surface_mesh<Point_3>                              Mesh;
typedef K::Vector_3                                              Vector_3;

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif

#if defined(__linux) || defined(__APPLE__)
    #include <unistd.h>
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)    CONCAT(x,y,z)

#define PLUGIN_VERSION 5 // 2 since version 3.2.1, 3 since V3.3.1, 4 since V3.4.0, 5 since V3.4.1

LIBRARY simLib; // the CoppelisSim library that we will dynamically load and bind


float force_max = 0.0;

// CoppeliaSim object parameter IDs
const int RESPONDABLE = 3004;                       // Object parameter id for toggling respondable.
const int RESPONDABLE_MASK = 3019;                  // Object parameter id for toggling respondable mask.
const float FRICTION_COEFFICIENT = 0.03;            // Unit: N/mm ? Delete this?

// Coefficients for bidirectional Karnopp friction model
const float D_p = 18.45;                            // Positive static friction coefficient. Unit: N/m 
const float D_n = -18.23;                           // Negative static friction coefficient. Unit: N/m
const float b_p = 212.13;                           // Positive damping coefficient. Unit: N-s/m²
const float b_n = -293.08;                          // Negative damping coefficient. Unit: N-s/m²
const float C_p = 10.57;                            // Positive dynamic friction coefficient. Unit: N/m
const float C_n = -11.96;                           // Negative dynamic friction coefficient. Unit: N/m
const float zero_threshold = 5.0e-6;                // (delta v/2 in paper) Threshold on static and dynamic fricion. Unit: m/s

// Handles
int dummyToolTipHandle;
int phantomHandle;
int needleHandle;
int needleTipHandle;
int extForceGraphHandle;

Point_3 toolTipPoint;
float needleVelocity;
Vector_3 needleDirection;
float full_penetration_length;
float f_ext_magnitude;
Vector_3 f_ext;


float friction_val = 0;
float x_resistance;
float y_resistance;
float z_resistance;

struct sPuncture {
    int handle;
    Point_3 position;
    Vector_3 direction;
    std::string name;
    float penetration_length;

    void printPuncture(bool puncture) {
        if (puncture) {
            std::cout << "New puncture: " << name << std::endl;
        } else {
            std::cout << "Exit puncture: " << name << std::endl;
        }
        std::cout << "Position: " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
        std::cout << "Direction: " << direction.x() << ", " << direction.y() << ", " << direction.z() << std::endl;
    }
};

std::vector<sPuncture> punctures;
std::vector<int> crashDisables;
std::map<int,int> crashCount;

float sgn(float x);
float karnoppFriction();
void modelFriction(std::string = "kelvin-voigt");
float distance3d(Point_3 point1, Point_3 point2);
float getVelocityMagnitude(simFloat* velocities);
void checkVirtualFixture();
void setRespondable(int handle);
void setUnRespondable(int handle);
void setForceGraph();
void addPuncture(int handle);
void checkPunctures();
void updateNeedleDirection();
void updateNeedleTipPos();
void updateNeedleVelocity();
float kelvinVoigtModel();
Vector_3 simObjectMatrix2Vector_3Direction(const float* objectMatrix);
void checkContacts(float forceMagnitudeThreshold);
float checkSinglePuncture(sPuncture puncture);


// --------------------------------------------------------------------------------------
// simExtSkeleton_getData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_GETDATA_COMMAND "simSkeleton.getData" // the name of the new Lua command

void LUA_GETDATA_CALLBACK(SScriptCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getData")
    int stack=p->stackID;
    CStackArray inArguments;
    inArguments.buildFromStack(stack);
    if ( (inArguments.getSize()>=2)&&inArguments.isString(0)&&inArguments.isMap(1) )
    { // we expect at least 2 arguments: a string and a map

        CStackMap* map=inArguments.getMap(1);
        printf("We received a string (%s) and following message in the map: %s\n",inArguments.getString(0).c_str(),map->getString("message").c_str());
    }
    else
        simSetLastError(LUA_GETDATA_COMMAND,"Not enough arguments or wrong arguments.");

    // Now return a string and a map:
    CStackArray outArguments;
    outArguments.pushString("Hello World");
    CStackMap* map=new CStackMap();
    map->setBool("operational",true);
    CStackArray* pos=new CStackArray();
    double _pos[3]={0.0,1.0,2.0};
    pos->setDoubleArray(_pos,3);
    map->setArray("position",pos);
    outArguments.pushMap(map);
    outArguments.buildOntoStack(stack);
}
// --------------------------------------------------------------------------------------



// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind CoppelisSim functions:
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#else
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the CoppelisSim library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the CoppelisSim library:
    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        return(0); // Means error, CoppelisSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        std::cout << "Error, could not find all required functions in the CoppelisSim library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Check the version of CoppelisSim:
    int simVer,simRev;
    simGetIntegerParameter(sim_intparam_program_version,&simVer);
    simGetIntegerParameter(sim_intparam_program_revision,&simRev);
    if( (simVer<30400) || ((simVer==30400)&&(simRev<9)) )
    {
        std::cout << "Sorry, your CoppelisSim copy is somewhat old, CoppelisSim 3.4.0 rev9 or higher is required. Cannot start 'PluginSkeleton' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppelisSim will unload this plugin
    }

    // Implicitely include the script lua/simExtPluginSkeleton.lua:
    simRegisterScriptVariable("simSkeleton","require('simExtPluginSkeleton')",0);

    // Register the new function:
    simRegisterScriptCallbackFunction(strConCat(LUA_GETDATA_COMMAND,"@","PluginSkeleton"),strConCat("...=",LUA_GETDATA_COMMAND,"(string data1,map data2)"),LUA_GETDATA_CALLBACK);

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppelisSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
    // Here you could handle various clean-up tasks


    unloadSimLibrary(simLib); // release the library
}


// This is the plugin messaging routine (i.e. CoppelisSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from CoppelisSim (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the CoppelisSim user manual.

    if (message==sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag=true; // CoppelisSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message==sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message==sim_message_eventcallback_instancepass)
    {   // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in CoppelisSim. This message is the most convenient way to do so:

        int flags=auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message==sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
        
    }

    if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start
        dummyToolTipHandle = simGetObjectHandle("Dummy_tool_tip");
        phantomHandle = simGetObjectHandle("_Phantom");
        needleHandle = simGetObjectHandle("Needle");
        needleTipHandle = simGetObjectHandle("Needle_tip");
        extForceGraphHandle = simGetObjectHandle("Force_Graph");
        full_penetration_length = 0.0;


    }

    if (message==sim_message_eventcallback_simulationended)
    { // Simulation just ended

        std::cout << punctures.size() << std::endl;
        for (sPuncture puncture : punctures){
            simSetObjectInt32Parameter(puncture.handle, RESPONDABLE, 1);
            std::cout << "Reactivated respondable for object " << simGetObjectName(puncture.handle) << std::endl;
        }
        punctures.clear();
        for (int handle : crashDisables) {
            simSetObjectInt32Parameter(handle, RESPONDABLE, 1);
            std::cout << "Reactivated respondable for crash object " << simGetObjectName(handle) << std::endl;
        }


    }

    if (message==sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message==sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running

            updateNeedleTipPos();
            updateNeedleVelocity();
            updateNeedleDirection();
            checkPunctures();
            checkContacts(2e-5);
            modelFriction("karnopp");
            checkVirtualFixture();
            setForceGraph();


        }
    }

    if (message==sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message==sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message==sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message==sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}


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

float punctureLength(sPuncture puncture)
{
    return distance3d(puncture.position, toolTipPoint) * checkSinglePuncture(puncture);
}

void checkPunctures()
{
    // Iterate through punctures backwards, because if a puncture still is active, all punctures before will also still be active.
    for (auto it = punctures.rbegin(); it != punctures.rend(); ++it)
    {
        float puncture_length = punctureLength(*it);
        // If puncture length is above zero, all punctures before it in the vector will be unchanged.
        if (checkSinglePuncture(*it) > 0)
        {
            full_penetration_length -= it->penetration_length;
            full_penetration_length += puncture_length;
            // This penetration length might have been updated, so update.
            it->penetration_length = puncture_length;
            // Set punctures to be from the first puncture up until the current.
            punctures = std::vector<sPuncture>(it, punctures.rend());
            std::reverse(punctures.begin(), punctures.end());
            return;
        } else {
            // The needle isn't puncturing this tissue anymore. Set tissue respondable and print.
            setRespondable(it->handle);
            full_penetration_length  -= it->penetration_length;
            it->printPuncture(false);
        }

    }
    // No punctures had length > 0
    punctures.clear();
}




void setRespondable(int handle)
{
    simSetObjectInt32Parameter(handle, RESPONDABLE, 1);
}

void setUnRespondable(int handle)
{
    simSetObjectInt32Parameter(handle, RESPONDABLE, 0);
}

float checkSinglePuncture(sPuncture puncture) {
    Vector_3 current_translation = Vector_3(puncture.position, toolTipPoint);
    // If the dot product of the two vectors are positive, we are still in the tissue.
    if (CGAL::scalar_product(current_translation, puncture.direction) >= -1e-6)
        return 1.0;
    return -1;
}

void updateNeedleVelocity()
{
    simFloat needleVelocities[3];
    if (simGetObjectVelocity(needleTipHandle, needleVelocities, NULL) == -1)
        std::cerr << "Needle tip velocity retrieval failed" << std::endl;
    needleVelocity = getVelocityMagnitude(needleVelocities);
}

void updateNeedleDirection()
{
    simFloat objectMatrix[12];
    simGetObjectMatrix(needleTipHandle, -1, objectMatrix);
    needleDirection = simObjectMatrix2Vector_3Direction(objectMatrix);

}

void updateNeedleTipPos()
{
    float needleTipPos[3];
    simGetObjectPosition(needleTipHandle, -1, needleTipPos);
    toolTipPoint = Point_3(needleTipPos[0], needleTipPos[1], needleTipPos[2]);
}

void addPuncture(int handle)
{
    float needleMatrix[12];
    simGetObjectMatrix(needleHandle, -1, needleMatrix);
    sPuncture puncture;
    puncture.position = toolTipPoint;
    puncture.direction = Vector_3(needleMatrix[2], needleMatrix[6], needleMatrix[10]);
    puncture.handle = handle;
    puncture.name = simGetObjectName(handle);
    puncture.penetration_length = punctureLength(puncture);
    full_penetration_length += puncture.penetration_length;
    setUnRespondable(handle);
    punctures.push_back(puncture);
    puncture.printPuncture(true);
}


void checkContacts(float forceMagnitudeThreshold)
{
    for (int a=0; a<20; a++)
    {
        simInt contactHandles[2];
        simFloat contactInfo[12];
        simGetContactInfo(sim_handle_all, needleTipHandle, a + sim_handleflag_extended, contactHandles, contactInfo);
        if (contactHandles[1] < 1000)
        {
            float forceMagnitude = 0;
            for (int b=3; b<6; b++)
            {
                forceMagnitude += pow(contactInfo[b], 2);
            }
            forceMagnitude = sqrt(forceMagnitude);
            simSetGraphUserData(extForceGraphHandle, "Detection", forceMagnitude);
            if (forceMagnitude > force_max)
            {
                force_max = forceMagnitude;
                std::cout << force_max << std::endl;
            }
            int respondableValue;
            simGetObjectInt32Parameter(contactHandles[1], RESPONDABLE, &respondableValue);
            if(forceMagnitude > forceMagnitudeThreshold && respondableValue != 0 && simGetObjectParent(contactHandles[1]) == phantomHandle){
                addPuncture(contactHandles[1]);
                std::cout << "Force magnitude: " << forceMagnitude << std::endl;
            }

        }
    }
}


float sgn(float x) {
    if (x > 0) return 1.0;
    if (x < 0) return -1.0;
    return 0.0;
}

void modelFriction(std::string model) {

    if (model == "kelvin-voigt") {
        f_ext_magnitude = kelvinVoigtModel();
    }
    else if (model == "karnopp")
    {
        f_ext_magnitude = karnoppFriction();
    }
    else
    {
        f_ext_magnitude = kelvinVoigtModel();
    }
    f_ext = f_ext_magnitude * needleDirection;
}

float distance3d(Point_3 point1, Point_3 point2){
    Vector_3 vector = Vector_3(point1, point2);
    return vector.squared_length();
}

float getVelocityMagnitude(simFloat* velocities) {
    return std::sqrt(std::pow(velocities[0], 2)
                     + std::pow(velocities[1], 2)
                     + std::pow(velocities[2], 2));
}

void checkVirtualFixture(){
    if (punctures.size() > 0){
        x_resistance = 10000; // a big number
        y_resistance = 10000;
        z_resistance = friction_val;
    } else
    {
        full_penetration_length = 0.0;
    }
}

void setForceGraph()
{
    //simSetGraphUserData(ext_force_graph_handler, "Detection", (float)detection);
    //simSetGraphUserData(ext_force_graph_handler, "Ext_F_y", (float)g);
    //simSetGraphUserData(ext_force_graph_handler, "predicted_F", (float)predicted_F);
    simSetGraphUserData(extForceGraphHandle, "measured_F", f_ext_magnitude);
    for (sPuncture puncture : punctures)
    {
        if (puncture.name == "Fat"){
            simSetGraphUserData(extForceGraphHandle, "fat_penetration", puncture.penetration_length);
        }
        else if (puncture.name == "muscle")
        {
            simSetGraphUserData(extForceGraphHandle, "muscle_penetration", puncture.penetration_length);
        }
        else if (puncture.name == "lung")
        {
            simSetGraphUserData(extForceGraphHandle, "lung_penetration", puncture.penetration_length);
        }
        else if (puncture.name == "bronchus")
        {
            simSetGraphUserData(extForceGraphHandle, "bronchus_penetration", puncture.penetration_length);
        }

    }
    simSetGraphUserData(extForceGraphHandle, "full_penetration", full_penetration_length);
}


void calculateWithRotatingSomething()
{
    /*

    // Iterate through all punctures.
    for (auto it = punctures.begin(); it != punctures.end(); it++)
    {
        if (!simCheckCollision(it->handle, needleHandle) && !simCheckCollision(it->handle, needleTipHandle) && distance3d(toolTipPoint, it->getPosPoint3()) < 0.02)
        {

            simSetObjectInt32Parameter(it->handle, RESPONDABLE, 1);
            it->printPuncture(false);
            punctures.erase(it--);
        } else
        {
            Point_3 puncturePoint = it->getPosPoint3();
            float penetrationDistance = distance3d(toolTipPoint, puncturePoint);
            if (penetrationDistance > full_penetration_length) full_penetration_length = penetrationDistance;
            float *pIntersections;
            int numberOfRetrievals = simCheckCollisionEx(it->handle, needleHandle, &pIntersections);
            if (numberOfRetrievals > 0 ) {

                Mesh sm;
                std::map<std::size_t, Point_3> vertice_map;
                for (int i = 0; i < numberOfRetrievals; i+=6) {
                    Point_3 point1 = Point_3(pIntersections[i], pIntersections[i+1], pIntersections[i+2]);
                    Point_3 point2 = Point_3(pIntersections[i+3], pIntersections[i+4], pIntersections[i+5]);
                    vertice_map[sm.add_vertex(point1).idx()] = point1;
                    vertice_map[sm.add_vertex(point2).idx()] = point2;
                }

                //This will contain the extreme vertices
                std::vector<Mesh::Vertex_index> extreme_vertices;
                //call the function with the traits adapter for vertices
                CGAL::extreme_points_3(vertices(sm), std::back_inserter(extreme_vertices),
                                       CGAL::make_extreme_points_traits_adapter(sm.points()));
                float max_distance = 0.0;
                for (auto it_ev = extreme_vertices.begin(); it_ev != extreme_vertices.end(); it_ev++) {
                    Point_3 point1 = vertice_map[it_ev->idx()];
                    for (auto it_ev2 = extreme_vertices.begin(); it_ev2 != extreme_vertices.end(); it_ev2++) {
                        Point_3 point2 = vertice_map[it_ev2->idx()];
                        float distance = distance3d(point1, point2);
                        if (distance > max_distance) max_distance = distance;
                    }
                }
                it->penetration_length = max_distance;

                total_contact_length += max_distance;
                simReleaseBuffer((simChar*) pIntersections);
            }
        }
    }
    */

}

float karnoppFriction()
{
    if(needleVelocity <= -zero_threshold) {
        return full_penetration_length*(C_n*sgn(needleVelocity) + b_n*needleVelocity);
    } else if (-zero_threshold < needleVelocity && needleVelocity <= 0){
        return full_penetration_length * D_n;
    } else if (0 < needleVelocity && needleVelocity < zero_threshold) {
        return full_penetration_length * D_p;
    } else if (needleVelocity >= zero_threshold) {
        return full_penetration_length * (C_p*sgn(needleVelocity) + b_p*needleVelocity);
    }
    return -1;
}

float B(sPuncture puncture)
{
    if (puncture.name == "Fat")
    {
        return 3.0f * 100.0f;
    }
    else if (puncture.name == "muscle")
    {
        return 3.0f * 100.0f;
    }
    else if (puncture.name == "lung")
    {
        return 3.0f * 100.0f;
    }
    else
    {
        return 3.0f * 100.0f;
    }
}

float kelvinVoigtModel(){
    float f_magnitude = 0.0;
    for (auto puncture_it = punctures.begin(); puncture_it != punctures.end(); puncture_it++)
    {
        f_magnitude += (B(*puncture_it) * puncture_it->penetration_length);
    }
    f_magnitude *= needleVelocity;
    return f_magnitude;
}

Vector_3 simObjectMatrix2Vector_3Direction(const float* objectMatrix)
{
    return Vector_3(objectMatrix[2], objectMatrix[6], objectMatrix[10]);
}


