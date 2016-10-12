#ifndef TREEPASS_H
#define TREEPASS_H

#include <thread>
#include <iomanip>
#include <bitset>
#include <map>
#include <atomic>
#include <rtdk.h>

#include <aris.h>
#include <Robot_Gait.h>
#include "Velodyne.h"
#include "Vision_Gait.h"
#include "VisionSensor.h"

using namespace std;

namespace TreePass
{

class TreePassWrapper
{
public:
    static aris::control::Pipe<int> treePassPipe;

    static atomic_bool isSending;
    static atomic_bool isStop;

    static std::thread treePassThread;

    static void TreePassStart();

    static auto TreePassParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    static auto TreePaseWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int;
    static auto StopTreePassParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;


   static VISION_TREEPASS_PARAM visionTreePassParam;
private:
    //parameters
    static float t_r;    // radius of obstacle
    static float t_d;    // distance between two obstacles
    static float t_x;    // x set
    static float t_y;    // y set
    static float t_z;    // z set
    static float t_xDis;  // x distacne between lidar and robot's origin
    static float t_yDis;  // y distance between lidar and robot's origin
    static float t_n1;    // first step number
    static float t_n2;    // second step number
    static int   t_t1;    // first step count
    static int   t_t2;    // second step count
    static bool  t_isPause;    // paus
    static int   t_treeNumber; // trees' count

    static float t_deltax;
    static float t_deltay;
    static float t_deltaz;

    static void generatePoint(char orentation);
    static void generateParam(double x0, double y0,double x1,double y1,int n,int t);
    static float generateAngle(double x0, double y0,double x1,double y1,int n,int t);

    static int phase;
    static double m_targetPointX[2];
    static double m_targetPointY[2];
    static bool isGo;
    static bool isAnalyzed;
    static char treeNumber;
    static int  isFind;

};

extern TreePassWrapper treePassWrapper;
}

#endif // TREEPASS_H
