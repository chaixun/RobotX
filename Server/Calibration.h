#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>

#include <thread>
#include <aris.h>
#include <rtdk.h>
#include <Robot_Gait.h>
#include "Vision_Gait.h"
#include "VisionSensor.h"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace aris::core;

namespace Calibration
{
class CalibrationWrapper
{
public:
    static aris::control::Pipe<int> calibrationPipe;
    static std::thread calibrationThread;

    static void CalibrationStart();

    static auto visionCalibrateParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
    static auto visionCalibrate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int;
    static auto captureParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;

private:
    static atomic_bool isTerrainCaliRecorded;
    static atomic_bool isSending;
    static atomic_bool isStop;
    static atomic_int calibrationState;
};
extern CalibrationWrapper calibrationWrapper;
}

#endif // CALIBRATION_H
