#ifndef UPONESTEP_H
#define UPONESTEP_H

#include <aris.h>
#include <Robot_Gait.h>
#include <rtdk.h>

using namespace std;

struct UpOneStepParam final :public aris::server::GaitParamBase
{
    double stepHeight;
    double bodyHeight;
    double normalHeight;
    std::int32_t footMoveTime;
    std::int32_t totalCount;
};
void ParseUp20Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int Up20StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

void ParseDown20Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int Down20StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

#endif // UPONESTEP_H
