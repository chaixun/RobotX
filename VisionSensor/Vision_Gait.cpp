#include "Vision_Gait.h"

#include <string.h>
#include <math.h>
#include <iostream>

using namespace std;

void parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
    MoveRotateParam param;

    for(auto &i:params)
    {
        if(i.first=="u")
        {
            param.targetBodyPE213[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            param.targetBodyPE213[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            param.targetBodyPE213[2]=stod(i.second);
        }
        else if(i.first=="yaw")
        {
            param.targetBodyPE213[3]=stod(i.second)*PI/180;
        }
        else if(i.first=="pitch")
        {
            param.targetBodyPE213[4]=stod(i.second)*PI/180;
        }
        else if(i.first=="roll")
        {
            param.targetBodyPE213[5]=stod(i.second)*PI/180;
        }
        else if(i.first=="totalCount")
        {
            param.totalCount=stoi(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    msg.copyStruct(param);

    std::cout<<"finished parse"<<std::endl;
}

int moveWithRotate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const MoveRotateParam &>(param_in);
    
    static double beginBodyPE213[6];
    static double pEE[18];
    if(param.count==0)
    {
        robot.GetPeb(beginBodyPE213,"213");
        robot.GetPee(pEE);
    }

    double realBodyPE213[6];
    for(int i=0;i<6;i++)
    {
        double s = -(param.targetBodyPE213[i] / 2)*cos(PI * (param.count + 1) / param.totalCount ) + param.targetBodyPE213[i] / 2;
        realBodyPE213[i]=beginBodyPE213[i]+s; //target of current ms
    }

    double pBody[6];

    robot.SetPeb(realBodyPE213,"213");
    robot.SetPee(pEE);

    return param.totalCount - param.count - 1;
}

int RobotVisionWalkForTreePass(Robots::RobotBase &robot, const VISION_TREEPASS_PARAM &param)
{
    Robots::WalkParam wk_param;

    wk_param.alpha = param.alpha;
    wk_param.d = -param.stepDis;
    wk_param.beta = param.beta;
    wk_param.h = 0.075;
    wk_param.n = param.stepNumber;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::walkGait(robot, wk_param);
}

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    Robots::WalkParam wk_param;

    switch(param.movetype)
    {
    case turn:
    {
        wk_param.alpha = 0;
        wk_param.beta = param.turndata * M_PI / 180;
        wk_param.d = 0;
        wk_param.h = 0.05;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                wk_param.alpha = -M_PI/2;
                wk_param.d = param.movedata[0];
            }
            else
            {
                wk_param.alpha = M_PI/2;
                wk_param.d = -param.movedata[0];
            }
            wk_param.beta = 0;
            wk_param.h = 0.05;
        }
        else
        {
            wk_param.alpha = M_PI;
            wk_param.beta = 0;
            wk_param.d = param.movedata[2];
            wk_param.h = 0.05;
        }
    }
        break;
    default:
        break;
    }

    wk_param.n = 1;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::walkGait(robot, wk_param);
}

int RobotBody(Robots::RobotBase &robot, int count, float bodymovedata[3])
{
    int totalCount = 2000;

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double beginPeb[6];

    if (count % totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.GetPeb(beginPeb, beginMak);
    }

    double Peb[6], Pee[18];
    std::copy(beginPeb, beginPeb + 6, Peb);
    std::copy(beginPee, beginPee + 18, Pee);

    double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

    Peb[0] += bodymovedata[0] * (1 - cos(s))/2;
    Peb[1] += bodymovedata[1] * (1 - cos(s))/2;
    Peb[2] += bodymovedata[2] * (1 - cos(s))/2;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return totalCount - count - 1;
}
