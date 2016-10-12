#include "TreePass.h"

namespace TreePass
{

TreePassWrapper treePassWrapper;

float TreePassWrapper::t_r=0.02f;    // radius of obstacle
float TreePassWrapper::t_d= 2.5f;    // distance between two obstacles //2.5
float TreePassWrapper::t_x= 0.8f;    // x set
float TreePassWrapper::t_y= 0.1f;    // y set   //0.2
float TreePassWrapper::t_z= 0.0f;    // z set
float TreePassWrapper::t_xDis=0.0f;  // x distacne between lidar and robot's origin
float TreePassWrapper::t_yDis=1.025f;  // y distance between lidar and robot's origin
float TreePassWrapper::t_n1=3.0f;    // first step number
float TreePassWrapper::t_n2=3.0f;    // second step number
int  TreePassWrapper::t_t1=1500;    // first step count
int  TreePassWrapper::t_t2=1500;    // second step count
bool TreePassWrapper::t_isPause=0;  // paus
int  TreePassWrapper::t_treeNumber=11; // trees' count

float TreePassWrapper::t_deltax=0.7f;
float TreePassWrapper::t_deltay=1.0f;
float TreePassWrapper::t_deltaz=0.6f;

int TreePassWrapper::phase=0;
double TreePassWrapper::m_targetPointX[2]={0};
double TreePassWrapper::m_targetPointY[2]={0};
bool TreePassWrapper::isGo=0;
bool TreePassWrapper::isAnalyzed=0;
char TreePassWrapper::treeNumber=0;
int  TreePassWrapper::isFind=1;

aris::control::Pipe<int> TreePassWrapper::treePassPipe(true);

atomic_bool TreePassWrapper::isSending(false);
atomic_bool TreePassWrapper::isStop(false);

std::thread TreePassWrapper::treePassThread;

VISION_TREEPASS_PARAM TreePassWrapper::visionTreePassParam;

void TreePassWrapper::generatePoint(char orentation)
{
    double x0=0;
    double x1=0;
    double y0=0;
    double y1=0;
    double midX=0;
    double midY=0;

    double x=0;
    double y=0;
    double dis=0;

    double xTarget[2]={0};
    double yTarget[2]={0};
    double tempX=0;
    double tempY=0;

    x0=obstacle.at(0).x+t_xDis;
    x1=obstacle.at(1).x+t_xDis;
    y0=obstacle.at(0).y+t_yDis;
    y1=obstacle.at(1).y+t_yDis;

    std::cout<<"x0: "<<x0<<"y0: "<<y0<<"x1: "<<x1<<"y1: "<<y1<<std::endl;

    midX=(x0+x1)/2;
    midY=(y0+y1)/2;

    x=x1-x0;
    y=y1-y0;//(x/dis,y/dis)
    dis=sqrt(x*x+y*y);

    if(0==orentation)//Left
    {
        m_targetPointX[0]=midX-t_x*( y)/dis-t_y*(x)/dis;
        m_targetPointY[0]=midY-t_x*(-x)/dis-t_y*(y)/dis;
        m_targetPointX[1]=midX+t_x*( y)/dis+t_y*(x)/dis;
        m_targetPointY[1]=midY+t_x*(-x)/dis+t_y*(y)/dis;
    }
    else
    {
        m_targetPointX[0]=midX+t_x*( y)/dis-t_y*(x)/dis;
        m_targetPointY[0]=midY+t_x*(-x)/dis-t_y*(y)/dis;
        m_targetPointX[1]=midX-t_x*( y)/dis+t_y*(x)/dis;
        m_targetPointY[1]=midY-t_x*(-x)/dis+t_y*(y)/dis;
    }
}
void TreePassWrapper::generateParam(double x0, double y0, double x1, double y1, int n, int t)
{

    double x=0;
    double y=0;
    double dis=0;
    double alpha=0;
    double stepDis=0;

    x=x1-x0;
    y=y1-y0;
    dis=sqrt(x*x+y*y);
    alpha=acos(y/dis);
    if(x>0)
    {
        alpha = -alpha;
    }
    stepDis = dis/(n-0.5);

    std::cout<<"x0: "<<x0<<"  y0: "<<y0<<"  x1: "<<x1<<" y1: "<<y1<<" alpha: "<<alpha/PI*180 <<"  dis: "<<stepDis<<std::endl;

    visionTreePassParam.alpha=alpha;
    visionTreePassParam.stepDis=stepDis;
    visionTreePassParam.beta=0;
    visionTreePassParam.totalCount=t;
    visionTreePassParam.stepNumber=n;
}

float TreePassWrapper::generateAngle(double x0, double y0, double x1, double y1, int n, int t)
{
    double x=0;
    double y=0;
    double dis=0;
    double alpha=0;
    double beta=0;
    double stepDis=0;

    x=x1-x0;
    y=y1-y0;
    dis=sqrt(x*x+y*y);
    alpha=acos(y/dis);
    if(x>0)
    {
        alpha = -alpha;
    }
    beta = alpha/(n-0.5);
    std::cout<<"x0: "<<x0<<"  y0: "<<y0<<"  x1: "<<x1<<" y1: "<<y1<<" alpha: "<<alpha/PI*180 <<"  beta: "<<beta/PI*180<<std::endl;

    visionTreePassParam.alpha=0;
    visionTreePassParam.stepDis=0;
    visionTreePassParam.beta=beta;
    visionTreePassParam.totalCount=t;
    visionTreePassParam.stepNumber=n;

    return abs(alpha);
}

void TreePassWrapper::TreePassStart()
{
    treePassThread = std::thread([](){
    pcl::StopWatch time;
        while(true)
        {
            int a;
            float beta;
            treePassPipe.recvInNrt(a);
            std::cout<<"Passing Tree:"<<(int)treeNumber<<std::endl;
            std::cout<<"case: "<<(int)phase<<std::endl;
            switch(phase)
            {
            case 0:
                time.reset();
                if(visionTreePassParam.orentation)
                {
                    isFind=velodyne1.Update((-t_x-t_deltax),(-t_x+t_deltax),(t_d/2-t_y-t_yDis-t_deltay),(t_d/2-t_y-t_yDis+t_deltay),(t_z-t_deltaz),(t_z+t_deltaz),t_r,t_d);
                }
                else
                {
                    isFind=velodyne1.Update((t_x-t_deltax),(t_x+t_deltax),(t_d/2-t_y-t_yDis-t_deltay),(t_d/2-t_y-t_yDis+t_deltay),(t_z-t_deltaz),(t_z+t_deltaz),t_r,t_d);
                }
                //isFind =1 ;
                if(0==isFind)
                {
                    isStop=1;
                    std::cout<<"Not Found!"<<std::endl;
                    break;
                }
                generatePoint(visionTreePassParam.orentation);
                beta=generateAngle(obstacle.at(0).x+t_xDis, obstacle.at(0).y+t_yDis,obstacle.at(1).x+t_xDis,obstacle.at(1).y+t_yDis,1,2000);
                std::cout<<"time took:  "<<time.getTime()<<"   ms"<<" (in case 0) "<<std::endl;
                if(beta>0.08)//((beta>0.04)&&(beta<0.08))//2.5'
                {
                    if(beta>0.35)
                    {
                        isStop=1;
                        std::cout<<"Not Found! "<<beta/3.14*180<<" Beta too large!!"<<std::endl;
                        break;
                    }
                    std::cout<<"in case0!"<<std::endl;
                    isGo=1;
                    break;
                }
                else
                {
                    phase=1;
                    std::cout<<"pass case0!"<<std::endl;
                }
            case 1:
                std::cout<<"case1!"<<std::endl;
                time.reset();
                if(visionTreePassParam.orentation)
                {
                    isFind=velodyne1.Update((-t_x-t_deltax),(-t_x+t_deltax),(t_d/2-t_y-t_yDis-t_deltay),(t_d/2-t_y-t_yDis+t_deltay),(t_z-t_deltaz),(t_z+t_deltaz),t_r,t_d);
                }
                else
                {
                    isFind=velodyne1.Update((t_x-t_deltax),(t_x+t_deltax),(t_d/2-t_y-t_yDis-t_deltay),(t_d/2-t_y-t_yDis+t_deltay),(t_z-t_deltaz),(t_z+t_deltaz),t_r,t_d);
                }
                //isFind=1;
                if(0==isFind)
                {
                    isStop=1;
                    std::cout<<"Not Found!"<<std::endl;
                    break;
                }
                generatePoint(visionTreePassParam.orentation);
                generateParam(0, 0,m_targetPointX[0],m_targetPointY[0],t_n1,t_t1);
                std::cout<<"time took:  "<<time.getTime()<<"   ms"<<" (in case 1) "<<std::endl;
                isGo=1;
                if(0==visionTreePassParam.orentation)
                {
                    visionTreePassParam.orentation=1;
                }
                else
                {
                    visionTreePassParam.orentation=0;
                }
                break;
            case 2:
                std::cout<<"case2!"<<std::endl;
                generateParam(m_targetPointX[0],m_targetPointY[0],m_targetPointX[1],m_targetPointY[1],t_n2,t_t2);
                isGo=1;
                treeNumber++;
                break;
            case 3:
                std::cout<<"case3!"<<std::endl;
                generateParam(0,0,0,1.0,3,1200);
                isGo=1;
                treeNumber++;
                break;
            default:
                std::cout<<"phase error!"<<std::endl;
                break;
            }
        }
    });
}

auto TreePassWrapper::TreePassParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    for (auto &i : params)
    {
        if (i.first == "orentation")
        {
            visionTreePassParam.orentation = std::stoi(i.second);
        }
        if (i.first == "number")
        {
            t_treeNumber = std::stoi(i.second);
        }
    }
    phase=0;
    treeNumber=0;
    isGo=0;
    isSending = false;
    visionTreePassParam.count = 0;
    msg_out.copyStruct(param);
}

auto TreePassWrapper::StopTreePassParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

auto TreePassWrapper::TreePaseWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    static bool isFirstTime = true;
    int remainCount=0;

    if(isStop)
    {
        isStop = false;
        phase=0;
        //m_targetPointX[2]={0};
        //m_targetPointY[2]={0};
        m_targetPointX[0]=0;
        m_targetPointX[1]=0;
        m_targetPointY[0]=0;
        m_targetPointY[1]=0;
        isGo = 0;
        treeNumber=0;
        visionTreePassParam.count = 0;
        isFind=1;
        return 0;
    }

    if(isGo)
    {
        if(isFirstTime)
        {
            visionTreePassParam.count = 0;
            isFirstTime = false;
            std::cout<<"First Time"<<std::endl;
        }

        remainCount = RobotVisionWalkForTreePass(robot, visionTreePassParam);
        visionTreePassParam.count++;
        if(0 == remainCount)
        {
            std::cout<<"Finished this step!"<<std::endl;
            phase=(phase+1)%3;
            if((t_isPause)&&(0==phase))
            {
                treeNumber=t_treeNumber;
            }

            if(treeNumber<(t_treeNumber-1))
            {

            }
            else if((t_treeNumber-1) == treeNumber)
            {
                phase=3;
            }
            else
            {
                isFirstTime = true;
                phase=0;
                treeNumber=0;
                isGo=0;
                isSending = false;
                std::cout<<"All done!!!"<<std::endl;
                return 0;
            }
            isFirstTime = true;
            isGo = 0;
            isSending = false;
            std::cout<<"continue!!!"<<std::endl;
            return -1;
        }
        return -1;
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            treePassPipe.sendToNrt(6);
            std::cout<<"Send!!!"<<std::endl;
            isSending = true;
            return -1;
        }
    }
}
}
