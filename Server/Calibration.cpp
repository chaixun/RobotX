#include "Calibration.h"

namespace Calibration
{

enum CalibrationState
{
    None = 0,
    BodyUP = 4,
    Go=1,
    Processing=2,
    Back=3,
};

void TransM(double matrixIn[6], double matrixOut[6])
{
    double	alpha = matrixIn[0];
    double	beta = matrixIn[1];
    double	gama = matrixIn[2];

    Eigen::Matrix3f R_X;
    R_X << 1, 0, 0, 0, cos(gama), -sin(gama), 0, sin(gama), cos(gama);

    Eigen::Matrix3f R_Z;
    R_Z << cos(beta), -sin(beta), 0, sin(beta), cos(beta), 0, 0, 0, 1;

    Eigen::Matrix3f R_Y;
    R_Y << cos(alpha), 0, sin(alpha), 0, 1, 0, -sin(alpha), 0, cos(alpha);

    Eigen::Matrix3f Pose;
    Pose = R_Y * R_Z * R_X;

    double pMatrix[4][4] = { 0 };

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            pMatrix[i][j] = Pose(i, j);
        }
    }
    pMatrix[0][3] = matrixIn[3];
    pMatrix[1][3] = matrixIn[4];
    pMatrix[2][3] = matrixIn[5];
    pMatrix[3][3] = 1;

    aris::dynamic::s_pm2pe(*pMatrix, matrixOut, "313");
}

CalibrationWrapper calibrationWrapper;

aris::control::Pipe<int> CalibrationWrapper::calibrationPipe(true);
std::thread CalibrationWrapper::calibrationThread;

atomic_bool CalibrationWrapper::isTerrainCaliRecorded(false);
atomic_bool CalibrationWrapper::isSending(false);
atomic_bool CalibrationWrapper::isStop(false);
atomic_int CalibrationWrapper::calibrationState(CalibrationState::None);

void CalibrationWrapper::CalibrationStart()
{
    calibrationThread = std::thread([]()
    {
        while(true)
        {
            int postureCount;
            calibrationPipe.recvInNrt(postureCount);

            kinect2.SavePcd();
            cout<<" map recorded"<<endl;
            isTerrainCaliRecorded=true;
        }
    });
}

auto CalibrationWrapper::visionCalibrateParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    VISION_CALIBRATION_PARAM param;

    double a;

    std::ifstream file;
    std::string FileName = "/home/hex/Desktop/codes/chaixun/VisionIX/Pose.txt";
    cout<<"file name:"<<FileName<<endl;

    file.open(FileName);
    if (!file) throw std::logic_error("calibration params file not exist");
    file>>a;

    int postureNum{ 0 };

    for (double tem; !file.eof(); file >> tem)  ++postureNum;
    if (postureNum % 6 != 0) throw std::logic_error("calibration params file invalid, because the num of numbers is not valid");
    postureNum /= 6;
    file.close();

    param.postureNum = postureNum;

    file.open(FileName);
    //file>>a;
    for (int i = 0; !file.eof(); file >> param.postureLib[i++]);
    file.close();

    cout<<"postureNum:"<<postureNum<<endl;

    msg_out.copyStruct(param);
}

auto CalibrationWrapper::captureParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void
{
    kinect2.SavePcd();
    cout<<"capture map"<<endl;
}

auto CalibrationWrapper::visionCalibrate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast< const VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static int localCount=0;
    static int postureCount=0;
    static double targetPosture[6];
    double currentPeb[6];
    double s;

    static int bodyUpCount = 0;
    static double beginPee[18]{0};
    static double beginPeb[6]{0};
    static double bodyHeight = 0.95;

   // static double partio = 1;
   // static double bodypartio = 0.8;

     static double partio = 0.6;
        static double bodypartio = 1;

//   static double partio = 0.8;
//   static double bodypartio = 1;

    if(pSP.count==0)
    {
        rt_printf("calibration gait begins\n");
    }

    localCount = localCount%pSP.gaitLength;

    switch(calibrationState)
    {
    case None:
        calibrationState = BodyUP;
        break;
    case BodyUP:
        if(bodyUpCount == 0)
        {
            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();
            robot.GetPee(beginPee, beginMak);
            robot.GetPeb(beginPeb, beginMak);
            rt_printf("body Up \n");
            rt_printf("bodyHeight: %lf\n", beginPee[1]);
        }

        double Peb[6], Pee[18];
        std::copy(beginPeb, beginPeb + 6, Peb);
        std::copy(beginPee, beginPee + 18, Pee);

        double s1;
        s1 = cos(PI * (bodyUpCount + 1) / 2000);

        Peb[1] = beginPeb[1] + (bodyHeight + beginPee[1]) * (1 - s1)/2;

        robot.SetPeb(Peb, beginMak);
        robot.SetPee(Pee, beginMak);

        bodyUpCount++;

        if(bodyUpCount == 2000)
        {
            calibrationState = Go;
            bodyUpCount = 0;
            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();
            robot.GetPee(beginPee, beginMak);
            robot.GetPeb(beginPeb, beginMak);
            rt_printf("body Up End \n");
            rt_printf("bodyHeight: %lf\n", beginPee[1]);
        }
        break;
    case Go:
        if(localCount == 0)
        {
            rt_printf("calibration posture %d\n",postureCount);
            memcpy(targetPosture,&pSP.postureLib[6*postureCount],sizeof(targetPosture));
            rt_printf("%lf %lf %lf %lf %lf %lf \n", targetPosture[0], targetPosture[1], targetPosture[2], targetPosture[3], targetPosture[4], targetPosture[5]);
        }

        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]

        currentPeb[2]= partio * targetPosture[2]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[1]= partio * targetPosture[1]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[0]= partio * targetPosture[0]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[3]= partio * targetPosture[3]*(1-cos(s))/2;
        currentPeb[4]= partio * targetPosture[4]*(1-cos(s))/2 * bodypartio;
        currentPeb[5]= partio * targetPosture[5]*(1-cos(s))/2;

        double bodyPose[6];
        TransM(currentPeb, bodyPose);

        robot.SetPeb(bodyPose,beginMak);
        robot.SetPee(beginPee, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            calibrationState=Processing;
            calibrationPipe.sendToNrt(postureCount);
            localCount = 0;
            robot.GetPeb(beginPeb, beginMak);
            rt_printf("bodyHeight: %lf\n", beginPeb[1]);
            rt_printf("capture begin\n");
        }
        break;
    case Processing:
        if(isTerrainCaliRecorded==true)
        {
            calibrationState=Back;
            isTerrainCaliRecorded=false;
            rt_printf("end capture !\n");
        }
        break;
    case Back:
        if(localCount==0)
        {
            rt_printf("calibration posture finished going back %d\n",postureCount);
        }
        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]

        currentPeb[2]= partio * targetPosture[2]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[1]= partio * targetPosture[1]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[0]= partio * targetPosture[0]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[3]= partio * targetPosture[3]*(1+cos(s))/2;
        currentPeb[4]= partio * targetPosture[4]*(1+cos(s))/2 * bodypartio;
        currentPeb[5]= partio * targetPosture[5]*(1+cos(s))/2;

        double bodyPose1[6];
        TransM(currentPeb, bodyPose1);

        robot.SetPeb(bodyPose1,beginMak);
        robot.SetPee(beginPee, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            postureCount+=1;
            calibrationState = Go;
            localCount = 0;
            if(postureCount==pSP.postureNum)
            {
                localCount = 0;
                calibrationState = None;
                return 0;
            }
        }
        break;
    default:
        break;
    }

    return 1;
}

}
