#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

// CX, ZYN, ZY's gait
#include "Vision_Gait.h"
#include "VisionSensor.h"
#include "PassStepDitch.h"
#include "Calibration.h"
#include "TreePass.h"
#include "UpOneStep.h"

int main(int argc, char *argv[])
{
    std::string xml_address;

//    velodyne1.Start();
//    kinect2.Start();
//    Calibration::calibrationWrapper.CalibrationStart();
    TreePass::treePassWrapper.TreePassStart();
    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
    }
    else if (std::string(argv[1]) == "X")
    {
        xml_address = "/home/hex/Desktop/X/Robot_X/Robot_X.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();


    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    // CX and ZYN's gait
    rs.addCmd("sdwk", PassStepDitch::adjustWrapper.PassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);
    rs.addCmd("ssdwk", PassStepDitch::adjustWrapper.StopPassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);
    rs.addCmd("ca", Calibration::calibrationWrapper.visionCalibrateParse, Calibration::calibrationWrapper.visionCalibrate);
    rs.addCmd("cap", Calibration::calibrationWrapper.captureParse, nullptr);

    rs.addCmd("up20", ParseUp20Step, Up20StepGait);
    rs.addCmd("dw20", ParseDown20Step, Down20StepGait);

    // ZY's gait
    rs.addCmd("twk", TreePass::treePassWrapper.TreePassParse, TreePass::treePassWrapper.TreePaseWalk);
    rs.addCmd("swk", TreePass::treePassWrapper.StopTreePassParse, TreePass::treePassWrapper.TreePaseWalk);


    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();



    return 0;
}
