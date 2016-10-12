#ifndef VELODYNE_H
#define VELODYNE_H

#include <stdlib.h>
#include <memory>
#include <iostream>
#include "Velodyne.h"
#include "pcl/io/vlp_grabber.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include "pcl/common/time.h"

struct OBSTACLE_DATA
{
    float x;
    float y;
    float r;
};

using namespace std;
extern vector<OBSTACLE_DATA> obstacle;

namespace VelodyneSensor
{
struct VISION_DATA
{
    unsigned long long timeStamp;
    float gridMap[400][400];
    int obstacleMap[400][400];
};

class VELODYNE
{
public:
    VELODYNE();
    ~VELODYNE();

    void Start();
    int Update(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax,float r,float d);
    void UpdateO();
    void SavePcd();
    void Stop();
    VISION_DATA visData;

private:
   class VELODYNE_STRUCT;
   std::auto_ptr<VELODYNE_STRUCT> mVelodyneStruct;
};
}

#endif // VELODYNE_H
