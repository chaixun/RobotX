#include "Velodyne.h"

using namespace pcl;
vector<OBSTACLE_DATA> obstacle;
char isFound[2]={0};

namespace VelodyneSensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

pcl::PassThrough<PointXYZ> pass;

void GenPointCoud(const CloudConstPtr &rawCloud, CloudPtr &adjCloud)
{
    cout<<"Point: "<<rawCloud->points.front().x<<" "<<rawCloud->points.front().y<<" "<< rawCloud->points.front().z<<endl;

    Eigen::Matrix4f matrixSTS;
    matrixSTS << -1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTR;
    matrixSTR <<  0.9999, 0.0132, 0.0088, 0.0108,
            -0.0033, 0.7145, -0.6996, 0.2615,
            -0.0155, 0.6995, 0.7144, 0.5368,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixRTG;
    matrixRTG <<  1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTG;
    matrixSTG = matrixRTG * matrixSTR * matrixSTS;

    pcl::transformPointCloud(*rawCloud, *adjCloud, matrixSTG);
}

void GenGridMap(const CloudPtr &adjCloud, VISION_DATA &cdata)
{
    int cGridNum[400][400] = {0};

    for (size_t i = 0; i < adjCloud->points.size(); ++i)
    {
        if(adjCloud->points[i].x > -2 && adjCloud->points[i].x < 2 &&
                adjCloud->points[i].z > 0 && adjCloud->points[i].z < 4)
        {
            int m = 0, n = 0;
            n = floor(adjCloud->points[i].x / 0.01) + 200;
            m = floor(adjCloud->points[i].z / 0.01);

            //Mean
            cdata.gridMap[m][n] = (cdata.gridMap[m][n] * cGridNum[m][n] + adjCloud->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }
}

void GenObstacleMap(VISION_DATA &cdata)
{
    for(int i =0; i < 400; i++)
    {
        for(int j = 0; j < 400; j++)
        {
            if(cdata.gridMap[i][j] > 0.1)
            {
                cdata.obstacleMap[i][j] = 1;
            }
        }
    }
}

class VELODYNE::VELODYNE_STRUCT
{
    friend class VELODYNE;
private:
    void Init();
    void Release();
    VLPGrabber grabber;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    boost::signals2::connection cloud_connection;
    void cloud_callback(const CloudConstPtr& cloud);
    int frameNum = 0;
};

VELODYNE::VELODYNE():mVelodyneStruct(new VELODYNE::VELODYNE_STRUCT)
{
    ;
}

VELODYNE::~VELODYNE()
{
    ;
}

void VELODYNE::Start()
{
    mVelodyneStruct->Init();
    cout<<"Device Open ! "<<endl;
    for (int i = 0; i < 7; i++)
    {
        UpdateO();
    }
    cout<<"Clear !"<<endl;
}

void VELODYNE::Stop()
{
    mVelodyneStruct->Release();
    cout<<"Device Close ! "<<endl;
}

void VELODYNE::SavePcd()
{


}

void VELODYNE::UpdateO()
{
    while(true)
    {
        CloudConstPtr rawCloud;
        if(mVelodyneStruct->cloud_mutex_.try_lock())
        {

            mVelodyneStruct->cloud_.swap(rawCloud);
            mVelodyneStruct->cloud_mutex_.unlock();
        }
        if(rawCloud)
        {
            cout<<"Have got data"<<endl;
            break;
        }
    }
}

int VELODYNE::Update(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax,float r,float d)
{
    vector<CloudPtr> obstacleCloud;
    obstacle.clear();
    isFound[0]=0;
    isFound[1]=0;
    //isFound[2]={0};
    while(true)
    {
        CloudConstPtr rawCloud;
        if(mVelodyneStruct->cloud_mutex_.try_lock())
        {

            mVelodyneStruct->cloud_.swap(rawCloud);
            mVelodyneStruct->cloud_mutex_.unlock();
        }
        if(rawCloud)
        {
            pcl::PointCloud<PointXYZ>::Ptr m_rawCloud (new pcl::PointCloud<PointXYZ>);

            pcl::PointCloud<PointXYZ>::Ptr x_cloud_filtered (new pcl::PointCloud<PointXYZ>);
            pcl::PointCloud<PointXYZ>::Ptr z_cloud_filtered (new pcl::PointCloud<PointXYZ>);
            std::cout<<"Update Begin!"<<endl;
            if(rawCloud->size()<3)
            {
                return 0;
            }
            pass.setInputCloud (rawCloud);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (0,20);
            pass.filter (*m_rawCloud);

            if(m_rawCloud->size()<3)
            {
                return 0;
            }
            pass.setInputCloud (m_rawCloud);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (xMin, xMax);
            pass.filter (*x_cloud_filtered);

            if(x_cloud_filtered->size()<3)
            {
                return 0;
            }
            pass.setInputCloud (x_cloud_filtered);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (zMin, zMax);
            pass.filter (*z_cloud_filtered);
            std::cout<<" xmin: "<<xMin<<" xmax: "<<xMax<<std::endl;

            std::cout<<"Phase 1 !"<<endl;
            for(int i=0; i<2; i++)
            {
                pcl::PointCloud<PointXYZ>::Ptr y_cloud_filtered (new pcl::PointCloud<PointXYZ>);

                if(z_cloud_filtered->size()<3)
                {
                    return 0;
                }
                pass.setInputCloud (z_cloud_filtered);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (d*i+yMin,d*i+yMax);
                std::cout<<" ymin: "<<d*i+yMin<<" ymax: "<<d*i+yMax<<std::endl;
                pass.filter (*y_cloud_filtered);
                if(y_cloud_filtered->size()>2)
                {
                    isFound[i]=1;
                    obstacleCloud.push_back(y_cloud_filtered);
                    std::cout<<"Phase1 "<<i<<": "<<"Found "<<y_cloud_filtered->size()<<" Points"<<endl;
                }
                else
                {
                    isFound[i]=0;
                    return 0;
                }
            }
            std::cout<<"Phase 2 !"<<endl;
            int zy_i=0;
            for(zy_i=0; zy_i<2; zy_i++)
            {
                OBSTACLE_DATA temp;
                temp.x=0;
                temp.y=0;
                temp.r=0;
                if(1==isFound[zy_i])
                {
                    /*
                    // 创建一个系数为X=Y=0,Z=1的平面
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
                    coefficients->values.resize (4);
                    coefficients->values[0] = 0.0;
                    coefficients->values[1] = 0.0;
                    coefficients->values[2] = 1.0;
                    coefficients->values[3] = 0.0;
                    // 创建滤波器对象
                    pcl::Poin`tCloud<PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<PointXYZ>);
                    pcl::ProjectInliers<pcl::PointXYZ> proj;
                    proj.setModelType (pcl::SACMODEL_PLANE);
                    proj.setInputCloud (obstacleCloud.at(i));
                    std::cout<<"size: "<< obstacleCloud.at(i)->size()<<std::endl;
                    proj.setModelCoefficients (coefficients);
                    proj.filter (*cloud_projected);
                    std::cout<<"size: "<< cloud_projected->size()<<std::endl;
                    */
                    //nearest point
                    int number=0;
                    float lastDis=100;
                    float thisDis=0;
                    for(int i=0;i<obstacleCloud.at(zy_i)->size();i++)
                    {
                        //static float lastDis=100;
                        //static float thisDis=0;
                        //thisDis = sqrt(cloud_projected->points[i].x*cloud_projected->points[i].x+cloud_projected->points[i].y*cloud_projected->points[i].y);
                        thisDis = sqrt(obstacleCloud.at(zy_i)->points[i].x*obstacleCloud.at(zy_i)->points[i].x+obstacleCloud.at(zy_i)->points[i].y*obstacleCloud.at(zy_i)->points[i].y);
                        if(thisDis<lastDis)
                        {
                            number = i;
                            lastDis=thisDis;
                        }
                    }
                    //caculate circle
                    float distance = 0.0;
                    //distance = sqrt(cloud_projected->points[number].x*cloud_projected->points[number].x+cloud_projected->points[number].y*cloud_projected->points[number].y);
                    //temp.r=r;
                    //temp.x=cloud_projected->points[number].x*(distance+temp.r)/distance;
                    //temp.y=cloud_projected->points[number].y*(distance+temp.r)/distance;;
                    distance = sqrt(obstacleCloud.at(zy_i)->points[number].x*obstacleCloud.at(zy_i)->points[number].x+obstacleCloud.at(zy_i)->points[number].y*obstacleCloud.at(zy_i)->points[number].y);
                    temp.r=r;
                    temp.x=obstacleCloud.at(zy_i)->points[number].x*(distance+temp.r)/distance;
                    temp.y=obstacleCloud.at(zy_i)->points[number].y*(distance+temp.r)/distance;;
                    obstacle.push_back(temp);
                }
                else
                {
                    return 0;
                }

            }
            std::cout<<"isFound: ";
            for(int i=0;i<2;i++)
            {
                std::cout<<(int)isFound[i]<<" ";
            }
            std::cout<<endl;
            break;
        }
    }
    return 1;
}

void VELODYNE::VELODYNE_STRUCT::cloud_callback(const CloudConstPtr& cloud)
{
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_ = cloud;
}

void VELODYNE::VELODYNE_STRUCT::Init()
{
    boost::function <void (const CloudConstPtr&)> cloud_cb = boost::bind(& VELODYNE::VELODYNE_STRUCT::cloud_callback, this, _1);
    this->cloud_connection = this->grabber.registerCallback(cloud_cb);
    this->grabber.start();
}

void VELODYNE::VELODYNE_STRUCT::Release()
{
    this->grabber.stop();
    this->cloud_connection.disconnect();
}

}

