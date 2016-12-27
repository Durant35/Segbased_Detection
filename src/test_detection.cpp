#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/transform.h"
#include "nabo/nabo.h"

#include <tf/transform_broadcaster.h>

#include <fstream>

using namespace std;
using namespace PointMatcherSupport;

class Detection
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

public:
    Detection(ros::NodeHandle &n);
    ~Detection();
    void process();
    void publish();


private:

    string mapFileName;
    string scanFileName;
    string poseFileName;
    PM::DataPoints *mapPointCloud;
    PM::DataPoints *scanPointCloud;

    ros::Publisher mapPointCloudPub;
    ros::Publisher scanPointCloudPub;
    ros::NodeHandle& n;
    PM::TransformationParameters TRobotToMap;
    unique_ptr<PM::Transformation> transformation;

protected:


};

Detection::Detection(ros::NodeHandle& n):
  n(n),
  mapFileName(getParam<string>("map_vtk", ".")),
  scanFileName(getParam<string>("scan_vtk", ".")),
  poseFileName(getParam<string>("pose", ".")),
  transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{

    //load map vtk
    try
    {
        DP* cloud(new DP(DP::load(mapFileName)));
        mapPointCloud = cloud;
    }
    catch(std::exception &)
    {
        ROS_ERROR_STREAM("Cannot load Reference Map from vtk file " << mapFileName);
    }

    //load scan vtk
    try
    {
        DP* cloud(new DP(DP::load(scanFileName)));
        scanPointCloud = cloud;
    }
    catch(std::exception &)
    {
        ROS_ERROR_STREAM("Cannot load Scan PointCloud from vtk file " << mapFileName);
    }

    //Pubs
    mapPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("reference_map_points", 2, true);
    scanPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("raw_scan_points", 2, true);
    TRobotToMap.setZero(4,4);

    ifstream in;
    in.open(poseFileName.c_str());
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            in >> TRobotToMap(i,j);
    in.close();
}

Detection::~Detection()
{}

void Detection::process()
{

    *scanPointCloud = transformation->compute(*scanPointCloud, TRobotToMap);





    //for Publish new Cloud
    //NEW CLOUD?

    //start knn
    const int readPtsCount(scanPointCloud->features.cols());
    const int mapPtsCount(mapPointCloud->features.cols());

    // Build a range image of the reading point cloud (local coordinates)
    PM::Matrix radius_reading = scanPointCloud->features.topRows(3).colwise().norm();
    PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

    // No atan in Eigen, so we are for to loop through it...
    for(int i=0; i<readPtsCount; i++)
    {
        const float ratio = scanPointCloud->features(2,i)/radius_reading(0,i);
        //if(ratio < -1 || ratio > 1)
            //cout << "Error angle!" << endl;

        angles_reading(0,i) = acos(ratio);
        angles_reading(1,i) = atan2(scanPointCloud->features(1,i), scanPointCloud->features(0,i));
    }

    //Initial NNS
    shared_ptr<NNS> featureNNS;
    featureNNS.reset( NNS::create(angles_reading));

    // Remove points out of sensor range
    // FIXME: this is a parameter
    const float sensorMaxRange = 80.0;
    PM::Matrix globalId(1, mapPtsCount);

    int mapCutPtsCount = 0;
    DP mapLocalFrameCut(mapPointCloud->createSimilarEmpty());
    for (int i = 0; i < mapPtsCount; i++)
    {
        if (mapPointCloud->features.col(i).head(3).norm() < sensorMaxRange)
        {
            mapLocalFrameCut.setColFrom(mapCutPtsCount, *mapPointCloud, i);
            globalId(0,mapCutPtsCount) = i;
            mapCutPtsCount++;
        }
    }

    mapLocalFrameCut.conservativeResize(mapCutPtsCount);

    //SAME AS READING CLOUD
    PM::Matrix radius_map = mapPointCloud->features.topRows(3).colwise().norm();

    PM::Matrix angles_map(2, mapCutPtsCount); // 0=inclination, 1=azimuth

    // No atan in Eigen, so we are for to loop through it...
    for(int i=0; i<mapCutPtsCount; i++)
    {
        const float ratio = mapLocalFrameCut.features(2,i)/radius_map(0,i);
        //if(ratio < -1 || ratio > 1)
            //cout << "Error angle!" << endl;

        angles_map(0,i) = acos(ratio);

        angles_map(1,i) = atan2(mapLocalFrameCut.features(1,i), mapLocalFrameCut.features(0,i));
    }

}

void Detection::publish()
{

    mapPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, "map", ros::Time::now()));
    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*scanPointCloud, "map", ros::Time::now()));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_detection");
    ros::NodeHandle n;
    Detection detection(n);

    detection.process();

    ros::Rate r(10);
    while(ros::ok())
    {
        detection.publish();
        r.sleep();
    }

    ros::spin();

    return 0;
}
