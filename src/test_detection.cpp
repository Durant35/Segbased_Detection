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
    typedef PM::Matches Matches;

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
    PM::DataPoints outlierScan;

    ros::Publisher mapPointCloudPub;
    ros::Publisher scanPointCloudPub;
    ros::Publisher outlierScanPub;
    ros::NodeHandle& n;
    PM::TransformationParameters TRobotToMap;
    unique_ptr<PM::Transformation> transformation;

    const float distanceThreshold;

protected:


};

Detection::Detection(ros::NodeHandle& n):
  n(n),
  mapFileName(getParam<string>("map_vtk", ".")),
  scanFileName(getParam<string>("scan_vtk", ".")),
  poseFileName(getParam<string>("pose", ".")),
  transformation(PM::get().REG(Transformation).create("RigidTransformation")),
  distanceThreshold(getParam<double>("distanceThreshold", 1.0))
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
    outlierScanPub = n.advertise<sensor_msgs::PointCloud2>("outlier_scan_points", 2, true);
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
    cout<<"scan Num: "<<scanPointCloud->features.cols()<<endl;
    cout<<"Map Num: "<<mapPointCloud->features.cols()<<endl;

    //start knn
    const int scanPtsCount(scanPointCloud->features.cols());
    const int mapPtsCount(mapPointCloud->features.cols());






    std::shared_ptr<NNS> featureNNS;
    featureNNS.reset( NNS::create(mapPointCloud->features, mapPointCloud->features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

    PM::Matches matches_overlap(
        Matches::Dists(1, scanPtsCount),
        Matches::Ids(1, scanPtsCount)
    );

    featureNNS->knn(scanPointCloud->features, matches_overlap.ids, matches_overlap.dists, 1, 0);


    outlierScan = scanPointCloud->createSimilarEmpty();
    int outlierCount = 0;

    for(int i = 0; i < scanPtsCount; i++)
    {
        if(matches_overlap.dists(i) > distanceThreshold)
        {
            outlierScan.setColFrom(outlierCount, *scanPointCloud, i);
            outlierCount++;
        }
    }

    outlierScan.conservativeResize(outlierCount);

    cout<<"outlier Num: "<<outlierCount<<endl;


}

void Detection::publish()
{
    outlierScanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(outlierScan, "map", ros::Time::now()));
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
