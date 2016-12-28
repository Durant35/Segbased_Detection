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
#include <vector>
#include <algorithm>

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
    void extract();
    void publish();
    void cluster();

private:

    string mapFileName;
    string scanFileName;
    string poseFileName;
    PM::DataPoints *mapPointCloud;
    PM::DataPoints *scanPointCloud;
    PM::DataPoints outlierScan;
    PM::DataPoints scanInRangeCloud;

    ros::Publisher mapPointCloudPub;
    ros::Publisher scanPointCloudPub;
    ros::Publisher outlierScanPub;
    ros::Publisher scanInRangePub;
    ros::NodeHandle& n;
    PM::TransformationParameters TRobotToMap;
    unique_ptr<PM::Transformation> transformation;

    const float distanceThreshold;
    const float rangeThreshold;
    const float growingThreshold;

protected:


};

Detection::Detection(ros::NodeHandle& n):
  n(n),
  mapFileName(getParam<string>("map_vtk", ".")),
  scanFileName(getParam<string>("scan_vtk", ".")),
  poseFileName(getParam<string>("pose", ".")),
  transformation(PM::get().REG(Transformation).create("RigidTransformation")),
  distanceThreshold(getParam<double>("distanceThreshold", 1.0)),
  rangeThreshold(getParam<double>("rangeThreshold", 20.0)),
  growingThreshold(getParam<double>("growingThreshold", 0.5))
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
    scanInRangePub = n.advertise<sensor_msgs::PointCloud2>("scan_inrange_points", 2, true);
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

void Detection::extract()
{

    cout<<"scan Num: "<<scanPointCloud->features.cols()<<endl;
    cout<<"Map Num: "<<mapPointCloud->features.cols()<<endl;

    const int scanPtsCount(scanPointCloud->features.cols());

    scanInRangeCloud = scanPointCloud->createSimilarEmpty();
    int scanInRangeCount = 0;

    for(int i =0; i < scanPtsCount; i++)
    {
        if(scanPointCloud->features.col(i).head(3).norm() < rangeThreshold)
        {
            scanInRangeCloud.setColFrom(scanInRangeCount, *scanPointCloud, i);
            scanInRangeCount++;
        }
    }

    cout<<"scan In Range Num: "<<scanInRangeCount<<endl;

    scanInRangeCloud.conservativeResize(scanInRangeCount);

    *scanPointCloud = transformation->compute(*scanPointCloud, TRobotToMap);
    scanInRangeCloud = transformation->compute(scanInRangeCloud, TRobotToMap);

    //NNS
    shared_ptr<NNS> featureNNS;
    featureNNS.reset( NNS::create(mapPointCloud->features, mapPointCloud->features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));

    PM::Matches matches_overlap(
        Matches::Dists(1, scanInRangeCount),
        Matches::Ids(1, scanInRangeCount)
    );

    featureNNS->knn(scanInRangeCloud.features, matches_overlap.ids, matches_overlap.dists, 1, 0);


    //filter According to E-distance
    outlierScan = scanInRangeCloud.createSimilarEmpty();
    int outlierCount = 0;

    for(int i = 0; i < scanInRangeCount; i++)
    {
        if(matches_overlap.dists(i) > distanceThreshold)
        {
            outlierScan.setColFrom(outlierCount, scanInRangeCloud, i);
            outlierCount++;
        }
    }

    outlierScan.conservativeResize(outlierCount);

    cout<<"outlier Num: "<<outlierCount<<endl;

}

void Detection::cluster()
{

    const int outlierCount(outlierScan.features.cols());


    //initial donePoints
    DP outlierScanTemp = outlierScan;
    DP outlierScanTempTemp, thePointCloud;
    int thePointIndex = 0;
    shared_ptr<NNS> forceNNS;
    int cluster = 0;

    for(int i = 1; i < outlierCount - 1; i++)
    {

        //create the new DP-kd-tree form the remaining, delete the thePoint by Index, move it to thePointCloud
        int remainPointCount(outlierScanTemp.features.cols());
        int count = 0;
        outlierScanTempTemp = outlierScanTemp.createSimilarEmpty();
        thePointCloud = outlierScanTemp.createSimilarEmpty();

        for(int j = 0; j < remainPointCount; j++)
        {
            if(j != thePointIndex)
            {
                outlierScanTempTemp.setColFrom(count, outlierScanTemp, j);
                count++;
            }
            else
            {
                thePointCloud.setColFrom(0, outlierScanTemp, j);
            }
        }
        outlierScanTempTemp.conservativeResize(count);
        cout<<"count:  "<<count<<endl;
        thePointCloud.conservativeResize(1);

        outlierScanTemp = outlierScanTempTemp;


        //NNS
        forceNNS.reset( NNS::create(outlierScanTemp.features, outlierScanTemp.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
        PM::Matches matches_overlap(
            Matches::Dists(1, 1),
            Matches::Ids(1, 1)
        );

        forceNNS->knn(thePointCloud.features, matches_overlap.ids, matches_overlap.dists, 1, 0);

        cout<<i<<" dis: "<<matches_overlap.dists(0)<<endl;//<<" itn:  "<<matches_overlap.ids(0)//<<"  "<<matches_overlap.ids(0,0)

        thePointIndex = matches_overlap.ids(0);

        if(matches_overlap.dists(0) > growingThreshold)
        {
            cluster++;
        }


    }

    cout<<"clustered:  "<<cluster<<endl;

}

void Detection::publish()
{
    scanInRangePub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scanInRangeCloud, "map", ros::Time::now()));
    outlierScanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(outlierScan, "map", ros::Time::now()));
    mapPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, "map", ros::Time::now()));
    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*scanPointCloud, "map", ros::Time::now()));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_detection");
    ros::NodeHandle n;
    Detection detection(n);

    detection.extract();
    detection.cluster();

    ros::Rate r(10);
    while(ros::ok())
    {
        detection.publish();
        r.sleep();
    }

    ros::spin();

    return 0;
}
