#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/transform.h"
#include "nabo/nabo.h"
#include "eigen_conversions/eigen_msg.h"

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
    void postFilter();
    int findThePoint(Eigen::Vector3f input);
    float calculateDist(Eigen::Vector3f inputA, Eigen::Vector3f inputB);

private:

    string mapFileName;
    string scanFileName;
    string poseFileName;

    PM::DataPoints *mapPointCloud;
    PM::DataPoints *scanPointCloud;
    PM::DataPoints outlierScan;
    PM::DataPoints scanInRangeCloud;
    PM::DataPoints finalScan;

    ros::Publisher mapPointCloudPub;
    ros::Publisher scanPointCloudPub;
    ros::Publisher outlierScanPub;
    ros::Publisher scanInRangePub;
    ros::Publisher finalScanPub;

    ros::NodeHandle& n;
    PM::TransformationParameters TRobotToMap;
    unique_ptr<PM::Transformation> transformation;

    const float distanceThreshold;
    const float rangeThreshold;
    const float growingThreshold;
    const int filterPointsInt;

    int clusterCount = 0;  //label of cluster
    vector<int> clusterPointsNum;
    bool isCout;

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
  growingThreshold(getParam<double>("growingThreshold", 0.5)),
  isCout(getParam<bool>("isCout", false)),
  filterPointsInt(getParam<int>("filterPointsInt", 3))
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
    finalScanPub = n.advertise<sensor_msgs::PointCloud2>("final_scan_points", 2, true);

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

    if(isCout)
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

}

void Detection::cluster()
{
    outlierScan.addDescriptor("cluster", PM::Matrix::Zero(1, outlierScan.features.cols()));
    const int rowLine = outlierScan.getDescriptorStartingRow("cluster");

    const int outlierCount(outlierScan.features.cols());
    if(isCout)
    {
        cout<<"scan Num: "<<scanPointCloud->features.cols()<<endl;
        cout<<"Map Num: "<<mapPointCloud->features.cols()<<endl;
        cout<<"outlier Num: "<<outlierCount<<endl;
    }


    //initial donePoints
    DP outlierScanTemp = outlierScan;
    DP outlierScanTempTemp, thePointCloud;
    int thePointIndex = 0;
    shared_ptr<NNS> forceNNS;
    int clusterPointsCount = 1; //the num of points in one cluster

    for(int i = 1; i < outlierCount + 1; i++)
    {
        if(isCout)
            cout<<"iter:  "<<i;

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

                int indexInOutlierScan = findThePoint(outlierScanTemp.features.col(j).head(3));
                if(indexInOutlierScan != -1)
                {
                    outlierScan.descriptors(rowLine, indexInOutlierScan) = clusterCount;
                }

            }
        }
        outlierScanTempTemp.conservativeResize(count);
        thePointCloud.conservativeResize(1);

        outlierScanTemp = outlierScanTempTemp;

        if(isCout)
            cout<<"  remain Points-kd:  "<<outlierScanTemp.features.cols();

        //last point could not use NNS in kd-tree
        if(i == outlierCount - 1)
        {
           Eigen::Vector3f inputA = thePointCloud.features.col(0).head(3);
           Eigen::Vector3f inputB = outlierScanTemp.features.col(0).head(3);

           if(this->calculateDist(inputA, inputB) > growingThreshold)
           {

               clusterCount++;
               clusterPointsNum.push_back(clusterPointsCount);
               clusterPointsCount = 1;
           }
           else
           {
               clusterPointsCount ++;
           }

           thePointIndex = 0;

           continue;
        }

        if(i == outlierCount)
        {
            clusterPointsNum.push_back(clusterPointsCount);
            break;
        }
        //NNS
        forceNNS.reset( NNS::create(outlierScanTemp.features, outlierScanTemp.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
        PM::Matches matches_overlap(
            Matches::Dists(1, 1),
            Matches::Ids(1, 1)
        );

        forceNNS->knn(thePointCloud.features, matches_overlap.ids, matches_overlap.dists, 1, 0);

        thePointIndex = matches_overlap.ids(0);

        if(matches_overlap.dists(0) > growingThreshold)
        {
            clusterCount++;
            clusterPointsNum.push_back(clusterPointsCount);
            clusterPointsCount = 1;
        }
        else
        {
            clusterPointsCount ++;
        }

        if(isCout)
            cout<<"  num in Cluster:  "<<clusterPointsCount<<endl;
    }


    if(isCout)
        cout<<"   CLUSTERED:  "<<clusterCount<<endl;

}

float Detection::calculateDist(Eigen::Vector3f inputA, Eigen::Vector3f inputB)
{
    return pow(inputA(0)-inputB(0),2) + pow(inputA(1)-inputB(1),2) + pow(inputA(2)-inputB(2),2);
}

void Detection::postFilter()
{
    const int outlierCount(outlierScan.features.cols());
    const int rowLine = outlierScan.getDescriptorStartingRow("cluster");

    vector<int> weedOut;

    for(int i = 0; i <= clusterCount; i++)
    {
        if(isCout)
        {
            cout<<"---------------------------------------"<<endl;
            cout<<"clusterNum: "<<clusterPointsNum.at(i)<<endl;
            cout<<"Ratio:  "<<(double)clusterPointsNum.at(i) / (double)outlierCount<<endl;
            cout<<"ratioThreshold:  "<<(double)1 / (double)(clusterCount * filterPointsInt)<<endl;
        }
        if((double)clusterPointsNum.at(i) / (double)outlierCount
                <
                (double)1 / (double)(clusterCount * filterPointsInt) )
        {
            if(isCout)
                cout<<"weed OUT:  "<<i<<endl;
            weedOut.push_back(i);
        }
    }

    finalScan = outlierScan.createSimilarEmpty();;
    int count = 0;

    for(int i = 0; i < outlierCount; i++)
    {
        vector<int>::iterator it= find(weedOut.begin(), weedOut.end(), outlierScan.descriptors(rowLine, i));
        if(it != weedOut.end())
        {
            continue;
        }
        else
        {
            finalScan.setColFrom(count, outlierScan, i);
            count++;
        }
    }

    finalScan.conservativeResize(count);

    if(isCout)
        cout<<"After filter:  "<<finalScan.features.cols()<<endl;

}

int Detection::findThePoint(Eigen::Vector3f input)
{
    const int outlierCount(outlierScan.features.cols());

    for(int i = 0; i < outlierCount; i++)
    {
        if(outlierScan.features.col(i).head(3) == input)
        {
            return i;
        }
    }

    return -1;
}

void Detection::publish()
{
    scanInRangePub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scanInRangeCloud, "map", ros::Time::now()));
    outlierScanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(outlierScan, "map", ros::Time::now()));
    mapPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*mapPointCloud, "map", ros::Time::now()));
    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(*scanPointCloud, "map", ros::Time::now()));
    finalScanPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(finalScan, "map", ros::Time::now()));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_detection");
    ros::NodeHandle n;
    Detection detection(n);

    detection.extract();
    detection.cluster();
    detection.postFilter();

//    ros::Rate r(10);
//    while(ros::ok())
//    {
        detection.publish();
//        r.sleep();
//    }

    ros::spin();

    return 0;
}
