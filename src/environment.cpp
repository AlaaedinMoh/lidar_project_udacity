/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <stdio.h>

using namespace std;
using namespace pcl;

vector<Color> colors = {Color(1,1,0),Color(0,1,1), Color(1,1,1), Color(0.5,0.5,0), Color(0,0.5,0.5)};

void ProcessProjectBlock(PointCloud<PointXYZI>::Ptr cloud, ProcessPointClouds<PointXYZI> processor, visualization::PCLVisualizer::Ptr& viewer)
{
    Eigen::Vector4f min_f(-12, -6.5, -1.8, 1);
    Eigen::Vector4f max_f(35, 6.5, 5, 1);
    PointCloud<PointXYZI>::Ptr filteredCloud = processor.FilterCloud(cloud, 0.1f, min_f, max_f);
    pair<PointCloud<PointXYZI>::Ptr, PointCloud<PointXYZI>::Ptr> segRslt = processor.SegmentPlane(filteredCloud, 50, 0.1f);
    vector<PointCloud<PointXYZI>::Ptr> rsltClusters = processor.Clustering(segRslt.second, 0.45, 3, 30);
    renderPointCloud(viewer, segRslt.first, "inliers", Color(0,1,0));
    int i = 0;
    for(auto cluster : rsltClusters)
    {
        renderPointCloud(viewer, cluster, "cluster:" + std::to_string(i), colors[i%5]);
        Box cBox = processor.BoundingBox(cluster);
        renderBox(viewer, cBox, i);
        ++i;
    }
}

void ProjectStream(visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<PointXYZI>* customProcessor = new ProcessPointClouds<PointXYZI>();
    vector<boost::filesystem::path> stream = customProcessor->streamPcd("../src/sensors/data/pcd/data_1/");
    
    // PointCloud<PointXYZI>::Ptr streamCloud=customProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000001.pcd");
    // ProcessProjectBlock(streamCloud, *customProcessor, viewer);    

    auto streamItr = stream.begin();
    PointCloud<PointXYZI>::Ptr streamCloud;
    while(!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        streamCloud=customProcessor->loadPcd((*streamItr).string());
        streamItr++;
        if(streamItr==stream.end())
            streamItr=stream.begin();

        ProcessProjectBlock(streamCloud, *customProcessor, viewer);
        viewer->spinOnce();
    }
    
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    try
    {
        std::cout << "starting enviroment" << std::endl;

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        CameraAngle setAngle = XY;
        initCamera(setAngle, viewer);
        ProjectStream(viewer);

        while (!viewer->wasStopped ())
        {
           viewer->spinOnce ();
        } 
    }
    catch(exception& exp)
    {
        cout << exp.what() <<endl;
    }
}