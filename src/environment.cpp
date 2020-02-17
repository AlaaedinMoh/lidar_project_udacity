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

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 

    // TODO:: Create point processor
  
}

void ProcessProjectBlock(PointCloud<PointXYZI>::Ptr cloud, ProcessPointClouds<PointXYZI> processor, visualization::PCLVisualizer::Ptr& viewer)
{
    Eigen::Vector4f min_f(-12, -7, -2, 1);
    Eigen::Vector4f max_f(25, 7, 20, 1);
    // PointCloud<PointXYZI>::Ptr filteredCloud = processor.CustomFilterCloud(cloud, 0.2f, min_f, max_f);
    PointCloud<PointXYZI>::Ptr filteredCloud = processor.FilterCloud(cloud, 0.2f, min_f, max_f);
    // renderPointCloud(viewer, filteredCloud, "filtered cloud", Color(0,1,0));
    // pair<PointCloud<PointXYZI>::Ptr, PointCloud<PointXYZI>::Ptr> customClouds = processor.CustomSegmentPlane(cloud, 100, 0.2f);
    pair<PointCloud<PointXYZI>::Ptr, PointCloud<PointXYZI>::Ptr> segRslt = processor.SegmentPlane(filteredCloud, 20, 0.5f);
    // renderPointCloud(viewer, filteredCloud, "filtered cloud", Color(0,1,0));
    cout<<"inliniers size = "<<segRslt.first->points.size()<<endl;
    cout<<"Obstacles size = "<<segRslt.second->points.size()<<endl;
    // renderPointCloud(viewer, customClouds.first, "Obstacles", Color(1, 0, 0));
    // renderPointCloud(viewer, customClouds.second, "Plane", Color(0, 1, 0));
    renderPointCloud(viewer, segRslt.first, "inliers", Color(1,0,0));
    renderPointCloud(viewer, segRslt.second, "obstacles", Color(0,1,0));
}

void ProjectStream(visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<PointXYZI>* customProcessor = new ProcessPointClouds<PointXYZI>();
    PointCloud<PointXYZI>::Ptr inputCloud;
    vector<boost::filesystem::path> stream = customProcessor->streamPcd("../src/sensors/data/pcd/data_1/");
    auto streamItr = stream.begin();
    inputCloud=customProcessor->loadPcd((*streamItr).string());
    // inputCloud=customProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000001.pcd");

    ProcessProjectBlock(inputCloud, *customProcessor, viewer);
    viewer->spinOnce();
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
        // simpleHighway(viewer);
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