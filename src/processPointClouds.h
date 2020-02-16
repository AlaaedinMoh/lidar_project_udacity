// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include<unordered_set>
#include "render/box.h"

using namespace std;
using namespace pcl;

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename PointCloud<PointT>::Ptr cluster);

    void savePcd(typename PointCloud<PointT>::Ptr cloud, string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(string file);

    vector<boost::filesystem::path> streamPcd(string dataPath);
    
    ///Project Methods

    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> CustomSeparateClouds(PointIndices::Ptr inliers, typename PointCloud<PointT>::Ptr cloud);

    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> CustomSegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIteration, float distThreshold);

    vector<typename PointCloud<PointT>::Ptr> CustomClustering(typename PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    unordered_set<int> DoRasnan3D(typename PointCloud<PointT>::Ptr cloud, int maxIteration, float distanceTol);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */