// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

using namespace std;
using namespace pcl;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::VoxelGrid<PointT> vxl;
    typename PointCloud<PointT>::Ptr filteredCloud (new PointCloud<PointT> ());
    vxl.setInputCloud(cloud);
    vxl.setLeafSize(filterRes, filterRes, filterRes);
    vxl.filter(*filteredCloud);

    typename PointCloud<PointT>::Ptr cloudRegion (new PointCloud<PointT> ());
    CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud (filteredCloud);
    region.filter(*cloudRegion);

    return cloudRegion;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  typename PointCloud<PointT>::Ptr obstacleCld {new PointCloud<PointT>};
  typename PointCloud<PointT>::Ptr planeCld {new PointCloud<PointT>};

  for(int idx : inliers->indices)
  {
      planeCld->points.push_back(cloud->points[idx]);
  }

  ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacleCld);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCld, planeCld);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    unordered_set<int> inliersRslt;
    ///////////////////////////////////////////////
    srand(time(NULL));
    for(int i = 0; i < maxIterations; i++)
    {
        unordered_set<int> inliers;
        while(inliers.size()<3)
        {
            inliers.insert(rand()%cloud->points.size());
        }
        pcl::PointXYZ point1;
        pcl::PointXYZ point2;
        pcl::PointXYZ point3;

        auto iter = inliers.begin();
        point1.x = cloud->points[*iter].x;
        point1.y = cloud->points[*iter].y;
        point1.z = cloud->points[*iter].z;
        iter++;
        point2.x = cloud->points[*iter].x;
        point2.y = cloud->points[*iter].y;
        point2.z = cloud->points[*iter].z;
        iter++;
        point3.x = cloud->points[*iter].x;
        point3.y = cloud->points[*iter].y;
        point3.z = cloud->points[*iter].z;
        
        Plane3d plane;
        plane.A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        plane.B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        plane.C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        plane.D = -(point1.x*plane.A + point1.y*plane.B + point1.z*plane.C);

        for(int idx = 0; idx < cloud->points.size(); idx++)
        {
            if(inliers.count(idx) > 0)
            {
                continue;
            }
            PointT point = cloud->points[idx];
            float distance = fabs(point.x*plane.A + point.y*plane.B + point.z*plane.C + plane.D) / sqrt(plane.A*plane.A + plane.B*plane.B + plane.C*plane.C);
            if(distance <= distanceThreshold)
            {
                inliers.insert(idx);
            }
        }
        if(inliers.size() > inliersRslt.size())
        {
            inliersRslt = inliers;
        }
    }
    ///////////////////////////////////////////////

    typename PointCloud<PointT>::Ptr inliersCld(new PointCloud<PointT>());
    typename PointCloud<PointT>::Ptr obstaclesCld(new PointCloud<PointT>());
    for(int idx = 0; idx < cloud->points.size(); idx++)
    {
        PointT point = cloud->points[idx];
        if(inliersRslt.count(idx))
            inliersCld->points.push_back(point);
        else
            obstaclesCld->points.push_back(point);
    }
    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segRslt;
    segRslt.first = inliersCld;
    segRslt.second = obstaclesCld;
    return segRslt;
}

template<typename PointT>
void ProcessPointClouds<PointT>::ClusteringHelper(int pointIndex, typename PointCloud<PointT>::Ptr obstaclesCloud, typename PointCloud<PointT>::Ptr cluster, vector<bool>& processed, EuclideanKdTree<PointT>* eTree, float dist)
{
     processed[pointIndex] = true;
     cluster->points.push_back(obstaclesCloud->points[pointIndex]);
     vector<int> resultIds = eTree->search(obstaclesCloud->points[pointIndex], dist);
     for(int id : resultIds)
     {
         if(!processed[id])
            ClusteringHelper(id, obstaclesCloud, cluster, processed, eTree, dist);
     }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    vector<typename PointCloud<PointT>::Ptr> clusters;
    vector<bool> processed(cloud->points.size(), false);
    EuclideanKdTree<PointT>* eTree = new EuclideanKdTree<PointT>();
    for(int i = 0; i < cloud->points.size(); i++)
        eTree->insert(cloud->points[i], i);
    int i = 1;
    for(int idx = 0; idx < cloud->points.size(); idx++)
    {
        if(processed[idx])
            continue;
      typename pcl::PointCloud<PointT>::Ptr cluster(new PointCloud<PointT>());
      ClusteringHelper(idx, cloud, cluster, processed, eTree, clusterTolerance);
      clusters.push_back(cluster);
    }
    eTree->freeResources();
    delete eTree;
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    // std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
