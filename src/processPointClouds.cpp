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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering#
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

    // vector<int> indices;
    // CropBox<PointT> roof;
    // roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    // roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    // roof.setInputCloud(cloudRegion);
    // roof.filter(indices);

    // PointIndices::Ptr inliers (new PointIndices ());
    // for(int point : indices)
    // {
    //     inliers->indices.push_back(point);
    // }

    // ExtractIndices<PointT> extract;
    // extract.setInputCloud(cloudRegion);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}

template <typename PointT> unordered_set<int> 
DoRasnac3D(typename PointCloud<PointT>::Ptr cloud, int maxIteration, float distanceTol)
{
    unordered_set<int> inliersRslt;
    ///////////////////////////////////////////////
    srand(time(NULL));
    for(int i = 0; i < maxIteration; i++)
    {
        unordered_set<int> inliers;
        while(inliers.size()<3)
        {
            inliers.insert(rand()%cloud->points.size());
        }
        float point_1[3];
        float point_2[3];
        float point_3[3];
        auto iter = inliers.begin();
        point_1[0]=cloud->points[*iter].x;
        point_1[1]=cloud->points[*iter].y;
        point_1[2]=cloud->points[*iter].z;
        iter++;
        point_2[0]=cloud->points[*iter].x;
        point_2[1]=cloud->points[*iter].y;
        point_2[2]=cloud->points[*iter].z;
        iter++;
        point_3[0]=cloud->points[*iter].x;
        point_3[1]=cloud->points[*iter].y;
        point_3[2]=cloud->points[*iter].z;

        cout<<"Point 1: X= "<<point_1[0]<<"\t,Y="<<point_1[1]<<"\t,Z="<<point_1[2]<<endl;
        cout<<"Point 2: X= "<<point_2[0]<<"\t,Y="<<point_2[1]<<"\t,Z="<<point_2[2]<<endl;
        cout<<"Point 3: X= "<<point_3[0]<<"\t,Y="<<point_3[1]<<"\t,Z="<<point_3[2]<<endl;

            //<x_value, y_value>
        float vect_1[] = {point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]};
        float vect_2[] = {point_2[0] - point_1[0], point_3[1] - point_1[1], point_3[2] - point_1[2]};

        cout<<"Vector 1: i= "<<vect_1[0]<<"\t,j="<<vect_1[1]<<endl;
        cout<<"Vector 2: i= "<<vect_2[0]<<"\t,j="<<vect_2[1]<<endl;

        float a = vect_1[1]*vect_2[2] - vect_1[2]*vect_1[1];
        float b = vect_1[2]*vect_2[1] - vect_1[0]*vect_2[2];
        float c = vect_1[0]*vect_2[1] - vect_1[1]*vect_2[0];
        

        float plane_D = -1*(point_1[0]*a + point_1[1]*b + point_1[2]*c);
        cout<<"A= "<<a<<"\t,B= "<<b<<"\t,c= "<<c<<"\t,D= "<<plane_D<<endl;
        float plane[] = {a, b, c, plane_D};
        for(int idx = 0; idx < cloud->points.size(); idx++)
        {
            if(inliers.size() > 0)
            {
                continue;
            }
            PointT point = cloud->points[idx];
            float distance = fabs(point.x*plane[0] + point.y*plane[1] + point.z*plane[2] + plane[3] / sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]));
            if(distance <= distanceTol)
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
    return inliersRslt;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
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
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // // TODO:: Fill in this function to find inliers for the cloud.
    // SACSegmentation<PointT> seg;
    // ModelCoefficients::Ptr coefficients{new ModelCoefficients};

    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    // if(inliers->indices.size() == 0)
    // {
    //     cout<<"couldn't fine any planar";
    // }

    // auto endTime = std::chrono::steady_clock::now();
    // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    // std::unordered_set<int> inliers = DoRasnac3D(cloud, maxIterations, distanceThreshold);
    std::unordered_set<int> inliersRslt;
    srand(time(NULL));
    for(int i = 0; i < maxIterations; i++)
    {
        unordered_set<int> inliers;
        while(inliers.size()<3)
        {
            inliers.insert(rand()%cloud->points.size());
        }
        float point_1[3];
        float point_2[3];
        float point_3[3];
        auto iter = inliers.begin();
        point_1[0]=cloud->points[*iter].x;
        point_1[1]=cloud->points[*iter].y;
        point_1[2]=cloud->points[*iter].z;
        iter++;
        point_2[0]=cloud->points[*iter].x;
        point_2[1]=cloud->points[*iter].y;
        point_2[2]=cloud->points[*iter].z;
        iter++;
        point_3[0]=cloud->points[*iter].x;
        point_3[1]=cloud->points[*iter].y;
        point_3[2]=cloud->points[*iter].z;

        cout<<"Point 1: X= "<<point_1[0]<<"\t,Y="<<point_1[1]<<"\t,Z="<<point_1[2]<<endl;
        cout<<"Point 2: X= "<<point_2[0]<<"\t,Y="<<point_2[1]<<"\t,Z="<<point_2[2]<<endl;
        cout<<"Point 3: X= "<<point_3[0]<<"\t,Y="<<point_3[1]<<"\t,Z="<<point_3[2]<<endl;

            //<x_value, y_value>
        float vect_1[] = {point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]};
        float vect_2[] = {point_3[0] - point_1[0], point_3[1] - point_1[1], point_3[2] - point_1[2]};

        cout<<"Vector 1: i= "<<vect_1[0]<<"\t,j="<<vect_1[1]<<endl;
        cout<<"Vector 2: i= "<<vect_2[0]<<"\t,j="<<vect_2[1]<<endl;

        float a = vect_1[1]*vect_2[2] - vect_1[2]*vect_2[1];
        float b = vect_1[2]*vect_2[0] - vect_1[0]*vect_2[2];
        float c = vect_1[0]*vect_2[1] - vect_1[1]*vect_2[0];
        

        float plane_D = -1*(point_1[0]*a + point_1[1]*b + point_1[2]*c);
        cout<<"A= "<<a<<"\t,B= "<<b<<"\t,c= "<<c<<"\t,D= "<<plane_D<<endl;
        float plane[] = {a, b, c, plane_D};     
        for(int idx = 0; idx < cloud->points.size(); idx++)
        {
            if(inliers.count(idx) > 0)
            {
                continue;
            }
            PointT point = cloud->points[idx];
            float distance = fabs(point.x*plane[0] + point.y*plane[1] + point.z*plane[2] + plane[3] / sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]));
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

    cout<<"Inliers Result count= "<<inliersRslt.size()<<endl;

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
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

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

template<typename PointT> pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr>
 CustomSegmentPlane(typename PointCloud<PointT>::Ptr cloud, int maxIteration, float distThreshold)
{
    // std::unordered_set<int> inliers = DoRansac3D(cloud, maxIteration, distThreshold);
    typename PointCloud<PointT>::ptr inliersCld(new PointCloud<PointT>);
    typename PointCloud<PointT>::ptr obstaclesCld(new PointCloud<PointT>);
    // for(int idx = 0; idx < cloud->points.size(); idx++)
    // {
    //     // PointT point = cloud->points[idx];
    //     // if(inliers.count(index))
    //     //     inliersCld->points.push_back(point);
    //     // else
    //     //     obstaclesCld->points.push_back(point);
    // }
    pair<typename PointCloud<PointT>::Ptr, typename PointCloud<PointT>::Ptr> segRslt;
    // segRslt.first = inliersCld;
    // segRslt.second = obstaclesCld;
    return segRslt;
}

