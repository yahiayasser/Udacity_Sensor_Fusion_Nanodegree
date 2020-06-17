// PCL lib Functions for processing point clouds 

#include <unordered_set>
#include "processPointClouds.h"
//#include "KDtree3D.h"

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

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);

    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr CloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud (filteredCloud);
    region.filter (*CloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud (CloudRegion);
    roof.filter (indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);
        
    pcl::ExtractIndices<PointT> extract;     

    // Extract the inliers
    extract.setInputCloud (CloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*CloudRegion);    


    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return CloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
     typename pcl::PointCloud<PointT>::Ptr Obstacles_Cloud (new pcl::PointCloud<PointT> ());
     typename pcl::PointCloud<PointT>::Ptr Plane_Cloud (new pcl::PointCloud<PointT> ());

     for (int index : inliers -> indices)
        Plane_Cloud -> points.push_back(cloud->points[index]);
        
     pcl::ExtractIndices<PointT> extract;     

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*Obstacles_Cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(Obstacles_Cloud, Plane_Cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // maxIterations is the number of iterations
    // distanceThreshold is the separation distance between objects

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if(inliers -> indices.size() == 0)
    {
        std::cout << "Could not estimate a planner model for the given dataset"<< std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::segmentPlaneRANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    typename pcl::PointCloud<PointT>::Ptr Plane(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr Obstacles(new pcl::PointCloud<PointT>);
	std::unordered_set<int> inliersResult;
    int cloudSize = cloud->points.size();

	srand(time(NULL));

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, y1, x2, y2, x3, y3, z1, z2, z3;

		// The three random points that form the plane
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Plane Linear equation
		// ax + by + cz + d = 0
		float a = ((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1));
		float d= ((a*x1)+(b*y1)+(c*z1)) * -1;

		for (int  index = 0; index < cloudSize; index++)
		{
			/* code */
			if(inliers.count(index) > 0)
				continue;

            if(((cloudSize - index) + inliers.size()) < (inliersResult.size()))
                break;

            //typename pcl::PointT point = cloud->points[index];
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float Distance = fabs((a*x4) + (b*y4) + (c*z4) + d)/sqrt(a*a + b*b + c*c);

			if(Distance <= distanceThreshold)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
            inliersResult = inliers;
	}

    for(int i = 0; i < cloudSize; i++)
    {

		PointT point = cloud->points[i];
		if(inliersResult.count(i) > 0)
			Plane->points.push_back(point);
		else
			Obstacles->points.push_back(point);
    }
	
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(Obstacles, Plane);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster -> points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
    {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<std::vector<int>> clusterResults;
    std::vector<std::vector<float>> CloudVector;
    KDTree3D* tree = new KDTree3D;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    for(int i = 0; i < cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        CloudVector.push_back({point.x, point.y, point.z});
    }

    tree->setInputCloud(CloudVector);

    clusterResults = EuclideanClusteringHelper(CloudVector, tree, clusterTolerance);

    for(auto indexSet : clusterResults)
    {
        if(indexSet.size() < minSize || indexSet.size() > maxSize)
        {
            continue; 
        }

        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        
        for(auto index : indexSet)
        {
            cluster->points.push_back(cloud->points[index]);
        }
    
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
    
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanClusteringHelper(const std::vector<std::vector<float>>& points, KDTree3D* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
    const int SIZE = points.size();
	std::vector<bool> processed(SIZE, false);

	for(int i = 0; i < SIZE; i++)
	{
		if(!processed[i])
		{
			std::vector<int> cluster;
			Proximity(i, points, processed, cluster, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int index, const std::vector<std::vector<float>>& points, std::vector<bool>& processed, std::vector<int>& cluster, KDTree3D* tree, float distanceTol)
{
 	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> ClusterMembers = tree->search(points[index], distanceTol);

	for(int count : ClusterMembers)
	{
		if(!processed[count])
		{
			Proximity(count, points, processed, cluster, tree, distanceTol); 
		}
	}
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