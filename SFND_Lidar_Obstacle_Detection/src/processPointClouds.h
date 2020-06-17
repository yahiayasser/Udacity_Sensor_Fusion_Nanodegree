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
#include "render/box.h"
//#include "KDtree3D.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KDTree3D
{
	Node* root;

	KDTree3D(): root(NULL)
	{}

	void insert(Node **node, int depth, std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		if(*node == NULL)
        {
		    *node = new Node(point,id);
        }
		else
		{
			uint cd = depth%3;
			if(point[cd] < ((*node)->point[cd]))
				insert(&((*node) -> left), depth+1, point, id);
			else
				insert(&((*node) -> right), depth+1, point, id);
		}
	}

    void setInputCloud(std::vector<std::vector<float>> points)
    {
        const int SIZE = points.size();
         for(int i = 0; i < SIZE; i++)
         {
            insert(&root, 0, points[i], i);
         }
    }

	std::vector<int> searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol)
	{
		std::vector<int> ids;
        int cd = depth%3;
	
		if(node != nullptr)
		{
			if((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) && (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)))
			{
				float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2] - target[2], 2));
				if(distance <= distanceTol)
				{
					ids.push_back(node->id); 
				}
			}
			
			if(target[cd] - distanceTol < node->point[cd])
			{
				std::vector<int> leftIDs = searchHelper(target, node->left, depth + 1, distanceTol);
				ids.insert(ids.end(), leftIDs.begin(), leftIDs.end());
			}
			if(target[cd] + distanceTol > node->point[cd])
			{
				std::vector<int> rightIDs = searchHelper(target, node->right, depth + 1, distanceTol);
				ids.insert(ids.end(), rightIDs.begin(), rightIDs.end());
			}
		}
	
		return ids;
	}

	std::vector<int> search(std::vector<float> point, float distanceTol)
	{
		// The implementation for this function and searchHelper was tweaked by using the solution offered
		std::vector<int> ids;
		ids = searchHelper(point, root, 0, distanceTol);
		return ids;
	}
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlaneRANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<std::vector<int>> EuclideanClusteringHelper(const std::vector<std::vector<float>>& points, KDTree3D* tree, float distanceTol);
  
    void Proximity(int index, const std::vector<std::vector<float>>& points, std::vector<bool>& processed, std::vector<int>& cluster, KDTree3D* tree, float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */