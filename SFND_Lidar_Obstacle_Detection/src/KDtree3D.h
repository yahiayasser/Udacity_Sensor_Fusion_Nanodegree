#ifndef KDTREE3D_H_
#define KDTREE3D_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <string>

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


template<typename PointT>
struct KDTree3D
{
	Node* root;

	KDTree3D(): root(NULL)
	{}

    void setInputCloud (typename pcl::PointCloud<PointT>::Ptr cloud)
    {
         for(int i = 0; i < cloud->points.size(); i++)
            insert(&root, 0, cloud->points[i], i);
    }

	void insert(Node **node, uint depth, PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		if(*node == NULL)
        {
            std::vector<float> v_point(point.data, point.data+3);
		    node = new Node(v_point,id);
        }
		else
		{
			uint cd = depth%3;
			if(point.data[cd] < ((*node)->point[cd]))
				insert(&((*node) -> left), depth+1, point, id);
			else
				insert(&((*node) -> right), depth+1, point, id);
		}
	}

    std::vector<int> search(PointT point, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(point, root, 0, distanceTol, ids);
		return ids;
	}

	void searchHelper(PointT target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{   
		if(node != NULL)
		{
			if((node->point[0] >= (target[0].data - distanceTol) && node->point[0] <= (target.data[0] + distanceTol)) && (node->point[1] >= (target.data[1] - distanceTol) && node->point[1] <= (target[1].data + distanceTol)) && (node->point[2] >= (target.data[2] - distanceTol) && node->point[2] <= (target[2].data + distanceTol)))
			{
				float distance = sqrt(((node->point[0] - target.data[0]) * (node->point[0] - target.data[0])) + ((node->point[1] - target.data[1]) * (node->point[1] - target.data[1])) + ((node->point[2] - target.data[2]) * (node->point[2] - target.data[2])));
				if(distance <= distanceTol);
					ids.push_back(node -> id);
			}
			if((target.data[depth%3] - distanceTol) < node->point[depth%3])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target.data[depth%3] + distanceTol) > node->point[depth%3])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}
};

#endif // !KDTREE3D_H_