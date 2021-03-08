/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <pcl/common/common.h>
#include <iostream>


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if (*node==NULL)
		{
			*node = new Node(point, id);
		}
		else 
		{
			uint cd = depth % 3;
          	//std::cout << "cd currently equals = " << cd << std::endl;
          	bool left = false;
          	if (cd == 0)
            {
              	//std::cout << "It made it to " << cd << std::endl;
            	if(point.x < ((*node)->point.x))
                {
                    left = true;
                }
            }
            if (cd == 1)
            {
              //std::cout << "It made it to " << cd << std::endl;
            	if(point.y < ((*node)->point.y))
                {
                    left = true;
                }
            }
            if (cd == 2)
            {
              //std::cout << "It made it to " << cd << std::endl;
            	if(point.z < ((*node)->point.z))
                {
                    left = true;
                }
            }
            if (left)		
            {
              insertHelper(&((*node)->left), depth+1, point, id);
            }
            else 
            {
              insertHelper(&((*node)->right), depth+1, point, id);
            }  
		}
	}

  	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	} 
  
  
	void searchHelper(pcl::PointXYZI target, Node* node, uint depth, float distanceTol, std::vector<int> &ids)
	{
		if (node!=NULL)
		{
			std::vector<float> deltas;
			deltas.push_back(node->point.x - target.x);
            deltas.push_back(node->point.y - target.y);
            deltas.push_back(node->point.z - target.z);

			bool distanceDeltaOk = true;

			for(float d : deltas)
			{
				if (fabs(d) >= distanceTol)
				{
					distanceDeltaOk = false;
					break;
				}
			}

			if (distanceDeltaOk) 
			{
				float distance = 0.0f;
				for (float d : deltas)
				{
					distance = distance + (d * d);
				}
				distance = sqrt(distance);

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			
			uint cd = depth % 3;
          	bool left = false;
          	if (cd == 0)
            {
                if ((target.x - distanceTol) < node->point.x)
                {
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                }
                if ((target.x + distanceTol) > node->point.x)
                {
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
                }
            }
            if (cd == 1)
            {
                if ((target.y - distanceTol) < node->point.y)
                {
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                }
                if ((target.y + distanceTol) > node->point.y)
                {
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
                }
            }
            if (cd == 2)
            {
                if ((target.z - distanceTol) < node->point.z)
                {
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                }
                if ((target.z + distanceTol) > node->point.z)
                {
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
                }
            }
          	//else 
            //{
            	//std::cerr << "Search Helper An error has occurred with cd not equaling 0,1,2: it currently equals = " << cd << std::endl;
            //}
		}
	}

  
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}

};




