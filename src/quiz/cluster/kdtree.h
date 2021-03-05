/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}



	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node==NULL)
		{
			*node = new Node(point, id);
		}
		else 
		{
			uint cd = depth % point.size();
			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else 
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}



	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	} 

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node!=NULL)
		{
			std::vector<float> deltas;

			for (int i = 0; i < node->point.size(); i++)
			{
				deltas.push_back(node->point[i] - target[i]);
			}

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
			
			int currentIndx = depth % node->point.size();

			if ((target[currentIndx] - distanceTol) < node->point[currentIndx])
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if ((target[currentIndx] + distanceTol) > node->point[currentIndx])
			{
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};




