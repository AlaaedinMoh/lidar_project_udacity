#pragma once

#include "./render/render.h"

using namespace std;

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

struct EuclideanKdTree
{
	Node* root;

	EuclideanKdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth % 2;
			if(point[cd] < (*node)->point[cd])
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(vector<float> target, Node* node, uint depth, float dist_tol, vector<int>& ids)
	{
		if(node != NULL)
		{
			if(node->point[0] >= (target[0] - dist_tol) && node->point[0] <= (target[0] + dist_tol)
			&& node->point[1] >= (target[1] - dist_tol) && node->point[1] <= (target[0] + dist_tol))
			{
				float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0])
								    + (node->point[1] - target[1])*(node->point[1] - target[1]));
				if(distance <= dist_tol)
				{
					ids.push_back(node->id);
				}
			}
			uint cd = depth%2;
			if((target[cd]-dist_tol) < node->point[cd])
			{
				searchHelper(target, node->left, depth+1, dist_tol, ids);
			}
			if((target[cd]+dist_tol)>node->point[cd])
			{
				searchHelper(target, node->right, depth+1, dist_tol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};




