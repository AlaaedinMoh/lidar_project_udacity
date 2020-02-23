
#include "./render/render.h"
#include <vector>

using namespace std;

template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT setPoint, int setId)
	:	point(setPoint), id(setId), left(NULL), right(NULL)
	{
	}
};

template<typename PointT>
struct EuclideanKdTree
{
	Node<PointT>* root;

	EuclideanKdTree()
	:root(NULL)
	{
	}
	int maxDepth = 0;

	void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node<PointT>(point, id);
		}
		else
		{
			uint cd = depth % 3;
			if(cd == 0)
			{
				if(point.x < (*node)->point.x)
				{
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
			if(cd == 1)
			{
				if(point.y < (*node)->point.y)
				{
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
			if(cd == 2)
			{
				if(point.z < (*node)->point.z)
				{
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
		}
	}

	void insert(PointT point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(PointT target, Node<PointT>* node, uint depth, float dist_tol, vector<int>& ids)
	{
		if(node != NULL)
		{
			if(node->point.x >= (target.x - dist_tol) && node->point.x <= (target.x + dist_tol)
			&& node->point.y >= (target.y - dist_tol) && node->point.y <= (target.y + dist_tol)
			&& node->point.z >= (target.z - dist_tol) && node->point.z <= (target.z + dist_tol))
			{
				float dx = node->point.x - target.x;
				float dy = node->point.y - target.y;
				float dz = node->point.z - target.z;
				float dist = sqrt(dx*dx + dy*dy + dz*dz);
				if(dist <= dist_tol)
				{
					ids.push_back(node->id);
				}
			}
			uint cd = depth%3;
			if(cd == 0)
			{
				if((target.x - dist_tol) < node->point.x)
				{
					searchHelper(target,node->left, depth+1, dist_tol, ids);
				}
				if((target.x + dist_tol) > node->point.x)
				{
					searchHelper(target, node->right, depth+1, dist_tol, ids);
				}
			}
			if(cd == 1)
			{
				if((target.y - dist_tol) < node->point.y)
				{
					searchHelper(target,node->left, depth+1, dist_tol, ids);
				}
				if((target.y + dist_tol) > node->point.y)
				{
					searchHelper(target, node->right, depth+1, dist_tol, ids);
				}
			}
			if(cd == 2)
			{
				if((target.z - dist_tol) < node->point.z)
				{
					searchHelper(target,node->left, depth+1, dist_tol, ids);
				}
				if((target.z + dist_tol) > node->point.z)
				{
					searchHelper(target, node->right, depth+1, dist_tol, ids);
				}
			}
		}
	}

	vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	void ResourcesFreeHelper(Node<PointT>** node)
	{
		if(*node)
		{
			return;
		}
		if(!(*node)->right)
		{
			ResourcesFreeHelper(&((*node)->right));
		}
		if(!(*node)->left)
		{
			ResourcesFreeHelper(&((*node)->left));
		}
		delete *node;
	}

	void freeResources()
	{
		ResourcesFreeHelper(&root);
	}
};