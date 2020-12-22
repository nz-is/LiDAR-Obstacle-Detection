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

    void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
        if(*node == NULL){
            *node = new Node(point, id);
        }
        else{ 
            uint compare_ax = depth % 2;
        
            if(point[compare_ax] < (*node)->point[compare_ax])
               insertHelper(&((*node)->left), depth+1, point, id);
            else
               insertHelper(&((*node)->right), depth+1, point, id);       
        }
    }

	void insert(std::vector<float> point, int id)
	{
	    insertHelper(&root, 0, point, id);
    }
    
    
    void searchHelper(Node* node, std::vector<float> target, uint depth, float distanceTol, std::vector<int>& ids)
    {
     if(node != NULL)
     {
        //Check if target node point is within the target+distanceTol box
        if ( (node->point[0] <= target[0]+distanceTol) && (node->point[0] >= target[0]-distanceTol) && 
              (node->point[1] <= target[1]+distanceTol) && (node->point[1] >= target[1]-distanceTol)) 
        {
            //Compute the Euclidean distance 
            float a = node->point[0] - target[0];
            float b = node->point[1] - target[1];
            float dst = sqrt(pow(a, 2) + pow(b, 2));
            
            if(dst <= distanceTol)
                ids.push_back(node->id);
        }
        
        uint ind = depth % 2;
        //check whether to traverse left or right
        //Traverse Right when the curr_node is above the distanceTol box else left
        if( (target[ind]-distanceTol) < node->point[ind])
            searchHelper(node->left, target, depth+1, distanceTol, ids);
        if( (target[ind]+distanceTol) > node->point[ind])
            searchHelper(node->right, target, depth+1, distanceTol, ids);
            
      }
    } 

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        
        searchHelper(root, target, 0, distanceTol, ids);
		return ids;
	}
};




