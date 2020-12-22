// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    while(maxIterations--){
        std::unordered_set<int> inliers;
        
        //Sample 3 points 
        while(inliers.size() < 3){
            inliers.insert(rand() % cloud->points.size());   
        }
        
        auto itr = inliers.begin();
        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

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
        
        std::vector<float> v1, v2;

        v1.push_back(x2- x1);
        v1.push_back(y2 - y1);
        v1.push_back(z2 - z1);

        v2.push_back(x3 - x1);
        v2.push_back(y3 - y1);
        v2.push_back(z3 - z1);
        
        std::vector<float> v1_cross_v2;
        
        v1_cross_v2.push_back(v1[2]*v2[3] - v1[2] * v2[1]);
        v1_cross_v2.push_back(v1[2]*v2[1] - v1[0] * v2[2]);
        v1_cross_v2.push_back(v1[0]*v2[1] - v1[1]*v2[0]);

        float A = v1_cross_v2[0];
        float B = v1_cross_v2[1];
        float C = v1_cross_v2[2];
        float D = A * x1 + B * y1 + C * z1;

        //Compute inliers 
        for(int i = 0; i < cloud->points.size(); i++){
            if(inliers.count(i) > 0 ) continue;
           
            pcl::PointXYZ points = cloud->points[i]; 
            float x4(points.x), y4(points.y), z4(points.z);

            float dst = abs( A*x4 + B * y4 + C * z4 + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
            
            if(dst <= distanceTol){
                inliers.insert(i);   
            }
        
        }
        if(inliers.size() > inliersResult.size()){
            inliersResult = inliers;  
        }
    }
    return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
    //Fitting a Line to 2d points
	// For max iterations 
    while(maxIterations--){
        
        // Randomly sample subset and fit line
        std::unordered_set<int> inlier;

        while(inlier.size() < 2){
            inlier.insert(rand() % (cloud->points.size()));
        }

        float x1, y1, x2, y2;

        auto itr = inlier.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // Measure distance between every point and fitted line
        //find the coeffs
        float A = y1 - y2;
        float B = x2 - x1;
        float C = (x1 * y2 - x2* y1);
        
        for (int i = 0; i < cloud->points.size(); i++){
            if(inlier.count(i)>0)
                 continue;
 
            pcl::PointXYZ point = cloud->points[i];            
            float x3(point.x), y3(point.y);
            float dst = abs(A * x3 + B * y3 + C) / sqrt(pow(A, 2)  + pow(B, 2));
            
            // If distance is smaller than threshold count it as inlier
            if(dst <= distanceTol){
                inlier.insert(i);
            }
        }    

        if(inlier.size() > inliersResult.size()){
            inliersResult = inlier;
        }     
        // Return indicies of inliers from fitted line with most inliers
	}

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    
    //Line below runs ransac on 2D data
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.);
    
    std::unordered_set<int> inliers = Ransac3D(cloud, 10, 1.);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
