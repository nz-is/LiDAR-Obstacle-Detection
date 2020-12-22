// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, .1f, Eigen::Vector4f(-30, -5, -4, 1), Eigen::Vector4f( 30, 6, 10, 1));
    renderPointCloud(viewer, filterCloud, "inputCloud");

    //Segment Point clouds to obstacles & roads
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pointPairs = pointProcessorI ->SegmentPlane(filterCloud, 200, 0.2);
    
    renderPointCloud(viewer, pointPairs.first, "obsCloud", Color(1, 0, 0));
    renderPointCloud(viewer, pointPairs.second, "planeCloud", Color(0, 1, 0));
    
    //Perform Clustering on different ObsPnts 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(pointPairs.first, 0.25, 100, 10000);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 1), Color(0, 0, 1), Color(1, 1, 0)};
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster :clusters){
        renderPointCloud(viewer, cloud_cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cloud_cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//Loads 3d viewer and display  PCD data
void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor(new ProcessPointClouds<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, .1f, Eigen::Vector4f(-30, -5, -3, 1), Eigen::Vector4f( 30, 6, 10, 1));
    renderPointCloud(viewer, filterCloud, "inputCloud");

    //Segment Point clouds to obstacles & roads
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> pointPairs = pointProcessor ->SegmentPlane(filterCloud, 200, 0.2);
    
    renderPointCloud(viewer, pointPairs.first, "obsCloud", Color(1, 0, 0));
    renderPointCloud(viewer, pointPairs.second, "planeCloud", Color(0, 1, 0));
    
    //Perform Clustering on different ObsPnts 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor->Clustering(pointPairs.first, .5, 10, 10000);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 1), Color(0, 0, 1), Color(1, 1, 0)};
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster :clusters){
        renderPointCloud(viewer, cloud_cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = pointProcessor->BoundingBox(cloud_cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//This function renders a simulated 3d highway w Lidars 
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    //Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds = lidar->scan();
    //renderRays(viewer, lidar->position, pointClouds); 
    renderPointCloud(viewer, pointClouds, "viewer", Color(255, 255, 255));

    //Process point clouds 
    //init to heap (ptr) -> Dynamically allocates to memeory but needs to be freed after
    ProcessPointClouds<pcl::PointXYZ>*  pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //init to stack 
    //ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pointPairs = pointProcessor -> SegmentPlane(pointClouds, 200, 0.1);
    
    //renderPointCloud(viewer, pointPairs.first, "ObstCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, pointPairs.second, "PlaneCloud", Color(0, 1, 0));

    //Perform Clustering on Obstacles points to further segment objects
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(pointPairs.first, 2.5, 3, 1000);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(0.5, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster : clusters){
        std::cout << "Cluster size: ";
        pointProcessor->numPoints(cloud_cluster);
        renderPointCloud(viewer, cloud_cluster, "obsCloud"+std::to_string(clusterId), colors[clusterId]);
        //Encapsulate point clouds into a bounding box
        Box box = pointProcessor->BoundingBox(cloud_cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);
    
    //Line below renders a simulated highway of point cloud data 
    //simpleHighway(viewer);

    //Function below loads a single PCD file and runs thru all the obs. det pipeline 
    //CityBlock(viewer);

    //Stream PCD file 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
     
    while (!viewer->wasStopped ())
    {
        //Clear viewe
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load PCD and run obstacle detection pipeline 
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        CityBlock(viewer, pointProcessorI, inputCloudI);
        
        streamIterator++;
        if(streamIterator == stream.end()){
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}
