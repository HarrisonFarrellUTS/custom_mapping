#include <sstream>
#include <iostream>
#include <string>
#include <cmath>
#include <math.h>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <condition_variable>
#include <vector>
#include <list>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "tf/transform_datatypes.h"
#include <ros/package.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

class CustomMapping{

public:


    bool initalRun = false;     //!A bool to make sure the occupancy grid grid is the first to run before the main loop  

    struct pixelPoint           //!A struct used for transfering data from the input occupancy grid into the open CV mat image
    {
        int x;
        int y;
        int point_value;
    };

    struct searchCell           //!A struct used for looking at pixelCell values and it's neighbours 
    {
        int x; 
        int y; 
    };

    struct Objects              //!A struct used to idenify obstacles
    {
        std::vector<searchCell> objects;
        double diagonal;
        bool obstacle;

        void compute_diagonal();    //!A function that is any to compute the digonal
    };

    ros::NodeHandle nh_;
    CustomMapping(ros::NodeHandle nh);
    ~CustomMapping();

    void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg); //!The callback for the occupancyGrid created by the gmapping

    void actionThread();        //! The main thread loop, everything takes place in here

    void publishImage();        //! The function that produces the output images 

    void neighboursCheck(int j, int k, cv::Mat erosion_dst);    //! The function that checks the neighbours of the search cell

     
private:
    
    image_transport::ImageTransport it_;	//!a nodeHandle for images
    image_transport::Publisher image_pub_;	//!used the same as a ros::publisher
    image_transport::Publisher image2_pub_; //!used the same as a ros::publisher
    ros::Publisher OccupancyGrid_pub_; 	//!used for the publishing of the occpancy grid 
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;

    image_transport::Subscriber sub3_;	//!used the same as a ros::subscriber
    cv_bridge::CvImagePtr cvPtr_;

    float map_resolution_ = 0;//! size of OgMap in m/cells
    int map_height_ = 0; //height of the map in number of cells
    int map_width_ = 0;  //width of the map in number of cells
    nav_msgs::MapMetaData map_info_;


    std::vector<pixelPoint> point_values_;
    std::vector<searchCell> object;
    std::vector<searchCell> temp; 
    std::vector<std::vector<searchCell>> objects; 
    std::vector<bool> imageCellCheck;

    std::mutex mtx;
    std::condition_variable cv;
};









