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

    bool initalRun = false;

    struct pixelPoint
    {
        int x;
        int y;
        int point_value;
    };

    struct searchCell
    {
        int x; 
        int y; 
    };

    ros::NodeHandle nh_;
    CustomMapping(ros::NodeHandle nh);
    ~CustomMapping();

    void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg);

    void actionThread(); 

    void publishImage();

    void neighboursCheck(int j, int k, cv::Mat erosion_dst);
     
private:
    
    image_transport::ImageTransport it_;	//!a nodeHandle for images
    image_transport::Publisher image_pub_;	//!used the same as a ros::publisher
    image_transport::Publisher image2_pub_; //!used the same as a ros::publisher
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;

    image_transport::Subscriber sub3_;	//!used the same as a ros::subscriber
    cv_bridge::CvImagePtr cvPtr_;

    float map_resolution_ = 0;//! size of OgMap in m/cells
    int map_height_ = 0; //height of the map in number of cells
    int map_width_ = 0;  //width of the map in number of cells

    std::vector<pixelPoint> point_values_;
    std::vector<searchCell> object;
    std::vector<searchCell> temp; 
    std::vector<std::vector<searchCell>> objects; 
    std::vector<bool> imageCellCheck;

    std::mutex mtx;
    std::condition_variable cv;
};









