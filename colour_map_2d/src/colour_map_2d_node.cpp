#include "colour_map_2d_node.h"

using namespace ColourMap2D;

ColourMap2DNode::ColourMap2DNode(ros::NodeHandle nh)
    : nh_(nh), it_(nh), target_frame_("map"), tf_listener_(tf_buffer_)
{
    double hit_prob;
    double miss_prob;
    double min_prob;
    double prob_thresh;

    int cell_occupied;
    int cell_obstacle;
    int cell_unknown;
    int cell_empty;
    int frame;

    double spawn_noise;
    int spawn_rate;
    int spawn_size;

    std::string og_topic;
    std::string point_topic;

    ros::NodeHandle pn("~");
    pn.param<double>("rate", map_image_rate_, 1.0);
    pn.param<double>("hit_prob", hit_prob, 0.6);
    pn.param<double>("miss_prob", miss_prob, 0.2);
    pn.param<double>("min_prob", min_prob, 0.7);
    pn.param<double>("prob_thresh", prob_thresh, 0.05);
    pn.param<int>("cell_occupied", cell_occupied, 100);
    pn.param<int>("cell_obstacle", cell_obstacle, 50);
    pn.param<int>("cell_unknown", cell_unknown, -1);
    pn.param<int>("cell_empty", cell_empty, 0);
    pn.param<int>("spawn_number", spawn_size, 10);
    pn.param<int>("spawn_rate", spawn_rate, 10);
    pn.param<int>("frame", frame, 100);
    pn.param<double>("spawn_noise", spawn_noise, 0.02);
    pn.param<std::string>("og_topic", og_topic, "/output_map");
    pn.param<std::string>("point_topic", point_topic, "/camera/depth_registered/points/filtered");

    og_sub_ = nh_.subscribe(og_topic, 1, &ColourMap2DNode::ogCallback, this);
    pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_topic, 2, &ColourMap2DNode::pcCallback, this);
    image_pub_ = it_.advertise("/colour_map_image", 1);

    grid_ = Grid(hit_prob, miss_prob, min_prob, prob_thresh, cell_occupied, cell_obstacle, cell_empty, cell_unknown,
                 spawn_rate, spawn_noise, spawn_size, frame);

    colour_map_image_.encoding = sensor_msgs::image_encodings::RGB8;
}

void ColourMap2DNode::ogCallback(const nav_msgs::OccupancyGrid og)
{
    if(!grid_.initialized())
    {
        grid_.initializeGrid(og.info);
        grid_.initializeMapImage(colour_map_image_.image);
    }
    grid_.processOGMap(og);
    ROS_INFO("OG Map Proccessed");
}

void ColourMap2DNode::pcCallback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
{
    if(grid_.initialized())
        {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg (*pCloud, *cloud);

        std::vector<ColourPoint> colour_points;
        for(auto point:cloud->points)
        {
            geometry_msgs::PointStamped point_in;
            point_in.point.x = point.x;
            point_in.point.y = point.y;
            point_in.point.z = point.z;
            point_in.header = pCloud->header;
            geometry_msgs::PointStamped point_out;

            bool exception = false;
            try
            {
                tf_buffer_.transform(point_in, point_out, target_frame_);
            }
            catch (tf2::TransformException &ex)
            {
                exception = true;

            }
            if(!exception)
            {
                colour_points.push_back(ColourPoint(point_out.point.x, point_out.point.y, point.r, point.g, point.b));
            }

        }
        grid_.processPoints(colour_points);

        ROS_INFO("Point Cloud Proccessed");
    }
}

void ColourMap2DNode::mapImageThread()
{
    ros::Rate rate_limiter(1);

    while(ros::ok())
    {
        if(grid_.initialized())
        {
            grid_.updateImage(colour_map_image_.image);
            ROS_INFO("Image Updated");
            image_pub_.publish(colour_map_image_.toImageMsg());
        }
        rate_limiter.sleep();
    }
}

int main(int argc, char **argv)
{
  chdir("/home/mitch");
  ros::init(argc, argv, "colour_map_2d_node");

  ros::NodeHandle nh;

  ColourMap2DNode colour_map_2d_node(nh);

  std::thread image_thread(&ColourMap2DNode::mapImageThread, std::ref(colour_map_2d_node));

  ros::spin();

  ros::shutdown();

  image_thread.join();

  return 0;
}

