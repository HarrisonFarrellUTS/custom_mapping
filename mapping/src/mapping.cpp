#include "mapping.h" 

#define requiredHz 1
#define WHITE 255
#define GREY 150

CustomMapping::CustomMapping(ros::NodeHandle nh)    //The constructor of the customMapping class
    : nh_(nh) , it_(nh)
{
    sub2_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &CustomMapping::occupancyGridCallback,this);

    image_pub_ = it_.advertise("/image_test", 10);

    image2_pub_ = it_.advertise("/processed_image", 10);

    OccupancyGrid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/output_map", 10);
}

CustomMapping::~CustomMapping()     //the Deconstructor of the customMapping class
{}

void CustomMapping::actionThread()  //The main loop
{
    while(!initalRun){};            //the initial occupancy grid bool check 

    ros::Rate loop_Rate(requiredHz);
    bool loopSet = 0;
    while (ros::ok())
     {
        loopSet = 1;
        if(loopSet)
        {
            publishImage();

            loop_Rate.sleep();
        }
    }
}

void CustomMapping::occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg)   //The occupancy grid call back from the gmapping
{
    std::unique_lock<std::mutex> lck(mtx);
    point_values_.clear();
    int i = 0; 
    map_info_ = msg->info;
    map_resolution_ = msg->info.resolution;
    map_height_ = msg->info.height;
    map_width_ = msg->info.width;
    ROS_INFO("INPUT_OG Data: height %d, width %d, resolution %f, size %d", msg->info.height, msg->info.width, msg->info.resolution, msg->data.size());

    for(int j = 0; j < map_height_; j++ )       //converts the 1D array of the occupancy grid into a vector of cells, [x].[y] [point_values]
    {
        for(int k =0; k < map_width_; k++ )
        {
            pixelPoint point;
            point.x = j;
            point.y = k;
            point.point_value = msg->data[i];
            point_values_.push_back(point);
            i ++ ;
        }
    }

    initalRun = true; 
    cv.notify_one();
}

void CustomMapping::publishImage()
{
    //initialisation all the images used
    cv::Mat inputImage(map_height_, map_width_, CV_8U);
    cv::Mat outputImage(map_height_, map_width_, CV_8UC3);
    cv::Mat erosion_dst(map_height_, map_width_, CV_8U);
    cv::Mat dilation_dst(map_height_, map_width_, CV_8U);
    cv::Mat greyMat(map_height_, map_width_, CV_8U);

    //initialisation of variables
    int largest_object = 0; 
    int largest_object_2 = 0; 
    int largest_object_3 = 0; 
    int largest_object_4 = 0; 
    int holding_value = 0; 

    cv::Vec3b white3C(255,255,255),
        grey3C(150, 150, 150),
    	black3C(0,0,0),
        red(30, 100, 200),
        green (200, 100, 40);

    uint8_t white = 255;
    uint8_t black = 0;

    //A simple loop for converting the vector of points into a cv::Mat image
    for(int j = 0; j < map_height_; j++ )
    {
        for(int k =0; k < map_width_; k++ )
        {
            imageCellCheck.push_back(false);    //sets the size of the imageCellCheck to the size of the image
            switch (point_values_[ (j * map_height_ ) +k ].point_value)
            {
            case 0:
                inputImage.at<uint8_t>(k , j) = WHITE; //known clear space
                outputImage.at<cv::Vec3b>(k , j) = white3C; 
                break;
            case 100:
                inputImage.at<uint8_t>(k , j) = black; //object
                outputImage.at<cv::Vec3b>(k , j) = black3C;
                break;
            case -1:
                inputImage.at<uint8_t>(k , j) = WHITE; //unknown, unclear space
                outputImage.at<cv::Vec3b>(k , j) = grey3C;
                break;
            default:
                inputImage.at<uint8_t>(k , j) = WHITE;
                outputImage.at<cv::Vec3b>(k , j) = white3C;
                break;
            }
        }
    }

    //using openCV to erode the input image to reduce the noise the 2D lidar creates 
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
	                       cv::Size(1,1),
	                       cv::Point(-1,-1));
	cv::erode(inputImage, erosion_dst, element);    


    //A loop to search the input image for all known Cells
    for(int j = 0; j < map_height_; j++)
    {
        for(int k = 0; k < map_width_; k++)
        {
            if(imageCellCheck.at( (j * map_height_ ) + k) == false)
            {
                imageCellCheck.at((j * map_height_ ) + k) = true;   //sets the imageCellCheck to true once searched 
                if(erosion_dst.at<uint8_t>(k, j) != WHITE)
                {
                	object.clear();    
                	temp.clear(); 
                    neighboursCheck(j,k, erosion_dst);
                }
            }
        }
    }

    //A set loops to find the diagonal of all the objects and determine the largest 
    std::vector<CustomMapping::Objects> diag_objects;
    for(auto object:objects)
    {
        CustomMapping::Objects new_object;
        new_object.objects = object;
        new_object.compute_diagonal();
        diag_objects.push_back(new_object);
    }
    std::sort(diag_objects.begin(), diag_objects.end(), [](const CustomMapping::Objects & a, const CustomMapping::Objects & b){ return a.diagonal > b.diagonal; });

	for(int a = 0; a < diag_objects.size(); a++)
	{
		if(diag_objects[a].diagonal > 0.4 * diag_objects[0].diagonal) diag_objects[a].obstacle = false;
        else diag_objects[a].obstacle = true;
		for(auto object:diag_objects[a].objects)
		{
			if(diag_objects[a].obstacle) outputImage.at<cv::Vec3b>( object.y, object.x ) = red;
			else outputImage.at<cv::Vec3b>( object.y, object.x ) = green;
		}
	}

    //turns the outimage into a 1D occupancy grid to be published 
    std::vector<signed char> outputVector;
    for (int i = 0; i< outputImage.cols; i++)
    {
        for (int j = 0; j < outputImage.rows; j++)
        {
			if(outputImage.at<cv::Vec3b>(j, i) == green)
			{
                //outputArray[ (i * map_height_ ) + j] = 100; 	//wall is value of 100
                outputVector.push_back(100);
			}
			else if(outputImage.at<cv::Vec3b>(j, i) == red)
			{
                //outputArray[ (i * map_height_ ) + j] = 50; 	//object is value of 50
                outputVector.push_back(50);
			}
			else if(outputImage.at<cv::Vec3b>(j, i) == white3C)
			{
                //outputArray[ (i * map_height_ ) + j] = 0; 	//clear space is 0
                outputVector.push_back(0);
			}
            else if(outputImage.at<cv::Vec3b>(j, i) == grey3C)
            {
                //outputArray[ (i * map_height_ ) + j] = -1; 	//clear space is 0
                outputVector.push_back(-1);
            }
			else
			{
                //outputArray[ (i * map_height_ ) + j] = -1; //anything else is -1
			}
        }
	}

    //Publishing all the images created along an 1D occupancy grid 
	nav_msgs::OccupancyGrid msg3;
    msg3.info = map_info_;
	msg3.data = outputVector; 	 

	OccupancyGrid_pub_.publish(msg3);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", erosion_dst).toImageMsg();
    image_pub_.publish(msg);

    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
    image2_pub_.publish(msg2);

    //Clears all the vectors at the end of the loop 
    imageCellCheck.clear(); 
    objects.clear();
    object.clear();;
}


void CustomMapping::neighboursCheck(int j, int k, cv::Mat erosion_dst)
{
    searchCell cell_initial; 
    cell_initial.x = j; 
    cell_initial.y = k; 
    object.push_back(cell_initial);
    temp.push_back(cell_initial);

    //the Searching of a cell known to be a known cell and it's 8 surrounding neighbours
    //if a neighbour is also known, it's added to a vector and has it's neighbours also checked. 
    while(!temp.empty())
    {
        int j_search_ = temp.back().x;
        int k_search_ = temp.back().y;

        temp.pop_back();

        int col_upper_limit = 2; 
        int col_lower_limit = -1; 
        int row_upper_limit = 2; 
        int row_lower_limit = -1; 

        if(j_search_ == 0 )	{ row_lower_limit = 0;}

        if(j_search_ == map_height_ -1)	{ row_upper_limit = 1;}

        if(k_search_ == 0 )	{ col_upper_limit = 0;}

        if(k_search_ == map_width_ -1){	col_upper_limit = 1;}

        for(int m = row_lower_limit; m < row_upper_limit; m++)
        {
        	for(int n = col_lower_limit; n < col_upper_limit; n++)
        	{
        		if(imageCellCheck.at( ((j_search_ + n)  * map_height_ ) + (k_search_ + m)) == false)
        		{
        			imageCellCheck.at( ((j_search_ + n) * map_height_ )  + (k_search_ + m)) = true;

	        		if(erosion_dst.at<uint8_t>(k_search_ + m , j_search_ + n) != WHITE)
	        		{
	        			searchCell cell_neighbour; 
	        			cell_neighbour.x = j_search_ + n; 
	        			cell_neighbour.y = k_search_ + m; 
	        			object.push_back(cell_neighbour);
	        			temp.push_back(cell_neighbour);
	        		}
        		}
        	}
        }
    }
    objects.push_back(object); 
}

//function for computing the diagonals for the objects
void CustomMapping::Objects::compute_diagonal()
{
    int min_x = 100000;
    int min_y = 100000;
    int max_x = 0;
    int max_y = 0;

    for(auto object:objects)
    {
        if(object.x < min_x) min_x = object.x;
        if(object.y < min_y) min_y = object.y;
        if(object.x > max_x) max_x = object.x;
        if(object.y > max_y) max_y = object.y;
    }

    diagonal = sqrt(pow(static_cast<double>(max_x) -  static_cast<double>(min_x),2) + pow(static_cast<double>(max_y) -  static_cast<double>(min_y) ,2));
}
