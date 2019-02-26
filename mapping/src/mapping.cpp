#include "mapping.h" 

#define requiredHz 1
#define WHITE 255

CustomMapping::CustomMapping(ros::NodeHandle nh)
    : nh_(nh) , it_(nh)

{
    sub2_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &CustomMapping::occupancyGridCallback,this);

    image_pub_ = it_.advertise("/image_test", 10);

    image2_pub_ = it_.advertise("/processed_image", 10);
}

CustomMapping::~CustomMapping()
{}

void CustomMapping::actionThread(){

    while(!initalRun){};

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

void CustomMapping::occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg){

    std::unique_lock<std::mutex> lck(mtx);
    point_values_.clear();
    int i = 0; 

    map_resolution_ = msg->info.resolution;
    map_height_ = msg->info.height;
    map_width_ = msg->info.width;

    for(int j = 0; j < map_height_; j++ )
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

void CustomMapping::publishImage(){
 
    cv::Mat inputImage(map_height_, map_width_, CV_8U);
    cv::Mat outputImage(map_height_, map_width_, CV_8UC3);
    cv::Mat erosion_dst(map_height_, map_width_, CV_8U);
    cv::Mat dilation_dst(map_height_, map_width_, CV_8U);
    cv::Mat greyMat(map_height_, map_width_, CV_8U);

    int largest_object = 0; 
    int largest_object_2 = 0; 
    int largest_object_3 = 0; 
    int largest_object_4 = 0; 
    int holding_value = 0; 

    cv::Vec3b white3C(255,255,255),
    	black3C(0,0,0),
        red(0, 0, 255),
        green (0, 255, 0);

    uint8_t white = 255;
    uint8_t black = 0;

    for(int j = 0; j < map_height_; j++ )
    {
        for(int k =0; k < map_width_; k++ )
        {
            imageCellCheck.push_back(false);
            switch (point_values_[ (j * map_height_ ) +k ].point_value)
            {
            case 0:
                inputImage.at<uint8_t>(k , j) = WHITE; //known clear space
                outputImage.at<cv::Vec3b>(k,j) = white3C; 
                break;
            case 100:
                inputImage.at<uint8_t>(k , j) = black; //object
                outputImage.at<cv::Vec3b>(k,j) = black3C;
                break;
            case -1:
                inputImage.at<uint8_t>(k , j) = WHITE; //unknown, unclear space
                outputImage.at<cv::Vec3b>(k,j) = white3C;
                break;
            default:
                inputImage.at<uint8_t>(k , j) = WHITE;
                outputImage.at<cv::Vec3b>(k,j) = white3C;
                break;
            }
        }
    }

	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
	                       cv::Size(1,1),
	                       cv::Point(-1,-1));
	
    // cv::erode( inputImage, dilation_dst, element);
    // cv::dilate( dilation_dst, erosion_dst, element);

	cv::erode(inputImage, erosion_dst, element);    

    for(int j = 0; j < map_height_; j++)
    {
        for(int k = 0; k < map_width_; k++)
        {
            if(imageCellCheck.at( (j * map_height_ ) + k) == false)
            {
                imageCellCheck.at((j * map_height_ ) + k) = true;
                if(erosion_dst.at<uint8_t>(k, j) != WHITE)
                {
                	object.clear();
                	temp.clear(); 
                    neighboursCheck(j,k, erosion_dst);
                }
            }
        }
    }

    for(int j = 0; j < objects.size(); j++)
    {
    	for(int k = 0; k < objects[j].size(); k++)
    	{
			outputImage.at<cv::Vec3b>( objects[j][k].y, objects[j][k].x ) = red;
    	}
    }

	for(int i = 0;  i < objects.size(); i++)
	{
		if(objects[i].size() > holding_value)
		{
			holding_value = objects[i].size();
			largest_object = i;  
		} 
	}

	for (int i = 0; i < objects[largest_object].size(); i++)
	{
		outputImage.at<cv::Vec3b>( objects[largest_object][i].y, objects[largest_object][i].x) = green;
	}

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", erosion_dst).toImageMsg();
    image_pub_.publish(msg);

    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputImage).toImageMsg();
    image2_pub_.publish(msg2);

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