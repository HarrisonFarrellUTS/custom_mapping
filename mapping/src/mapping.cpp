#include "mapping.h" 

#define requiredHz 1
#define WHITE 255
#define GREY 150

CustomMapping::CustomMapping(ros::NodeHandle nh)
    : nh_(nh) , it_(nh)

{
    sub2_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &CustomMapping::occupancyGridCallback,this);

    image_pub_ = it_.advertise("/image_test", 10);

    image2_pub_ = it_.advertise("/processed_image", 10);

    OccupancyGrid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/output_map", 10);
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
    map_info_ = msg->info;
    map_resolution_ = msg->info.resolution;
    map_height_ = msg->info.height;
    map_width_ = msg->info.width;
    ROS_INFO("INPUT_OG Data: height %d, width %d, resolution %f, size %d", msg->info.height, msg->info.width, msg->info.resolution, msg->data.size());

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
    //int outputArray[map_width_ * map_height_];

    cv::Vec3b white3C(255,255,255),
        grey3C(150, 150, 150),
    	black3C(0,0,0),
        red(30, 100, 200),
        green (200, 100, 40);

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

/////////////////////////////////////////////// NEW CODE vvvvvvvvvvvvvvvvv
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
/////////////////////////////////////////////// NEW CODE ^^^^^^^^^^^^^^

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

    //std::vector<signed char> outputVector(outputArray, outputArray + (map_height_ * map_width_));

	nav_msgs::OccupancyGrid msg3;
    msg3.info = map_info_;
	msg3.data = outputVector; 

    ROS_INFO("OUTPUT_OG Data: height %d, width %d, resolution %f, size %d", msg3.info.height, msg3.info.width, msg3.info.resolution, msg3.data.size());
	 

	OccupancyGrid_pub_.publish(msg3);

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

/////////////////////////////////////////////// NEW CODE vvvvvvvvvvvvvvvvv

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

/////////////////////////////////////////////// NEW CODE ^^^^^^^^^^^^^^^


/*


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  ROS_INFO("Got map %d %d", info.width, info.height);
  Map map(info.width, info.height);
  for (unsigned int x = 0; x < info.width; x++)
    for (unsigned int y = 0; y < info.height; y++)
      map.Insert(Cell(x,y,info.width,msg->data[x+ info.width * y]));
  nav_msgs::OccupancyGrid* newGrid = map.Grid();
  newGrid->header = header;
  newGrid->info = info;
  map_pub.publish(*newGrid);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "grid");
  ros::NodeHandle n;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
  ros::Subscriber map_sub = n.subscribe("map",10,mapCallback);
  

  */
