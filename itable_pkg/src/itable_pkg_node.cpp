#include "itable_pkg/itable_pkg_node.h"


namespace itable
{
    itable_service::itable_service()
    {
        marker_homography = cv::Mat(3,3, CV_64F, cvScalar(0.));
    }

    itable_service::~itable_service()
    {


    }

    void itable_service::ros_init()
    {
        // Subscribe
        depth_sub.subscribe(node_handle, "/kinect2/qhd/depth_color_rect", 1);
        rgb_sub.subscribe(node_handle, "/kinect2/qhd/image_color_rect",1);
        pointcloud_sub.subscribe(node_handle,"/kinect2/qhd/points",1);
        camerainfo_sub.subscribe(node_handle,"/kinect2/qhd/camera_info",1);

        //synchronizer_rgb_depth.reset( new synchronizer_rgb_depth(exact_sync_rgb_depth(10), rgb_sub, depth_sub));
        sync_rgb_depth = new synchronizer_rgb_depth(exact_sync_rgb_depth(10), rgb_sub, depth_sub);
        sync_rgb_depth->registerCallback(&itable_service::image_callback, this);

        //synchronizer_pointcloud.reset( new synchronizer_pointcloud( exact_sync_pointcloud_rgb(10),pointcloud_sub,rgb_sub));
        sync_pointcloud = new synchronizer_pointcloud( exact_sync_pointcloud_rgb(10),pointcloud_sub,rgb_sub);
        sync_pointcloud->registerCallback(&itable_service::pointcloud_callback, this);


        // Publish
        proj_cam_data_pub = node_handle.advertise<itable_pkg::proj_cam_data>("projector_camera_data",1);
        homography_pub    = node_handle.advertise<itable_pkg::marker_location>("marker_data",1);
        mask_pub          = node_handle.advertise<itable_pkg::mask>("mask_data",1);
        objects_pub       = node_handle.advertise<itable_pkg::objects>("objects_data",10);

        private_handle.getParam("topics_quality", topics_quality);
        private_handle.getParam("package_dir_path",package_dir_path);
        private_handle.getParam("marker_path",marker_path);

        ROS_INFO("itable_service node is running");
    }


    void itable_service::image_callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
    {
        cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, "bgr8")->image;
        cv::Mat depth_img = cv_bridge::toCvShare(msg_depth, "bgr8")->image;

        if ( recalculate_homography )
        {
            find_homography(rgb_img);
            //TODO Nalezt depth markeru ve scene pomoci depth_img
        }


    }


    void itable_service::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud,const sensor_msgs::ImageConstPtr& msg_rgb)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr  after_passthrough ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::fromROSMsg (*msg_pointcloud, *cloud_ptr);

        // Create the filtering object - threshold
        pcl::PassThrough<pcl::PointXYZ> pass_through(true);
        pass_through.setInputCloud (cloud_ptr);
        pass_through.setFilterFieldName ("z");
        pass_through.setFilterLimits (1.3, 1.7);
        pass_through.filter (*after_passthrough);

        if ( recalculate_mask )
        {
            int width  = cloud_ptr->width;
            int height = cloud_ptr->height;
            cv::Mat points_to_mask(width, height, CV_8UC1, cv::Scalar(0));
            pcl::IndicesConstPtr removed_indices = pass_through.getRemovedIndices();
            // Making black & white image for further processing
            for ( int i = 0; i < removed_indices->size(); i++)
            {
                points_to_mask.at<uchar>( (*removed_indices)[i] / width, (*removed_indices)[i] % height) = 255;
            }

            std::vector< std::vector< cv::Point> > contours;
            findContours(points_to_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            std::vector< std::vector< cv::Point> > contours2;

            // Reducing amount of contours...
            for( int i = 0; i < contours.size(); i++ )
                if ( contours[i].size() > 60)
                    contours2.push_back(contours[i]);

            std::vector< std::vector<int> > convex_hulls (contours2.size());

            for( int i = 0; i < contours2.size(); i++ )
                convexHull( contours2[i], convex_hulls[i], false );

            //TODO make as function? projectpoint to projector space

        }

    }


    void itable_service::caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo)
    {


    }

    void itable_service::publish_proj_cam()
    {
        proj_cam_data_pub.publish( proj_cam_msg );
    }

    void itable_service::publish_homography()
    {
        marker_msg.header.stamp = ros::Time::now();

        const double *ptr = marker_homography.ptr<double>(0, 0);
        for(int i = 0; i < 9; i++, ptr++)
            marker_msg.homography[i] = *ptr;

        marker_msg.depth = marker_depth;

        homography_pub.publish( marker_msg );
    }

    void itable_service::publish_mask()
    {
        itable_pkg::convex_hull dummy_convex_hull;
        dummy_convex_hull.convex_hull.push_back(0);

        mask_msg.header.stamp = ros::Time::now();
        mask_msg.mask.clear();
        mask_msg.mask.push_back( dummy_convex_hull );

        mask_pub.publish( mask_msg );
    }

    void itable_service::publish_objects()
    {
        itable_pkg::object dummy;
        itable_pkg::objects dummy_array;

        dummy_array.objects.push_back(dummy);

        objects_pub.publish(dummy_array);
    }

    void itable_service::publish_all()
    {
        publish_proj_cam();
        publish_homography();
        publish_mask();
        publish_objects();
    }

    void itable_service::load_data_from_files()
    {
        cv::FileStorage fs;
        try
        {
            fs.open(package_dir_path + std::string("calibration_data.yaml"), cv::FileStorage::READ);
        }
        catch ( cv::Exception& e )
        {
            const char* err_msg = e.what();
            ROS_ERROR("Caught OpenCV Exception: %s",err_msg);
        }

        if ( !fs.isOpened() ) // cannot open file
        {
            ROS_ERROR("Error opening file %s. Published calibration data will be invalid", (package_dir_path + std::string("calibration_data.yaml")).c_str() );
            proj_cam_mat = cv::Mat(3,3, CV_64F, cvScalar(0.));
            dist_coeffs  = cv::Mat(1,5, CV_64F, cvScalar(0.));
            rot_vec      = cv::Mat(3,3, CV_64F, cvScalar(0.));
            trans_vec    = cv::Mat(3,3, CV_64F, cvScalar(0.));
        }
        else // file opened
        {
            fs["camera_matrix"] >> proj_cam_mat;
            fs["dist_coeffs"]   >> dist_coeffs;
            fs["rotation_vec"]  >> rot_vec;
            fs["trans_vec"]     >> trans_vec;
        }

        const double *ptr = proj_cam_mat.ptr<double>(0, 0);
        for(int i = 0; i < 9; i++, ptr++)
        {
            proj_cam_msg.intrinsic[i] = *ptr;
        }

        ptr = dist_coeffs.ptr<double>(0, 0);
        for(int i = 0; i < 5; i++, ptr++)
        {
            proj_cam_msg.distortion[i] = *ptr;
        }

        ptr = rot_vec.ptr<double>(0, 0);
        for(int i = 0; i < 9; i++, ptr++)
        {
            proj_cam_msg.rotation[i] = *ptr;
        }

        ptr = trans_vec.ptr<double>(0, 0);
        for(int i = 0; i < 9; i++, ptr++)
        {
            proj_cam_msg.translation[i] = *ptr;
        }

        proj_cam_msg.header.stamp = ros::Time::now();

        fs.release();

        marker_img = cv::imread( marker_path, CV_LOAD_IMAGE_GRAYSCALE );

        if( !marker_img.data )
        {
            marker_loaded = false;
            ROS_ERROR("Could NOT load marker image file %s. Published homography matrix will be invalid",marker_path.c_str());
        }
        else
            marker_loaded = true;

    }

    void itable_service::find_homography(cv::Mat& rgb_img)
    {
        // Check if marker image is successfully loaded
        if ( !marker_loaded )
            return;
        // Detect keypoint with SURF det.
        cv::SurfFeatureDetector detector( 600 );
        std::vector<cv::KeyPoint> keypoints_marker, keypoints_scene;

        detector.detect( marker_img, keypoints_marker );
        detector.detect( rgb_img, keypoints_scene );

        // Calculate descriptors (feature vectors)
        cv::SurfDescriptorExtractor extractor;
        cv::Mat descriptors_marker, descriptors_scene;

        extractor.compute( marker_img, keypoints_marker, descriptors_marker );
        extractor.compute( rgb_img, keypoints_scene, descriptors_scene );

        // Matching descriptor vectors using FLANN matcher + KNN
        cv::FlannBasedMatcher matcher;
        std::vector< std::vector< cv::DMatch >  > matches;
        matcher.knnMatch( descriptors_marker, descriptors_scene, matches, 2 ); // find the 2 nearest neighbors

        std::vector< cv::DMatch > good_matches;
        float nndrRatio = 0.40f;

        for (size_t i = 0; i < matches.size(); ++i)
        {
            if (matches[i].size() < 2)
                        continue;

            const cv::DMatch &m1 = matches[i][0];
            const cv::DMatch &m2 = matches[i][1];

            if(m1.distance <= nndrRatio * m2.distance)
                good_matches.push_back(m1);
        }

        //-- Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
          //-- Get the keypoints from the good matches
          obj.push_back( keypoints_marker[ good_matches[i].queryIdx ].pt );
          scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        marker_homography = cv::findHomography( obj, scene);//, CV_RANSAC );
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "itable_service");

    itable::itable_service itable;

    itable.ros_init();

    itable.load_data_from_files();




    while( ros::ok() )
    {

        itable.publish_all();

    }


    return 0;
}






















/**

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<itable_pkg::marker_location>("marker",10);

  ros::Rate loop_rate(10);


  int count = 0;
  /*
  while (ros::ok())
  {

    itable_pkg::marker_location msg;

	//msg.homography = new float[9] { 0,1,2,3,4,5,6,7,8 };

	for ( int i = 0; i < 9 ;i++)
		msg.homography[i] = i;

	msg.valid = true;
	msg.depth_value = 1000;


    chatter_pub.publish(msg);

    //ros::spinOnce();
    ros::spin();
/*
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
*/
