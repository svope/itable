#include "itable_pkg/itable_pkg_node.h"

#define LOAD_CALIB
//#define IMG_CALLBACK
//#define POINTCLOUD_CALLBACK
namespace itable
{
    itable_service::itable_service()
    {
        marker_homography = cv::Mat(3,3, CV_64F, cvScalar(0.));


#ifdef LOAD_CALIB
        cv::FileStorage fs("/home/petr/Desktop/pkg/src/iai_kinect2/kinect2_bridge/data/000393642047/calib_color.yaml", cv::FileStorage::READ);
        cv::Mat testCM,testDC;
        fs["cameraMatrix"] >> testCM ;
        fs["distortionCoefficients"] >> testDC;
        fs.release();
        cam_info_set = true;
        cam_intrinsic = testCM;
        cam_dist_coeffs = testDC;
#endif


    }

    itable_service::~itable_service()
    {


    }

    void itable_service::ros_init()
    {
        // Subscribe
        depth_sub.subscribe(node_handle, "/kinect2/qhd/image_depth_rect", 1);
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
        marker_pub        = node_handle.advertise<itable_pkg::marker_location>("marker_data",1);
        mask_pub          = node_handle.advertise<itable_pkg::mask>("mask_data",1);
        objects_pub       = node_handle.advertise<itable_pkg::objects>("objects_data",10);

        private_handle.getParam("topics_quality", topics_quality);
        private_handle.getParam("package_dir_path",package_dir_path);
        private_handle.getParam("marker_path",marker_path);

        ROS_INFO("itable_service node is running");
    }


    void itable_service::image_callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
    {
        ROS_INFO("new image_callback");
        cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image;
        cv::Mat depth_img = cv_bridge::toCvCopy(msg_depth)->image;
	
        if ( recalculate_marker_pos )
        {
            find_marker(rgb_img, depth_img);

#ifdef IMG_CALLBACK
            cv::Point2f uno,dos,tres;
            uno = cv::Point2f(273,227);

            std::vector<cv::Point2f> marker_points,camera_points;

            marker_points.push_back( uno );

            perspectiveTransform( marker_points, camera_points, marker_homography);

            circle(rgb_img, camera_points[0], 5.0, cv::Scalar(255,0,0,255));

            cv::namedWindow( "RGB with circles", cv::WINDOW_AUTOSIZE );
            cv::imshow( "RGB with circles", rgb_img );
		
            cv::waitKey(0);
#endif

        }

    }


    void itable_service::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud,const sensor_msgs::ImageConstPtr& msg_rgb)
    {
        ROS_INFO("Pointcloud callback");
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr  after_passthrough ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::fromROSMsg (*msg_pointcloud, *cloud_ptr);

        // Create the filtering object - threshold
        pcl::PassThrough<pcl::PointXYZ> pass_through(true);
        pass_through.setInputCloud (cloud_ptr);
        pass_through.setFilterFieldName ("z");

        if ( marker_found_valid )
            pass_through.setFilterLimits ( marker_depth / 1000.0, marker_depth / 1000.0 + 0.25);
        else
            pass_through.setFilterLimits (1.2, 1.5);
        pass_through.filter (*after_passthrough);

        if ( recalculate_mask_flag )
        {
            recalculate_mask( cloud_ptr, pass_through.getRemovedIndices() );
        }

        if ( !find_object )
        {
            find_object_in_pointcloud(after_passthrough,cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image);
        }

    }

    void itable_service::find_object_in_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, cv::Mat rgb_img)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_without_plane ( new pcl::PointCloud<pcl::PointXYZ> );

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (pointcloud);
        vg.setLeafSize (0.01f, 0.01f, 0.005f);
        vg.filter (*cloud_filtered);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            ROS_INFO("Could not find planar model for this pointcloud. This could affect object recognition");
        }


        // Extract the inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_without_plane);

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_without_plane);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.01); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_without_plane);
        ec.extract (cluster_indices);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations (1);
        icp.setInputTarget (cloud_box);

        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_lowest_score ( new pcl::PointCloud<pcl::PointXYZ> );

        float min_score = 1.0;
        //std::string a ("a");
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_without_plane->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

           // pcl::io::savePCDFileASCII ("/home/petr/test_pcd" + a +".pcd", *cloud_cluster);

           // a.append("a");
            pcl::PointCloud<pcl::PointXYZ>::Ptr  temp ( new pcl::PointCloud<pcl::PointXYZ> );
            icp.setInputSource (cloud_cluster);

            icp.align(*temp);

            if (icp.hasConverged ())
            {
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                if ( icp.getFitnessScore () < min_score )
                {
                    cloud_lowest_score = cloud_cluster;
                    min_score = icp.getFitnessScore ();
                }
            }
            else
                ROS_INFO("Object hasn't converged");

        }

        ROS_INFO("Min score is %f",min_score);
        std::cout << cloud_lowest_score->size() << std::endl;

        std::vector<cv::Point2f> points;
        for( int i =0; i < cloud_lowest_score->size(); i++)
        {
            points.push_back(project3D_to_pixel(cv::Point3f((*cloud_lowest_score)[i].x,(*cloud_lowest_score)[i].y,(*cloud_lowest_score)[i].z)));
            //std::cout << cv::Point3f((*cloud_lowest_score)[i].x,(*cloud_lowest_score)[i].y,(*cloud_lowest_score)[i].z) << std::endl;
        }

#ifdef POINTCLOUD_CALLBACK

        //cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image;
        for ( int i = 0;i < points.size(); i++)
        {
            circle(rgb_img, points[i], 0.5, cv::Scalar(0,0,255,255));
        }

        cv::namedWindow( "RGB with circles", cv::WINDOW_AUTOSIZE );
        cv::resizeWindow("RGB with circles", 1024, 768);
        cv::imshow( "RGB with circles", rgb_img );
        cv::resizeWindow("RGB with circles", 1024, 768);

        cv::waitKey(1);

#endif
    }


    void itable_service::recalculate_mask( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, pcl::IndicesConstPtr removed_indices)
    {
        int width  = cloud_ptr->width;
        int height = cloud_ptr->height;
        cv::Mat points_to_mask(height, width, CV_8UC1, cv::Scalar(0));

        std::vector<cv::Point3f> mask_points;
        std::vector<cv::Point2f> projected_points;

        for ( int i = 0; i < removed_indices->size(); i++)
        {
            pcl::PointXYZ point = cloud_ptr->at( (*removed_indices)[i] );
            if ( pcl::isFinite(point) )
            	mask_points.push_back( cv::Point3f(point.x,point.y,point.z));
        }

        // Mask points projected to projector space
        cv::projectPoints(mask_points, rot_vec, trans_vec, proj_cam_mat, dist_coeffs, projected_points);


	
        for ( int i = 0; i < projected_points.size(); i++)
        {
		//std::cout << projected_points[i].x << " " << projected_points[i].y << std::endl;
		if ( projected_points[i].x < width && projected_points[i].y < height
			&& projected_points[i].x > 0 && projected_points[i].y > 0 )
            		points_to_mask.at<uchar>( projected_points[i].y, projected_points[i].x) = 255;
        }
        std::cout << points_to_mask.size() <<std::endl;
        //pcl::IndicesConstPtr removed_indices = pass_through.getRemovedIndices();
        // Making black & white image for further processing
        /*
        for ( int i = 0; i < removed_indices->size(); i++)
        {
            points_to_mask.at<uchar>( (*removed_indices)[i] / width, (*removed_indices)[i] % height) = 255;
        }
        */

        std::vector< std::vector< cv::Point> > contours;
        findContours(points_to_mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        std::vector< std::vector< cv::Point> > contours2;

        // Reducing amount of contours...
        for( int i = 0; i < contours.size(); i++ )
            if ( contours[i].size() > 1)
                contours2.push_back(contours[i]);

        std::vector< std::vector<cv::Point> > convex_hulls (contours2.size());
        for( int i = 0; i < contours2.size(); i++ )
            convexHull( contours2[i], convex_hulls[i], true );

        //TODO make as function? projectpoint to projector space.

    }

    void itable_service::caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo)
    {
        if ( cam_info_set )
            return;

        cam_intrinsic = cv::Mat(3,3,CV_64F,cvScalar(0.));
        double *ptrI = cam_intrinsic.ptr<double>(0, 0);

        for ( int i = 0; i < 9 ; i++, ptrI++)
            *ptrI = msg_camerainfo.K[i];

        // Expecting plum bob model -> 5 parameters
        cam_dist_coeffs = cv::Mat(5,1,CV_64F, cvScalar(0.));
        double *ptrD = cam_dist_coeffs.ptr<double>(0);

        for ( int i = 0; i < 5; i++, ptrD++)
            *ptrD = msg_camerainfo.D[i];

        cam_info_set = true;
        ROS_INFO("Camera intrinsic and dist_coeffs loaded from CameraInfo");
    }

    void itable_service::publish_proj_cam()
    {
        proj_cam_data_pub.publish( proj_cam_msg );
    }

    void itable_service::publish_marker()
    {
        marker_msg.header.stamp = ros::Time::now();

        const double *ptr = marker_homography.ptr<double>(0, 0);
        for(int i = 0; i < 9; i++, ptr++)
            marker_msg.homography[i] = *ptr;

        marker_msg.depth = marker_depth;

        marker_pub.publish( marker_msg );
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
        publish_marker();
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



        if (pcl::io::loadPCDFile<pcl::PointXYZ> (package_dir_path + "box.pcd", *cloud_box) == -1) //* load the file
        {
            ROS_ERROR ("Couldn't read file box.pcd in %s. Published box data will be invalid.",package_dir_path.c_str());
        }
        else
            object_box_loaded = true;

        ROS_INFO("Data from files loaded successfully");

    }

    void itable_service::find_marker(cv::Mat& rgb_img, cv::Mat& depth_img)
    {
        // Check if marker image is successfully loaded
        if ( !marker_loaded )
            return;
        // Detect keypoint with SURF det.
        cv::SurfFeatureDetector detector( 800 );
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
        float nndrRatio = 0.50f;

        for (size_t i = 0; i < matches.size(); ++i)
        {
            if (matches[i].size() < 2)
                        continue;

            const cv::DMatch &m1 = matches[i][0];
            const cv::DMatch &m2 = matches[i][1];

            if(m1.distance <= nndrRatio * m2.distance)
                good_matches.push_back(m1);
        }

        if ( good_matches.size() < 4 ) // minimum points
        {
            ROS_INFO("Could not find marker in the scene. There are less than 4 matches. Published homography is NOT valid");
            marker_found_valid = false;
            return;
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



        try
        {
            marker_homography = cv::findHomography( obj, scene);//, CV_RANSAC );
        }
        catch ( cv::Exception& e )
        {
            const char* err_msg = e.what();
            ROS_ERROR("Couldn't find homography in function find_marker()");
            ROS_ERROR("Caught OpenCV Exception: %s",err_msg);
            return;
        }

        // Get a point from marker -> scene to get a depth
        std::vector<cv::Point2f> marker_points,camera_points;
        marker_points.push_back( cv::Point2f( 300, 300 ) );
        perspectiveTransform( marker_points, camera_points, marker_homography);

        float depth = depth_img.at<unsigned short>(camera_points[0].y,camera_points[0].x);
        marker_depth = depth;

        marker_found_valid = true;

        ROS_INFO("Marker found with depth = %f mm",marker_depth);
    }

    void itable_service::project3D_to_pixel( std::vector<cv::Point3d>& input_3D, std::vector<cv::Point2f>& output_2D )
    {
        if ( !cam_info_set )
            return;

        float fx     = cam_intrinsic.at<double>(0,0);
        float fy     = cam_intrinsic.at<double>(1,1);
        float cx     = cam_intrinsic.at<double>(0,2);
        float cy     = cam_intrinsic.at<double>(1,2);

        for ( std::vector<cv::Point3d>::iterator it = input_3D.begin(); it != input_3D.end() ; it++)
        {
            output_2D.push_back ( cv::Point2f( it->x * fx / it->z + cx, it->y * fy / it->z + cy) );
        }
    }

    void itable_service::backproject_pixel_to_3D( std::vector<cv::Point3f>& input, std::vector<cv::Point3f>& output)
    {
        if ( !cam_info_set )
            return;

        float fx     = 1.f / cam_intrinsic.at<double>(0,0);
        float fy     = 1.f / cam_intrinsic.at<double>(1,1);
        float cx     = cam_intrinsic.at<double>(0,2);
        float cy     = cam_intrinsic.at<double>(1,2);

        for ( std::vector<cv::Point3f>::iterator it = input.begin(); it != input.end() ; it++)
        {
            output.push_back ( cv::Point3f( (it->x - cx) * (it->z/1000.0) * fx, (it->y - cy) * (it->z/1000.0) * fy, it->z/1000.0));
        }
    }

    cv::Point2f itable_service::project3D_to_pixel(cv::Point3f point3D)
    {
        if ( !cam_info_set )
            return cv::Point2f();

        float fx     = cam_intrinsic.at<double>(0,0) / 2.0;
        float fy     = cam_intrinsic.at<double>(1,1) / 2.0;
        float cx     = cam_intrinsic.at<double>(0,2) / 2.0;
        float cy     = cam_intrinsic.at<double>(1,2) / 2.0;

        float dist = point3D.z;
        float X    = (point3D.x  * fx / dist) + cx;
        float Y    = (point3D.y  * fy / dist) + cy;

        return cv::Point2f(X,Y);
    }

    cv::Point3f itable_service::backproject_pixel_to_3D( cv:: Point2f cam_2D, float depth)
    {
        if ( !cam_info_set )
            return cv::Point3f();

        float fx     = 1.f / cam_intrinsic.at<double>(0,0);
        float fy     = 1.f / cam_intrinsic.at<double>(1,1);
        float cx     = cam_intrinsic.at<double>(0,2);
        float cy     = cam_intrinsic.at<double>(1,2);
        float factor = 1.f/1000.f;

        float dist = depth * factor;
        float X    = (cam_2D.x - cx) * dist * fx;
        float Y    = (cam_2D.y - cy) * dist * fy;

        return cv::Point3f(X,Y,dist);
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
        ros::spinOnce();
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
