#include "itable_pkg/itable_pkg_node.h"

//#define LOAD_CALIB
//#define IMG_CALLBACK
//#define POINTCLOUD_CALLBACK
//#define POINTCLOUD_CALLBACK1

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
        std::string temp;

        // get parameters from launch file
        private_handle.getParam("topics_quality", topics_quality);
        private_handle.getParam("package_dir_path",package_dir_path);
        // Marker
        private_handle.getParam("calculate_marker",calculate_marker);
        private_handle.getParam("marker_path",marker_path);
        private_handle.getParam("recal_marker_time",recalculate_marker_time);
        // Mask
        private_handle.getParam("calculate_mask",calculate_mask);
        private_handle.getParam("mask_mode",temp);

        if ( temp == "marker+offset")
            mask_mode = 0;
        else if ( temp == "auto")
            mask_mode = 2;
        else mask_mode = 1;

        private_handle.getParam("mask_offest",mask_offset);
        private_handle.getParam("min_mask_depth",min_mask_depth);
        private_handle.getParam("max_mask_depth",max_mask_depth);

        // Objects
        private_handle.getParam("calculate_object",calculate_object);
        private_handle.getParam("object_mode",temp);

        if ( temp == "marker+offset")
            mask_mode = 0;
        else if ( temp == "auto")
            mask_mode = 2;
        else mask_mode = 1;

        private_handle.getParam("object_offest",object_offset);
        private_handle.getParam("min_cloud_depth",min_cloud_depth);
        private_handle.getParam("max_cloud_depth",max_cloud_depth);

        private_handle.getParam("max_corr_distance",max_corr_dist);

        // Subscribe
        depth_sub.subscribe     (node_handle, "/kinect2/" + topics_quality + "/image_depth_rect", 1);
        rgb_sub.subscribe       (node_handle, "/kinect2/" + topics_quality + "/image_color_rect",1);
        pointcloud_sub.subscribe(node_handle, "/kinect2/" + topics_quality + "/points",1);
        camerainfo_sub.subscribe(node_handle, "/kinect2/" + topics_quality + "/camera_info",1);

        camerainfo_sub.registerCallback(&itable_service::caminfo_callback, this);

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

        ROS_INFO("itable_service node is running, launch file parameters loaded");
    }


    void itable_service::image_callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
    {
        ROS_INFO("new image_callback");
        cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image;
        cv::Mat depth_img = cv_bridge::toCvCopy(msg_depth)->image;
	
        double time_diff = ros::Time::now().toSec() - marker_timer;
        if ( time_diff > recalculate_marker_time && calculate_marker )
        {
            marker_timer = ros::Time::now().toSec();
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
        // No interest in pointcloud data
        if ( !calculate_mask && !calculate_object )
            return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr  after_passthrough ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_mask ( new pcl::PointCloud<pcl::PointXYZ> );

        pcl::fromROSMsg (*msg_pointcloud, *cloud_ptr);

        // Create the filtering object - threshold
        pcl::PassThrough<pcl::PointXYZ> pass_through;
        pass_through.setInputCloud (cloud_ptr);
        pass_through.setFilterFieldName ("z");


        if ( calculate_object )
        {
            if ( object_mode == 0 ) // marker + offset
            {
                if ( marker_found_valid )
                {
                    pass_through.setFilterLimits ( (marker_depth - object_offset )/ 1000.0, marker_depth / 1000.0 );
                }
                else; // do nothing
            }
            else if ( object_mode == 1) //static
                pass_through.setFilterLimits (min_cloud_depth / 1000.0, max_cloud_depth / 1000.0);
            else // auto
            {
                if ( marker_found_valid )
                {
                    pass_through.setFilterLimits ( (marker_depth - object_offset )/ 1000.0, marker_depth / 1000.0 );
                }
                else
                    pass_through.setFilterLimits (min_cloud_depth / 1000.0, max_cloud_depth / 1000.0);
            }

            pass_through.filter (*after_passthrough);
        }


        if ( calculate_mask )
        {
            if ( object_mode == 0 ) // marker + offset
            {
                if ( marker_found_valid )
                {
                    pass_through.setFilterLimits ( (marker_depth - mask_offset )/ 1000.0, marker_depth / 1000.0 );
                }
                else; // do nothing
            }
            else if ( object_mode == 1) //static
                pass_through.setFilterLimits ( min_mask_depth / 1000.0, max_mask_depth / 1000.0);
            else // auto
            {
                if ( marker_found_valid )
                {
                    pass_through.setFilterLimits ( (marker_depth - mask_offset )/ 1000.0, marker_depth / 1000.0 );
                }
                else
                    pass_through.setFilterLimits ( min_mask_depth / 1000.0, max_mask_depth / 1000.0);
            }

            pass_through.filter(*cloud_mask);
        }

        if ( calculate_mask)
        {
            recalculate_mask( cloud_mask, cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image );
        }


#ifdef POINTCLOUD_CALLBACK1
        cv::Mat proj = cv::Mat(800,1280, CV_8U, cvScalar(0.));
        for ( int i =0;i < mask_points.size() ; i++)
            proj.at<uchar>( mask_points[i].y,mask_points[i].x) = 255;

        cv::imwrite("/home/petr/proj.bmp",proj);

        std::vector<cv::Point2f> pointz;

        for ( int i =0; i < cloud_mask->size() ;i++)
            pointz.push_back( project3D_to_pixel(cv::Point3f((*cloud_mask)[i].x,(*cloud_mask)[i].y,(*cloud_mask)[i].z)));


        cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image;
        //rectangle(rgb_img, minAreaRect(points), cv::Scalar(0,0,255,255));
        for ( int i = 0;i < pointz.size(); i++)
        {
            circle(rgb_img, pointz[i], 0.5, cv::Scalar(0,0,255,255));
        }

        cv::namedWindow( "RGB with circles", cv::WINDOW_AUTOSIZE );
        cv::resizeWindow("RGB with circles", 1024, 768);
        cv::imshow( "RGB with circles", rgb_img );
        cv::resizeWindow("RGB with circles", 1024, 768);

        cv::waitKey(0);

#endif

        if ( calculate_object && object_box_loaded)
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
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (400);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_without_plane);
        ec.extract (cluster_indices);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations (300);
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance ( max_corr_dist );
        icp.setInputTarget (cloud_box);
        //icp.setInputSource (cloud_box);

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

            //pcl::io::savePCDFileASCII ("/home/petr/test_pcd" + a +".pcd", *cloud_cluster);

            //a.append("a");
            pcl::PointCloud<pcl::PointXYZ>::Ptr  temp ( new pcl::PointCloud<pcl::PointXYZ> );
            icp.setInputSource (cloud_cluster);
            //icp.setInputTarget (cloud_cluster);

            icp.align(*temp);

            if (icp.hasConverged ())
            {
                std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << " and size "<< cloud_cluster->size() << std::endl;
                //std::cout << "number of points " << cloud_cluster->size() << " box "<< cloud_box->size()<<std::endl;
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
       // std::cout << cloud_lowest_score->size() << std::endl;

        // Object found, now project it to projector space
        std::vector<cv::Point3f> points;
        std::vector<cv::Point2f> projected_points;

        std::vector<cv::Point2f> boxx;

        // Pointcloud of object
        for( int i =0; i < cloud_lowest_score->size(); i++)
        {
            points.push_back(cv::Point3f((*cloud_lowest_score)[i].x,(*cloud_lowest_score)[i].y,(*cloud_lowest_score)[i].z));
            boxx.push_back( project3D_to_pixel(cv::Point3f((*cloud_lowest_score)[i].x,(*cloud_lowest_score)[i].y,(*cloud_lowest_score)[i].z)) );
        }

        if ( min_score == 1.0f || points.empty() )
        {
            ROS_INFO("Object not found in pointcloud");
            objects.clear();
            publish_objects();
            return;
        }

        try
        {
            // Project pointcloud to projector space
            cv::projectPoints(points, rot_vec, trans_vec, proj_cam_mat, dist_coeffs, projected_points);
        }
        catch ( cv::Exception& e )
        {
            const char* err_msg = e.what();
            ROS_ERROR("Calling cv::projectPoint failed");
            ROS_ERROR("Caught OpenCV Exception: %s",err_msg);
            return;
        }

        // Save object information and later publish
        objects.clear();
        object new_object;
        //cv::RotatedRect b_box   = minAreaRect(boxx);
        cv::RotatedRect b_box   = minAreaRect(projected_points);
        new_object.center_x     = b_box.center.x;
        new_object.center_y     = b_box.center.y;
        new_object.width        = b_box.size.width;
        new_object.height       = b_box.size.height;
        new_object.angle        = b_box.angle;

        objects.push_back(new_object);
        publish_objects();

#ifdef POINTCLOUD_CALLBACK
        //cv::Mat rgb_img = cv_bridge::toCvShare(msg_rgb, msg_rgb->encoding)->image;
        //rectangle(rgb_img, minAreaRect(points), cv::Scalar(0,0,255,255));
        for ( int i = 0;i < boxx.size(); i++)
        {
            circle(rgb_img, boxx[i], 0.5, cv::Scalar(0,0,255,255));
        }

        cv::namedWindow( "RGB with circles", cv::WINDOW_AUTOSIZE );
        cv::resizeWindow("RGB with circles", 1024, 768);
        cv::imshow( "RGB with circles", rgb_img );
        cv::resizeWindow("RGB with circles", 1024, 768);

        cv::waitKey(0);

#endif
    }

    void itable_service::recalculate_mask(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask, cv::Mat rgb_img)
    {

        cv::Mat bw = cv::Mat(rgb_img.size().height,rgb_img.size().width, CV_8U, cvScalar(0.));

        std::vector< cv::Point2f > projected_points;
        std::vector< cv::Point3f > cloud_points;

        for ( int i = 0; i < cloud_mask->size(); i++)
        {
            pcl::PointXYZ point = (*cloud_mask)[i];
            if ( pcl::isFinite(point) )
            {
                cloud_points.push_back( cv::Point3f(point.x,point.y,point.z) );
                cv::Point2f p =  project3D_to_pixel(cv::Point3f(point.x,point.y,point.z));
                if ( p.x < rgb_img.size().width && p.y < rgb_img.size().height
                    && p.x > 0 && p.y > 0 )
                            bw.at<uchar>( p.y, p.x) = 255;

            }
        }

        try
        {
            cv::projectPoints(cloud_points, rot_vec, trans_vec, proj_cam_mat, dist_coeffs, projected_points);
        }
        catch( cv::Exception& e )
        {
            return;
        }

        cv::Mat bw2 = cv::Mat(1024,1280, CV_8U, cvScalar(0.));
        for( std::vector< cv::Point2f >::iterator it = projected_points.begin() ; it != projected_points.end() ; it++ )
        {
            if ( it->x < 1280 && it->y < 1024
                && it->x > 0 && it->y > 0 )
                        bw2.at<uchar>( it->y, it->x) = 255;
        }

        //cv::projectPoints(cloud_points, rot_vec, trans_vec, proj_cam_mat, dist_coeffs, projected_points);


        cv::Mat bw3;

        cv::dilate(bw2, bw3, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
/*
        cv::namedWindow( "BW1", cv::WINDOW_AUTOSIZE );
        cv::imshow( "BW1", bw);
        cv::waitKey(0);

        cv::namedWindow( "BW", cv::WINDOW_AUTOSIZE );
        cv::imshow( "BW", bw2);
        cv::resizeWindow("BW", 1024, 768);

        cv::waitKey(0);
*/
        std::vector< std::vector< cv::Point> > contours;
        findContours(bw3, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        /*
        std::vector< std::vector< cv::Point> > contours2;

        // Reducing amount of contours...
        for( int i = 0; i < contours.size(); i++ )
        {
            if ( contours[i].size() > 1)
                contours2.push_back(contours[i]);
        }
        */


        //std::vector< std::vector<cv::Point> > convex_hulls (contours.size());
        //convex_hulls.clear();
        convex_hulls.resize(contours.size());
        for( int i = 0; i < contours.size(); i++ )
        {
            convexHull( contours[i], convex_hulls[i], true );
        }
/*
        for ( int i = 0; i < convex_hulls.size() ; i++)
        {
           cv::drawContours(bw2, convex_hulls, i, cv::Scalar(255,0,255,255));
        }

        cv::namedWindow( "RGB with circles", cv::WINDOW_AUTOSIZE );
        cv::resizeWindow("RGB with circles", 1024, 768);
        cv::imshow( "RGB with circles", bw2);
        cv::resizeWindow("RGB with circles", 1024, 768);

        cv::waitKey(0);
*/

        publish_mask();
        ROS_INFO("Mask recalculated");
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
        marker_msg.valid = marker_found_valid;

        marker_pub.publish( marker_msg );
    }

    void itable_service::publish_mask()
    {

        itable_pkg::hull_point mp;

        itable_pkg::mask mask;
        for ( std::vector< std::vector<cv::Point> >::iterator it = convex_hulls.begin(); it != convex_hulls.end(); it++)
        {
            itable_pkg::hull hull;
            for ( std::vector<cv::Point>::iterator it2 = it->begin(); it2 != it->end(); it2++)
            {
                mp.x = it2->x;
                mp.y = it2->y;
                hull.hull.push_back(mp);
            }
            mask.mask.push_back(hull);
        }
        mask.header.stamp = ros::Time::now();
        mask_pub.publish(mask);

    }

    void itable_service::publish_objects()
    {

        itable_pkg::object obj;
        itable_pkg::objects obj_array;
        for ( std::vector<object>::iterator it = objects.begin(); it != objects.end(); it++)
        {
            obj.center_x = it->center_x;
            obj.center_y = it->center_y;
            obj.width    = it->width;
            obj.height   = it->height;
            obj.angle    = it->angle;

            obj_array.objects.push_back( obj );
        }

        objects_pub.publish(obj_array);
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
            dist_coeffs  = cv::Mat(5,1, CV_64F, cvScalar(0.));
            rot_vec      = cv::Mat(3,1, CV_64F, cvScalar(0.));
            trans_vec    = cv::Mat(3,1, CV_64F, cvScalar(0.));
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
        for(int i = 0; i < 3; i++, ptr++)
        {
            proj_cam_msg.rotation[i] = *ptr;
        }

        ptr = trans_vec.ptr<double>(0, 0);
        for(int i = 0; i < 3; i++, ptr++)
        {
            proj_cam_msg.translation[i] = *ptr;
        }

        proj_cam_msg.header.stamp = ros::Time::now();

        fs.release();

        if ( calculate_marker )
        {
            marker_img = cv::imread( marker_path, CV_LOAD_IMAGE_GRAYSCALE );

            if( !marker_img.data )
            {
                marker_loaded = false;
                ROS_ERROR("Could NOT load marker image file %s. Published homography matrix will be invalid",marker_path.c_str());
            }
            else
                marker_loaded = true;

        }

        if ( calculate_object )
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (package_dir_path + "box.pcd", *cloud_box) == -1) //* load the file
            {
                ROS_ERROR ("Couldn't read file box.pcd in %s. Published box data will be invalid.",package_dir_path.c_str());
                object_box_loaded = false;
            }
            else
                object_box_loaded = true;
        }

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
            publish_marker();
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

        publish_marker();
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

        float fx     = cam_intrinsic.at<double>(0,0);
        float fy     = cam_intrinsic.at<double>(1,1);
        float cx     = cam_intrinsic.at<double>(0,2);
        float cy     = cam_intrinsic.at<double>(1,2);

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

        itable.publish_proj_cam();
        ros::spinOnce();
    }


    return 0;
}


