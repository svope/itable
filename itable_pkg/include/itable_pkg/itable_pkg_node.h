#ifndef _ITABLE_PKG_NODE
#define _ITABLE_PKG_NODE

#include <algorithm>
#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "itable_pkg/marker_location.h"
#include "itable_pkg/object.h"
#include "itable_pkg/objects.h"
#include "itable_pkg/proj_cam_data.h"
#include "itable_pkg/mask.h"
#include "itable_pkg/hull_point.h"
#include "itable_pkg/hull.h"

#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>



namespace itable
{

// Typedefs for new data types
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud2_subscriber;
typedef message_filters::Subscriber<sensor_msgs::Image> image_subscriber;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> camerainfo_subscriber;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> exact_sync_rgb_depth;
typedef message_filters::Synchronizer<exact_sync_rgb_depth> synchronizer_rgb_depth;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::Image> exact_sync_pointcloud_rgb;
typedef message_filters::Synchronizer<exact_sync_pointcloud_rgb> synchronizer_pointcloud;

typedef pcl::PointXYZRGB point_XYZRGB;
typedef pcl::PointCloud<point_XYZRGB> point_cloud;
typedef point_cloud::Ptr point_cloud_ptr;


struct object
{
    float center_x;
    float center_y;
    float width;
    float height;
    float angle;

    float pcl_center_x;
    float pcl_center_y;
    float pcl_width;
    float pcl_height;
    float pcl_depth;
};


class itable_service
{
public:
    // Constructor
    itable_service();
    // Destructor
    ~itable_service();

    // Initialize "ROS things" - subscribe to topics etc.
    void ros_init();
    // Return reference to ROD NodeHandle
    ros::NodeHandle& get_NodeHandle() {return node_handle;}
    // Load calibration data from file
    void load_data_from_files();
    // Publish
    void publish_proj_cam();

private:

    //pcl::PointCloud<pcl::PointXYZ> PointCloud;
    pcl::PointCloud<pcl::Normal>::Ptr box_normals;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr box_features;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr box_search;

    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle { ros::NodeHandle("~") };

    // Calibration data of Camera
    cv::Mat cam_intrinsic, cam_dist_coeffs;

    // Calibration data for Projector-Camera system
    cv::Mat proj_cam_mat,dist_coeffs,rot_vec,trans_vec;
    itable_pkg::proj_cam_data proj_cam_msg;

    // Homography ~ marker data
    itable_pkg::marker_location marker_msg;
    cv::Mat marker_homography;
    cv::Mat marker_img;
    std::string marker_path;
    double marker_timer;
    double recalculate_marker_time { 4.0 };
    float marker_depth;
    float marker_offset;

    // Mask
    std::vector< std::vector<cv::Point> > convex_hulls;
    std::vector<cv::Point2f> mask_points;
    int mask_mode {1};// 0 mask + offest 1 static 2 auto
    float mask_offset { 200 };
    float min_mask_depth;
    float max_mask_depth;

    // Objects in point-cloud
    float min_cloud_depth;
    float max_cloud_depth;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box {new pcl::PointCloud<pcl::PointXYZ>};
    std::vector<object> objects;
    int object_mode {1}; // 0 mask + offest 1 static 2 auto
    float object_offset { 200 };
    double max_corr_dist {0.08};
    float table_depth;

    // Flags
    bool calculate_marker       {true};
    bool marker_loaded          {false};
    bool marker_found_valid     {false};

    bool calculate_mask         {true};

    bool calculate_object       {true};
    bool object_box_loaded      {false};

    bool cam_info_set           {false};

    // Launch arguments
    std::string package_dir_path;
    std::string topics_quality;

    // Subscribers & Exactime synchronizer
    image_subscriber depth_sub;
    image_subscriber rgb_sub;
    camerainfo_subscriber camerainfo_sub;
    pointcloud2_subscriber pointcloud_sub;

    synchronizer_rgb_depth* sync_rgb_depth;
    synchronizer_pointcloud* sync_pointcloud;

    // Publishers
    ros::Publisher proj_cam_data_pub;
    ros::Publisher marker_pub;
    ros::Publisher mask_pub;
    ros::Publisher objects_pub;

    // Callbacks
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud,const sensor_msgs::ImageConstPtr& msg_rgb);
    void image_callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);
    void caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo);

    // Functions
    void publish_marker();
    void publish_mask();
    void publish_objects();

    void find_marker(cv::Mat& rgb_img, cv::Mat& depth_img);
    void recalculate_mask( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask, cv::Mat rgb_img);
    void find_object_in_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,cv::Mat rgb_img);

    // Project pointcloud point to RGB space
    cv::Point2f project3D_to_pixel(cv::Point3f point3D);
    // Project point from 2D RGB space + depth to pointcloud point
    cv::Point3f backproject_pixel_to_3D( cv:: Point2f, float depth);
    void project3D_to_pixel( std::vector<cv::Point3d>& input_3D, std::vector<cv::Point2f>& output_2D );
    void backproject_pixel_to_3D( std::vector<cv::Point3f>& input, std::vector<cv::Point3f>& output);
};



} // namespace


#endif
