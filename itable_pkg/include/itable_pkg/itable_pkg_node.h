#ifndef _ITABLE_PKG_NODE
#define _ITABLE_PKG_NODE

#include "ros/ros.h"

#include "itable_pkg/marker_location.h"
#include "itable_pkg/object.h"
#include "itable_pkg/objects.h"
#include "itable_pkg/proj_cam_data.h"
#include "itable_pkg/mask.h"
#include "itable_pkg/convex_hull.h"

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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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
    float bounding_box[4];
    float rgb[3];
    float depth;
    int type;
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
    void publish_all();

private:

    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle { ros::NodeHandle("~") };

    // Calibration data for Projector-Camera system
    cv::Mat proj_cam_mat,dist_coeffs,rot_vec,trans_vec;
    itable_pkg::proj_cam_data proj_cam_msg;

    // Homography ~ marker data
    itable_pkg::marker_location marker_msg;
    cv::Mat marker_homography;
    cv::Mat marker_img;
    std::string marker_path;
    bool marker_loaded {false};
    float marker_depth;

    // Mask
    itable_pkg::mask mask_msg;
    std::vector<int> convex_hull_points;
    bool recalculate_mask {false};

    // Objects in point-cloud
    std::vector<object> objects;

    // Flags
    bool recalculate_homography { false };

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
    ros::Publisher homography_pub;
    ros::Publisher mask_pub;
    ros::Publisher objects_pub;

    // Callbacks
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_pointcloud,const sensor_msgs::ImageConstPtr& msg_rgb);
    void image_callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);
    void caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo);

    // Functions
    void publish_proj_cam();
    void publish_homography();
    void publish_mask();
    void publish_objects();
    void find_homography(cv::Mat& rgb_img);

};



}


#endif
