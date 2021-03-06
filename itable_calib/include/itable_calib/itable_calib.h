/*
 * ROS node for kinect2 - projector calibration
 * Author: Petr Svoboda
 * Date: 1.5.2016
 */

#ifndef _ITABLE_CALIB_
#define _ITABLE_CALIB_

#include <ros/ros.h>
#include <string>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>


namespace itable_calib
{


typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> tExactPolicy;
typedef message_filters::Synchronizer<tExactPolicy> tExactSync;

class iTable_calibration
{
    // Chessboard size
    cv::Size pattern_size;
    // Projector resolution
    cv::Size projector_resolution;
    // Vector of RGB 2D + depth points
    std::vector< cv::Point3f> collection_camera_plus_depth_points;
    // Vector of 2D points in SDL screen
    std::vector< cv::Point2f> collection_SDL_screen_points;

    // Calibration data of Camera
    cv::Mat cam_intrinsic, cam_dist_coeffs;

    // Counter of couples of found chessboards in SDL screen and pointcloud
    int pairs_counter;
    double delay;

    // Flags
    bool cam_info_set;
    bool find_next_chessboard;
    bool pair_found;
    bool end_calibration;
    // Pointer to SDL screen data
    SDL_Surface* screen;


public:

    iTable_calibration(ros::NodeHandle nh);
    void Set_SDL_Surface_pointer(SDL_Surface* screen_pointer) { screen = screen_pointer;}
    void Start_capturing() { find_next_chessboard = true;}
    bool Found_pair() { bool temp = pair_found; if ( pair_found ) pair_found = false; return temp;}
    void End_calibration() { end_calibration = true;}
    void Set_chessboard_size( cv::Size ps) { pattern_size = ps;}
    void Set_projector_resolution( cv::Size pr) { projector_resolution = pr;}
    double Get_delay() { return delay;}
    cv::Size Get_projector_size() { return projector_resolution;}
    std::string Get_package_dir_path() { return output_path;}

private:

    void ros_init();

    // RGB and depth data callback
    void image_cb(const sensor_msgs::ImageConstPtr& msg_rgb,
                        const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::CameraInfoConstPtr& msg_camerainfo);

    // CameraInfo callback
    void caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo);


    // subscribers for kinect color and depth images
    message_filters::Subscriber<sensor_msgs::CameraInfo> kinect_caminfo_sub;
    message_filters::Subscriber<sensor_msgs::Image> kinect_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub;

    // exact sync policy
    boost::shared_ptr<tExactSync> exact_sync_;

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_handle;

    // topics strings
    std::string rgb_topic;
    std::string depth_topic;
    std::string caminfo_topic;
    std::string output_path;
};

} // namespace

#endif
