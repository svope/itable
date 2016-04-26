/*
 * Developed by dcgm-robotics@FIT group
 * Author: Michal Kapinus
 * Date: 01.04.2012 (version 0.1)
 *
 * License: BUT OPEN SOURCE LICENSE (http://www.fit.vutbr.cz/~lampa/ipv6/LICENSE)
 *-------------------------------------------------------------------------------
 */

#pragma once
#ifndef _ITABLE_CHESSFINDER_
#define _ITABLE_CHESSFINDER_

#include <ros/ros.h>
#include <string>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
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


namespace itable_chessfinder {

typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> tExactPolicy;
typedef message_filters::Synchronizer<tExactPolicy> tExactSync;


class iTable_chessfinder {

    cv::Size pattern_size;
    cv::Mat cameraMatrix,distCoeffs,tvec,rvec,camColor;
    cv::Mat SDL_screen;


public:

    SDL_Surface* screen;
    std::vector< cv::Point3f > objectPoints;
    std::vector< cv::Point2f > imagePoints;
    bool capture;




    iTable_chessfinder(ros::NodeHandle nh);

private:

    void ros_init();

    // Calibration data of Camera
    cv::Mat cam_intrinsic, cam_dist_coeffs;
    bool cam_info_set;


    void image_cb(const sensor_msgs::ImageConstPtr& msg_rgb,
                        const sensor_msgs::ImageConstPtr& msg_depth,const sensor_msgs::CameraInfo& msg_camerainfo);

    void caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo);

    // image transport instance for subscribing image data
    boost::shared_ptr<image_transport::ImageTransport> it_;

    // subscribers for kinect color and depth images
    message_filters::Subscriber<sensor_msgs::CameraInfo> kinect_caminfo_sub;
    message_filters::Subscriber<sensor_msgs::Image> kinect_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub;

    // subscriber for point cloud
    ros::Subscriber kinect_pcd_sub_;

    // exact sync policy
    boost::shared_ptr<tExactSync> exact_sync_;

    // node handler
    ros::NodeHandle nh_;

};

} // namespace

#endif
