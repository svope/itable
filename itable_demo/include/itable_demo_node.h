#ifndef ITABLE_DEMO_NODE_H
#define ITABLE_DEMO_NODE_H


#include <iostream>
#include <cstdlib>

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

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

#include "sfeMovie/Movie.hpp"
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> camerainfo_subscriber;

namespace itable
{


struct object
{
    float x;
    float y;
    float width;
    float height;
    float angle;
    float depth;
};

struct question
{
    sf::Vector2f pos;
    std::wstring  mount;
    question(std::wstring s, sf::Vector2f v) : mount(s), pos(v){}
    question(){}
};

class Trigger
{
public:
    Trigger(sf::RenderWindow** win,std::string file_path, sf::Sprite* cs);
    bool update( object obj );
    sf::Sprite* draw() { return circles; }
    std::string getIcon( ) { return icon_name;}
    bool is_ticking() { return ticking;}
private:
    std::string icon_name;
    std::string file;
    sf::Texture* tex;
    sf::Sprite*  sprite;
    sf::Sprite* circles;
    sf::RenderWindow** window;
    sf::Clock timer;
    // 0 hist,1 city, 2 movie, 3 cross, 4 null
    int icon_count[5] {0,0,0,0,0};
    bool ticking {false};
    void draw_object(object &obj);
};

class itable_demo
{

public:
    // Initialize "ROS things" - subscribe to topics etc.
    void ros_init();


    itable_demo();
    bool running(){ return demo_running;}
    void create_window(std::string window_name, bool fullscreen = true);
    void load_data();
    void display(){window->display();}
    void clear_window(){ window->clear(sf::Color::Black);}
    void draw_mask();
    void draw_objects();
    void events();
    void game();

private:

    //ROS
    ros::NodeHandle node_handle;

    ros::Subscriber mask_sub;
    ros::Subscriber object_sub;
    ros::Subscriber marker_sub;
    ros::Subscriber projector_camera_sub;
    ros::Subscriber icon_sub;
    camerainfo_subscriber camerainfo_sub;

    void mask_callback( const itable_pkg::mask& msg);
    void marker_callback( const itable_pkg::marker_location& msg);
    void object_callback( const itable_pkg::objects& msg);
    void icon_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& icon);
    void proj_cam_callback(const itable_pkg::proj_cam_data& msg);
    void caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo);
    cv::Point3f backproject_pixel_to_3D( cv:: Point2f cam_2D, float depth);
    cv::Point2f bitmap_to_projector( int x, int y);
    bool icon_valid() { return icon_timer.getElapsedTime().asSeconds() < 0.4f ? true : false;}
    bool object_valid() { return object_timer.getElapsedTime().asSeconds() < 0.2f ? true : false;}

    // SFML
    sf::RenderWindow* window;

    // Mask
    std::vector< sf::ConvexShape > masks;

    // Homo
    cv::Mat homography {cv::Mat(3,3, CV_32F) };
    bool homo_valid {false};
    float homo_depth;

    // proj cam
    cv::Mat intrinsic {cv::Mat(3,3, CV_64F) };
    cv::Mat dist { cv::Mat(5,1, CV_64F)};
    cv::Mat rot {cv::Mat(3,1, CV_64F) };
    cv::Mat trans {cv::Mat(3,1, CV_64F) };
    bool proj_cam_set { false };

    // cam info
    bool cam_info_set {false };
    cv::Mat cam_intrinsic;

public:
    // Object
    std::vector< object > objects;
    int last_icon_id { -1 };
    bool changed_alot { false };
    sf::Clock object_timer;
    float table_depth;

    // icon timer
    sf::Clock icon_timer;

    // game states
    enum game_states { s_init, s_prague, s_brno_hist, s_brno_movie, s_prague_hist, s_prague_movie , s_quiz, s_asked,s_answered,s_not_answered, s_map_search};
    game_states game_state { s_init };
    bool demo_running { true };

    // Images to load
    std::vector< std::string > img_files;
    std::vector< sf::Texture* > textures;
    sf::Sprite map_CR;
    sf::Sprite prague,brno;

    // Movies to load
    sfe::Movie movie_brno;
    sfe::Movie movie_prague;

    // Triggers
    itable::Trigger* trig_prague;
    itable::Trigger* trig_brno;
    sf::Sprite prague_trigger;
    sf::Sprite brno_trigger;

    // Window res
    int win_width {1280};
    int win_height{1024};

    // Font
    sf::Font font;
    sf::Text text_brno;
    sf::Text text_prague;

    // Quiz
    sf::ConvexShape paper_map;
    sf::Texture panorama_tex;
    sf::Sprite panorama;
    sf::Texture CR_mount;
    sf::Sprite quiz_map;
    sf::Text quiz_text;
    question actual_q;
    sf::Sprite sprite;
    sf::Texture sprite_texture;
    sf::Clock timeout;
    bool timeout_tick { false };
    sf::Vector2f last_mount;
    sf::Vector2f last_map_position {sf::Vector2f(-100,-100)};

    sf::SoundBuffer succ,fail;
    sf::Sound sound;

    sf::Clock safe_time;
    bool safe_time_done { false};
    float last_obj_x;
    float last_obj_y;

    std::vector< question > questions;
    std::vector< question > answered_q;

    // Timer
    bool ticking { false };
    sf::Clock timer;
};


} //namespace
#endif
