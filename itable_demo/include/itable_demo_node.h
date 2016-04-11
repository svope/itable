#ifndef ITABLE_DEMO_NODE_H
#define ITABLE_DEMO_NODE_H


#include <iostream>


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

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include "sfeMovie/Movie.hpp"

namespace itable
{


struct object
{
    float x;
    float y;
    float width;
    float height;
    float angle;
    std::string icon_name;
};

class Trigger
{
public:
    Trigger(sf::RenderWindow** win,std::string file_path, sf::Sprite* cs);
    bool update( object obj );
    sf::Sprite* draw() { return circles; }
    std::string getIcon( ) { return icon_name;}
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

    void mask_callback( const itable_pkg::mask& msg);
    void marker_callback( const itable_pkg::marker_location& msg);
    void object_callback( const itable_pkg::objects& msg);

    // SFML
    sf::RenderWindow* window;

    // Mask
    std::vector< sf::ConvexShape > masks;

    // Object
public:
    std::vector< object > objects;
    bool changed_alot { false };

    // game states
    enum game_states { s_init, s_prague, s_brno_hist, s_brno_movie, s_prague_hist, s_prague_movie };
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


    // Timer
    bool ticking { false };
    sf::Clock timer;
};


} //namespace
#endif
