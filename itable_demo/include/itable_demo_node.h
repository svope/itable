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


namespace itable
{


struct object
{
    float x;
    float y;
    float width;
    float height;
    float angle;
};

class Trigger
{
public:
    Trigger(sf::RenderWindow** win,std::string file_path, sf::CircleShape* cs);
    bool update( object obj );
    sf::Shape* draw() { return circles; }
private:
    std::string file;
    sf::Texture* tex;
    sf::Sprite*  sprite;
    sf::Shape* circles;
    sf::RenderWindow** window;
    sf::Clock timer;
    bool ticking {false};
};

class itable_demo
{
public:
    // Initialize "ROS things" - subscribe to topics etc.
    void ros_init();


    itable_demo();
    bool running(){ return demo_running;}
    void create_window(int width, int height, std::string window_name, bool fullscreen = true);
    void load_data();
    void display(){window->display();}
    void clear_window(){ window->clear(sf::Color::White);}
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
    enum game_states { s_init, s_prague, s_brno };
    game_states game_state { s_init };
    bool demo_running { true };

    // Images to load
    std::vector< std::string > img_files;
    std::vector< sf::Texture* > textures;
    sf::Sprite map_CR;
    sf::Sprite prague,brno;

    // Triggers
    itable::Trigger* trig;
    sf::CircleShape prague_trigger;
    sf::CircleShape brno_trigger;

    // Timer
    bool ticking { false };
    sf::Clock timer;
};


} //namespace
#endif
