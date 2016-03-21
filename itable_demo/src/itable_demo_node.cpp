#include "itable_demo_node.h"


namespace itable
{

void itable_demo::ros_init()
{
    mask_sub    = node_handle.subscribe("/mask_data", 1, &itable_demo::mask_callback,this);
    marker_sub  = node_handle.subscribe("/marker_data",1, &itable_demo::marker_callback,this);
    object_sub  = node_handle.subscribe("/objects_data",1, &itable_demo::object_callback,this);

}

void itable_demo::mask_callback(const itable_pkg::mask& msg)
{
    ROS_INFO_ONCE ( "Mask callback called");

    masks.resize( msg.mask.size() );

    for( int i = 0; i < msg.mask.size() ;i++)
    {
        sf::ConvexShape cs;
        cs.setPointCount( msg.mask[i].hull.size() );
        for ( int j = 0; j < msg.mask[i].hull.size(); j++)
        {
            cs.setPoint( j , sf::Vector2f(msg.mask[i].hull[j].x, msg.mask[i].hull[j].y));
        }
        cs.setFillColor(sf::Color::Black);
        masks[i] = cs;
    }
    ROS_INFO("Mask recalculated");
}

void itable_demo::marker_callback(const itable_pkg::marker_location& msg)
{
    //ROS_INFO ( "marker callback ");

}

void itable_demo::object_callback(const itable_pkg::objects& msg)
{
    ROS_INFO_ONCE ( "Object callback called");

    objects.resize( msg.objects.size() );
    for ( int i = 0; i < msg.objects.size(); i++ )
    {
        object obj;

        obj.x      = msg.objects[i].center_x;
        obj.y      = msg.objects[i].center_y;
        obj.width  = msg.objects[i].width;
        obj.height = msg.objects[i].height;
        obj.angle  = msg.objects[i].angle;

        objects[i] = obj;
    }

    ROS_INFO("Objects data updated");
}

void itable_demo::create_window(int width, int height, std::string window_name, bool fullscreen )
{
    if ( fullscreen )
        window =  new sf::RenderWindow(sf::VideoMode(width,height),window_name,sf::Style::Fullscreen);
    else
        window = new sf::RenderWindow(sf::VideoMode(width,height),window_name,sf::Style::Resize);
}

void itable_demo::load_data()
{
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/brno.jpg");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/praha.jpg");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/CR.jpg");

    for ( std::vector< std::string >::iterator it = img_files.begin(); it != img_files.end(); it++)
    {
        sf::Texture* txt = new sf::Texture();
        if (!txt->loadFromFile(it->c_str()))
        {
            ROS_ERROR("Cannot load image file %s", it->c_str()  );
        }
        txt->setSmooth(true);
        textures.push_back( txt );
    }

    brno.setTexture(*(textures[0]));
    prague.setTexture(*(textures[1]));
    map_CR.setTexture(*(textures[2]));

    sf::Vector2f targetSize(1280.0, 800.0);

    brno.setScale(   targetSize.x / brno.getLocalBounds().width,   targetSize.y / brno.getLocalBounds().height );
    prague.setScale( targetSize.x / prague.getLocalBounds().width, targetSize.y / prague.getLocalBounds().height);
    map_CR.setScale( targetSize.x / map_CR.getLocalBounds().width, targetSize.y / map_CR.getLocalBounds().height);
}

void itable_demo::draw_mask()
{
    for ( std::vector<sf::ConvexShape>::iterator it = masks.begin(); it != masks.end(); it++)
        window->draw(*it);
}

void itable_demo::draw_objects()
{
    for ( std::vector<object>::iterator it = objects.begin(); it != objects.end() ; it++)
    {
        sf::RectangleShape rect( sf::Vector2f(it->width,it->height) );
        rect.setOrigin(it->width / 2.0, it->height / 2.0);
        rect.setRotation(it->angle);
        rect.setPosition( it->x, it->y );
        rect.setFillColor(sf::Color::Red);
        window->draw( rect );
    }

}

void itable_demo::events()
{

    sf::Event event;
    while (window->pollEvent(event))
    {
                // "close requested" event: we close the window
        if (event.type == sf::Event::Closed)
            window->close();
    }
}


void itable_demo::game()
{
    switch( game_state )
    {
    case s_init:



        break;

    case s_prague:



        break;

    case s_brno:



        break;

    default:
        ROS_ERROR("Game state unknown");
        break;
    }

}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "itable_demo");
    itable::itable_demo demo;

    demo.ros_init();
    demo.create_window(960,540,"Game window",false);
    //demo.window = new sf::RenderWindow(sf::VideoMode(960,540),"temp",sf::Style::Resize);
    demo.load_data();


    while ( ros::ok())
    {
        demo.events();
        demo.clear_window();

        demo.draw_objects();
        demo.draw_mask();
        demo.display();

        ros::spinOnce();
    }


    return 0;
}
