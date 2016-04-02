#include "itable_demo_node.h"


namespace itable
{

itable_demo::itable_demo()
{
    // Create triggers
    prague_trigger = sf::CircleShape(40.f);
    prague_trigger.setPosition(425,250);
    prague_trigger.setFillColor( sf::Color::Transparent);
    prague_trigger.setOutlineThickness(5.f);
    prague_trigger.setOutlineColor( sf::Color::Red );

    brno_trigger = sf::CircleShape(40.f);
    brno_trigger.setPosition( 830,500);
    brno_trigger.setFillColor( sf::Color::Transparent);
    brno_trigger.setOutlineThickness(5.f);
    brno_trigger.setOutlineColor( sf::Color::Red );

    trig = new Trigger(&window, "/home/petr/catkin_ws/src/itable_demo/data/square.png", &prague_trigger );

}

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

    object temp;
    if ( !objects.empty() )
        temp = objects[0];

    objects.resize( msg.objects.size() );
    for ( int i = 0; i < msg.objects.size(); i++ )
    {
        object obj;

        obj.x      = msg.objects[i].center_x;
        obj.y      = msg.objects[i].center_y;
        obj.width  = msg.objects[i].width;
        obj.height = msg.objects[i].height;
        obj.angle  = msg.objects[i].angle;

        std::cout << obj.x << " " << obj.y << std::endl;

        if ( abs(temp.x - obj.x) > 30 && abs(temp.y - obj.y) > 30 )
        {
            std::cout << "change a lot" << std::endl;
            if ( changed_alot )
            {
                objects[i] = obj;
                changed_alot = false;
            }
            else
            {
                objects[i] = temp;
                changed_alot = true;
            }
        }
        else
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
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/brno.png");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/praha.jpg");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/CR.png");

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
        else if (event.type == sf::Event::KeyPressed)
        {
            if (event.key.code == sf::Keyboard::Escape)
            {
                window->close();
                demo_running = false;
            }
            if ( event.key.code == sf::Keyboard::Left)
            {
                objects[0].x -= 5;
            }
            if ( event.key.code == sf::Keyboard::Right)
            {
                objects[0].x += 5;
            }
            if ( event.key.code == sf::Keyboard::Down)
            {
                objects[0].y += 5;
            }
            if ( event.key.code == sf::Keyboard::Up)
            {
                objects[0].y -= 5;
            }
        }
    }
}


void itable_demo::game()
{
    switch( game_state )
    {
    case s_init:
        window->draw(map_CR);
        window->draw( *(trig->draw()) );
        window->draw(brno_trigger);

        draw_objects();
        if ( !objects.empty() )
        {
            if ( trig->update( objects[0]) )
                game_state = s_prague;

            /*
            sf::FloatRect pragueBB = prague_trigger.getGlobalBounds();
            sf::FloatRect brnoBB   = brno_trigger.getGlobalBounds();

            // check collision with a point
            sf::Vector2f object_center( objects[0].x, objects[0].y);
            if (pragueBB.contains(object_center))
            {
                std::cout << "PRAGUE" << std::endl;
                game_state = s_prague;
            }
            else if (  brnoBB.contains( object_center) )
            {
                std::cout << "BRNO" << std::endl;
                game_state = s_brno;
            }
            */
        }

        break;

    case s_prague:
        window->draw(prague);


        break;

    case s_brno:
        window->draw(brno);


        break;

    default:
        ROS_ERROR("Game state unknown");
        break;
    }

}

Trigger::Trigger(sf::RenderWindow** win,std::string file_path, sf::CircleShape* cs)
{
    window = win;
    file = file_path;
    sprite = new sf::Sprite();

    tex = new sf::Texture();
    if (!tex->loadFromFile(file.c_str()))
    {
        ROS_ERROR("Trigger: Cannot load image file %s", file.c_str()  );
    }
    tex->setSmooth(true);
    sprite->setTexture(*tex);
    circles = cs;
}

bool Trigger::update( object obj )
{
    sf::FloatRect fr = circles->getGlobalBounds();

    sf::Vector2f position(obj.x,obj.y);
    // check collision with a point
    if ( fr.contains(position) )
    {
        if ( ticking == false )
        {
            timer.restart();
            ticking = true;
        }
        else
        {
            sprite->setScale(   (obj.width / 2.0) / sprite->getLocalBounds().width,   (obj.height / 2.0) / sprite->getLocalBounds().height );
            if ( timer.getElapsedTime().asSeconds() < 0.5f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.0f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.5f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y  );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 2.0f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y  );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() > 2.0f )
            {
                return true;
            }

        }

    }
    else if ( ticking )
    {
        ticking = false;
    }
    return false;
}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "itable_demo");
    itable::itable_demo demo;

    demo.ros_init();
    demo.create_window(1280,1024,"Game window",false);
    //demo.window = new sf::RenderWindow(sf::VideoMode(960,540),"temp",sf::Style::Resize);
    demo.load_data();


    itable::object obj;
    obj.width = 30;
    obj.height = 30;
    obj.x = 100;
    obj.y = 100;
    obj.angle = 0;

    demo.objects.push_back(obj);

    while ( ros::ok() && demo.running() )
    {
        demo.events();
        demo.clear_window();

        demo.game();
        //demo.draw_objects();
        demo.draw_mask();
        demo.display();

        ros::spinOnce();
    }


    return 0;
}
