#include "itable_demo_node.h"


namespace itable
{

itable_demo::itable_demo()
{
    // Create triggers

    prague_trigger.setPosition(380,350);
    sf::Texture* prague_icon = new sf::Texture();
    if (!prague_icon->loadFromFile("/home/petr/catkin_ws/src/itable_demo/data/maps/prague_icon.png"))
    {
        ROS_ERROR("Cannot load image file ");
    }
    prague_icon->setSmooth(true);
    prague_trigger.setTexture(*prague_icon);


    brno_trigger.setPosition( 780,730);

    sf::Texture* brno_icon = new sf::Texture();
    if (!brno_icon->loadFromFile("/home/petr/catkin_ws/src/itable_demo/data/maps/brno_icon.png"))
    {
        ROS_ERROR("Cannot load image file ");
    }
    brno_icon->setSmooth(true);
    brno_trigger.setTexture(*brno_icon);

    trig_prague = new Trigger(&window, "/home/petr/catkin_ws/src/itable_demo/data/square.png", &prague_trigger );
    trig_brno   = new Trigger(&window, "/home/petr/catkin_ws/src/itable_demo/data/square.png", &brno_trigger );

    // movies load and positioning
    if (!movie_prague.openFromFile("/home/petr/catkin_ws/src/itable_demo/data/prague.mp4"))
        {
            std::cout <<"neporadilo se nahrat video praha" << std::endl;
        }

    float tmp = 1920.0 / win_width;
    float pos_y = win_height - 1080 * tmp;
    movie_prague.fit(0, pos_y / 2.0, win_width, 1080 * tmp);

    if (!movie_brno.openFromFile("/home/petr/catkin_ws/src/itable_demo/data/brno.mp4"))
    {
        std::cout <<"neporadilo se nahrat video brno" << std::endl;
    }
    movie_brno.fit(0, pos_y / 2.0, win_width, 1080 * tmp);

    // Font etc
    if (!font.loadFromFile("/home/petr/catkin_ws/src/itable_demo/data/arial.ttf"))
    {
        ROS_ERROR("Could not load font");
    }
    // set font
    text_brno.setFont(font);

    // set the character size
    text_brno.setCharacterSize(28); // in pixels, not points!

    // set the color
    text_brno.setColor(sf::Color::White);

    // set font
    text_prague.setFont(font);

    // set the character size
    text_prague.setCharacterSize(28); // in pixels, not points!

    // set the color
    text_prague.setColor(sf::Color::White);
    text_prague.setString(L"Praha je hlavní a současně největší město České republiky a 15. největší město Evropské unie.\nLeží mírně na sever od středu Čech na řece Vltavě, uvnitř Středočeského kraje, jehož je \nsprávním centrem, ale jako samostatný kraj není jeho součástí. Je sídlem velké části státních \ninstitucí a množství dalších organizací a firem. Sídlí zde prezident republiky, parlament, vláda,\n ústřední státní orgány a jeden ze dvou vrchních soudů.");


    // create quiz
    questions.push_back( question("Krusne hory", sf::Vector2f(577,302)) );
    questions.push_back( question("Cerny les", sf::Vector2f(400,885)) );
    questions.push_back( question("Sumava", sf::Vector2f(719,1174)) );
    questions.push_back( question("Krkonose", sf::Vector2f(1457,220)) );
    questions.push_back( question("Orlicke hory", sf::Vector2f(1700,440)) );
    questions.push_back( question("Moravskoslezske Beskydy", sf::Vector2f(2400,900)) );

}

void itable_demo::ros_init()
{
    mask_sub    = node_handle.subscribe("/mask_data", 1, &itable_demo::mask_callback,this);
    marker_sub  = node_handle.subscribe("/marker_data",1, &itable_demo::marker_callback,this);
    object_sub  = node_handle.subscribe("/objects_data",1, &itable_demo::object_callback,this);
    icon_sub    = node_handle.subscribe("/ar_pose_marker", 1, &itable_demo::icon_callback,this);

}

void itable_demo::icon_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& icon)
{
    ROS_INFO_ONCE("Icon callback");
    //if ( trig_prague->is_ticking() || trig_brno->is_ticking() )
    {
        for ( auto it = icon->markers.begin(); it != icon->markers.end() ; it++)
        {
            last_icon_id = it->id;
        }
    }


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

        obj.x         = msg.objects[i].center_x;
        obj.y         = msg.objects[i].center_y;
        obj.width     = msg.objects[i].width;
        obj.height    = msg.objects[i].height;
        obj.angle     = msg.objects[i].angle;

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

void itable_demo::create_window(std::string window_name, bool fullscreen )
{
    if ( fullscreen )
        window =  new sf::RenderWindow(sf::VideoMode(win_width,win_height),window_name,sf::Style::Fullscreen);
    else
        window = new sf::RenderWindow(sf::VideoMode(win_width,win_height),window_name,sf::Style::Resize);

    window->setVerticalSyncEnabled(true);
}

void itable_demo::load_data()
{
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/brno.png");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/praha.jpg");
    img_files.push_back("/home/petr/catkin_ws/src/itable_demo/data/maps/map_CR.png");

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

    sf::Vector2f targetSize(win_width, win_height);

    brno.setScale(   targetSize.x / brno.getLocalBounds().width,   targetSize.y / brno.getLocalBounds().height );
    prague.setScale( targetSize.x / prague.getLocalBounds().width, targetSize.y / prague.getLocalBounds().height);
    map_CR.setScale( targetSize.x / map_CR.getLocalBounds().width, targetSize.y / map_CR.getLocalBounds().height);

    int offset_x,offset_y;
    int map_width = map_CR.getGlobalBounds().width;
    int map_height = map_CR.getGlobalBounds().height;
    offset_x = win_width - map_width;
    offset_y = win_height - map_height;
    map_CR.setPosition(offset_x / 2, offset_y / 2);


    // quiz
    if (!CR_mount.loadFromFile("/home/petr/catkin_ws/src/itable_demo/data/maps/CR_hory.png"))
    {
        ROS_ERROR("Cannot load image file CR_hory");
    }
    CR_mount.setSmooth(true);
    quiz_map.setTexture(CR_mount);
    quiz_map.setScale( targetSize.x / map_CR.getLocalBounds().width, targetSize.y / map_CR.getLocalBounds().height );
    quiz_map.setPosition(0,250);
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
        rect.setFillColor(sf::Color::Black);
        window->draw( rect );
    }

}

void Trigger::draw_object(object &obj)
{
    sf::RectangleShape rect( sf::Vector2f(obj.width,obj.height) );
    rect.setOrigin(obj.width / 2.0, obj.height / 2.0);
    rect.setRotation(obj.angle);
    rect.setPosition( obj.x, obj.y );
    rect.setFillColor(sf::Color::Black);
    (*window)->draw( rect );
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
        window->draw( *(trig_prague->draw()) );
        window->draw( *(trig_brno->draw()));


        if ( !objects.empty() )
        {
            if ( trig_prague->update( objects[0]) )
            {
                //std::string next_state = trig_prague->getIcon();
                if ( last_icon_id == 1 )
                    game_state = s_prague_hist;
                else if (last_icon_id == 0)
                    game_state = s_prague_movie;
                else
                    game_state = s_prague_hist;
            }
            else if ( trig_brno->update( objects[0]) )
            {
                //std::string next_state = trig_brno->getIcon();
                if ( last_icon_id == 1)
                    game_state = s_brno_hist;
                else if (last_icon_id == 0)
                    game_state = s_brno_movie;
                else
                    game_state = s_brno_movie;
            }
            //else
            //    last_icon_id = -1;
        }

        break;

    case s_quiz:
    {
        if ( questions.empty() )
            questions = answered_q;

        int index = rand() % questions.size();
        actual_q = questions[index];

        game_state = s_asked;

        break;
    }

    case s_asked:



        break;


    case s_prague_movie:

        if ( movie_prague.getStatus() != sfe::Status::Playing )
            movie_prague.play();
        movie_prague.update();
        window->draw(movie_prague);
        if ( !objects.empty() )
        {
            if ( last_icon_id == 2)
            {
                movie_prague.pause();
                game_state = s_init;
            }
        }
        break;

    case s_prague_hist:
        window->draw(prague);
        window->draw(text_prague);
        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 )
            {
                game_state = s_init;
            }
        }
        break;

    case s_brno_movie:
        if ( movie_brno.getStatus() != sfe::Status::Playing )
            movie_brno.play();
        movie_brno.update();
        window->draw(movie_brno);

        if ( !objects.empty() )
        {
            if ( last_icon_id == 2)
            {
                movie_brno.pause();
                game_state = s_init;
            }
        }

        break;

    case s_brno_hist:
        window->draw(brno);
        if ( !objects.empty() )
        {
            if ( last_icon_id == 2)
            {
                game_state = s_init;
            }
        }
        break;

    default:
        ROS_ERROR("Game state unknown");
        break;
    }
    draw_objects();
}

Trigger::Trigger(sf::RenderWindow** win,std::string file_path, sf::Sprite* cs)
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
            for ( int &i : icon_count)
            {
                icon_count[i] = 0;
            }
        }
        else
        {
            //draw_object(obj);
            sprite->setScale(   (obj.width ) / sprite->getLocalBounds().width,   (obj.height) / sprite->getLocalBounds().height );
            if ( timer.getElapsedTime().asSeconds() < 0.5f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) - obj.height);
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.0f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) - obj.height);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x +  (obj.width / 2.0), obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.5f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) - obj.height);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x +  (obj.width / 2.0), obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y + (obj.height / 2.0)  );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 2.0f )
            {
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y - (obj.height / 2.0) - obj.height);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x +  (obj.width / 2.0), obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x - (obj.width / 2.0) , obj.y + (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x -  (obj.width / 2.0) - obj.width, obj.y - (obj.height / 2.0) );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() > 2.0f )
            {
                int max = 0,max_index = 4;
                for ( int &i : icon_count)
                {
                    if ( icon_count[i] > max )
                    {
                        max = icon_count[i];
                        max_index = i;
                    }
                }

                switch ( max_index )
                {
                case 0:
                    icon_name = "history";
                    break;
                case 1:
                    icon_name = "city";
                    break;
                case 2:
                    icon_name = "movie";
                    break;
                case 3:
                    icon_name = "cross";
                    break;
                case 4:
                    icon_name = "";
                    break;
                default:break;
                }

                ticking = false;
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
    demo.create_window("Game window",false);
    //demo.window = new sf::RenderWindow(sf::VideoMode(960,540),"temp",sf::Style::Resize);
    demo.load_data();


    itable::object obj;
    obj.width = 30;
    obj.height = 30;
    obj.x = 300;
    obj.y = 300;
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
