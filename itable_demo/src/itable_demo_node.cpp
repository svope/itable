#include "itable_demo_node.h"


namespace itable
{

itable_demo::itable_demo()
{
    // Create triggers

    prague_trigger.setPosition(380,350);
    sf::Texture* prague_icon = new sf::Texture();
    if (!prague_icon->loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/maps/prague_icon.png"))
    {
        ROS_ERROR("Cannot load image file ");
    }
    prague_icon->setSmooth(true);
    prague_trigger.setTexture(*prague_icon);


    brno_trigger.setPosition( 780,730);

    sf::Texture* brno_icon = new sf::Texture();
    if (!brno_icon->loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/maps/brno_icon.png"))
    {
        ROS_ERROR("Cannot load image file ");
    }
    brno_icon->setSmooth(true);
    brno_trigger.setTexture(*brno_icon);

    brno_trigger.setScale(1.5,1.5);
    prague_trigger.setScale(1.5,1.5);

    trig_prague = new Trigger(&window, "/home/artable/svoboda_ws/src/itable_demo/data/square.png", &prague_trigger );
    trig_brno   = new Trigger(&window, "/home/artable/svoboda_ws/src/itable_demo/data/square.png", &brno_trigger );

    // movies load and positioning
    if (!movie_prague.openFromFile("/home/artable/svoboda_ws/src/itable_demo/data/prague.mp4"))
        {
            std::cout <<"neporadilo se nahrat video praha" << std::endl;
        }

    float tmp = 1920.0 / win_width;
    float pos_y = win_height - 1080 * tmp;
    movie_prague.fit(0, pos_y / 2.0, win_width, 1080 * tmp);

    if (!movie_brno.openFromFile("/home/artable/svoboda_ws/src/itable_demo/data/brno.mp4"))
    {
        std::cout <<"neporadilo se nahrat video brno.mp4" << std::endl;
    }
    movie_brno.fit(0, pos_y / 2.0, win_width, 1080 * tmp);

    // Font etc
    if (!font.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/arial.ttf"))
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

    textB1.setFont(font);
    textB2.setFont(font);
    textB3.setFont(font);
    textP1.setFont(font);
    textP2.setFont(font);
    textP3.setFont(font);
    textP4.setFont(font);

    // create quiz
    questions.push_back( question(L"Krušné hory", sf::Vector2f(160,138)) );
    questions.push_back( question(L"Šumava", sf::Vector2f(230,390)) );
    questions.push_back( question(L"Krkonoše", sf::Vector2f(410,86)) );
    questions.push_back( question(L"Jeseníky", sf::Vector2f(661,195)) );
    questions.push_back( question(L"Moravskoslezské Beskydy", sf::Vector2f(800,309)) );




}

void itable_demo::ros_init()
{
    mask_sub    = node_handle.subscribe("/mask_data", 1, &itable_demo::mask_callback,this);
    marker_sub  = node_handle.subscribe("/marker_data",1, &itable_demo::marker_callback,this);
    object_sub  = node_handle.subscribe("/objects_data",1, &itable_demo::object_callback,this);
    icon_sub    = node_handle.subscribe("/ar_pose_marker", 1, &itable_demo::icon_callback,this);
    projector_camera_sub = node_handle.subscribe("/projector_camera_data",1,&itable_demo::proj_cam_callback,this);
    camerainfo_sub.subscribe(node_handle, "/kinect2/qhd/camera_info",1);
    camerainfo_sub.registerCallback(&itable_demo::caminfo_callback, this);

}

void itable_demo::proj_cam_callback(const itable_pkg::proj_cam_data& msg)
{
    ROS_INFO_ONCE("proj_cam_callback called");
    if ( proj_cam_set )
        return;
    else
    {
        double* ptrI = intrinsic.ptr<double>(0,0);
        for ( int i = 0; i < 9; i++, ptrI++)
        {
            *ptrI = msg.intrinsic[i];
        }

        double* ptrD = dist.ptr<double>(0,0);
        for ( int i = 0; i < 5; i++, ptrD++)
        {
            *ptrD = msg.distortion[i];
        }

        double* ptrR = rot.ptr<double>(0,0);
        for ( int i = 0; i < 3; i++, ptrR++)
        {
            *ptrR = msg.rotation[i];
        }

        double* ptrT = trans.ptr<double>(0,0);
        for ( int i = 0; i < 3; i++, ptrT++)
        {
            *ptrT = msg.translation[i];
        }
        proj_cam_set = true;

        //std::cout << intrinsic <<std::endl << dist <<std::endl << rot << std::endl << trans << std::endl;
    }

}

void itable_demo::icon_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& icon)
{
    ROS_INFO_ONCE("Icon callback");
    //if ( trig_prague->is_ticking() || trig_brno->is_ticking() )
    {
        for ( auto it = icon->markers.begin(); it != icon->markers.end() ; it++)
        {
            last_icon_id = it->id;
            icon_timer.restart();
        }

        //if ( icon->markers.empty() )
        //    last_icon_id = -1;
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
    //ROS_INFO("Mask recalculated");
}

void itable_demo::marker_callback(const itable_pkg::marker_location& msg)
{
    //ROS_INFO ( "marker callback ");
    if ( msg.valid && game_state == s_map_search )
    {
        float* ptrH = homography.ptr<float>(0,0);
        for ( int i = 0; i < 9; i++, ptrH++)
        {
            *ptrH = msg.homography[i];
        }
        homo_depth = msg.depth;
        homo_valid = true;
    }

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
        obj.depth     = msg.objects[i].pcl_depth * 1000.0f; // from pcl to mm

        table_depth = msg.table_depth;

        if ( abs( table_depth - obj.depth ) < 25 ) // it seems that object is on the table ~ is valid
        {
            object_timer.restart();
            objects[i] = obj;
            ROS_INFO("Object valid");
        }
        else
        {
            if ( object_valid() )
            {
                //objects.resize(1);
                objects[0] = temp;
            }
            else
            {
                objects.clear();
                ROS_INFO("No object found");
            }
        }

        //std::cout << obj.x << " " << obj.y << std::endl;
        /*
        if ( abs(temp.x - obj.x) > 100 && abs(temp.y - obj.y) > 100 )
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
        */
    }

    // If cannot localize object, the object is still valid for some time
    if ( msg.objects.empty() )
    {
        if ( object_valid() )
        {
            objects.resize(1);
            objects[0] = temp;
        }
        else
        {
            objects.clear();
            ROS_INFO("No object found");
        }
    }

    //ROS_INFO("Objects data updated");
}

void itable_demo::create_window(std::string window_name, bool fullscreen )
{
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
    if ( fullscreen )
        window =  new sf::RenderWindow(sf::VideoMode(win_width,win_height),window_name,sf::Style::Fullscreen);
    else
        window = new sf::RenderWindow(sf::VideoMode(win_width,win_height),window_name,sf::Style::Resize);

    window->setFramerateLimit(60);
}

void itable_demo::load_data()
{
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/maps/map_CR.png");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/brno/brno1.jpg");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/brno/brno2.jpg");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/brno/brno3.jpg");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/brno/brno4.jpg");

    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/praha/praha.jpg");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/praha/praha1.jpg");
    img_files.push_back("/home/artable/svoboda_ws/src/itable_demo/data/praha/praha2.jpg");

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

    map_CR.setTexture(*(textures[0]));
    brno1.setTexture(*(textures[1]));
    brno2.setTexture(*(textures[2]));
    brno3.setTexture(*(textures[3]));
    brno4.setTexture(*(textures[4]));

    prague.setTexture(*(textures[5]));
    prague1.setTexture(*(textures[6]));
    prague2.setTexture(*(textures[7]));


    sf::Vector2f targetSize(win_width, win_height);

    //brno1.setScale(   targetSize.x / brno1.getLocalBounds().width,   targetSize.y / brno1.getLocalBounds().height );
    //prague.setScale( targetSize.x / prague.getLocalBounds().width, targetSize.y / prague.getLocalBounds().height);
    map_CR.setScale( targetSize.x / map_CR.getLocalBounds().width, targetSize.y / map_CR.getLocalBounds().height);

    int offset_x,offset_y;
    int map_width = map_CR.getGlobalBounds().width;
    int map_height = map_CR.getGlobalBounds().height;
    offset_x = win_width - map_width;
    offset_y = win_height - map_height;
    map_CR.setPosition(offset_x / 2, offset_y / 2);


    // quiz
    if (!CR_mount.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/maps/CR_hory.png"))
    {
        ROS_ERROR("Cannot load image file CR_hory");
    }
    CR_mount.setSmooth(true);
    quiz_map.setTexture(CR_mount);
    quiz_map.setScale( targetSize.x / quiz_map.getLocalBounds().width, (targetSize.y - 100)/ quiz_map.getLocalBounds().height );
    quiz_map.setPosition(0,100);

    quiz_text.setFont(font);
    quiz_text.setCharacterSize(90); // in pixels, not points!
    quiz_text.setColor(sf::Color::White);

    if (!panorama_tex.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/maps/panorama.jpg"))
    {
        ROS_ERROR("Cannot load image panorama.jpg");
    }
    panorama_tex.setSmooth(true);
    panorama.setTexture(panorama_tex);
    panorama.setPosition(0,0);

    if ( !sprite_texture.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/square.png") )
    {
        ROS_ERROR("Cannot load sprite" );
    }
    sprite_texture.setSmooth(true);
    sprite.setTexture(sprite_texture);

    // sounds
    if (!succ.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/success.wav"))
    {
        ROS_ERROR("Cannot open success.wav file");
    }

    if (!fail.loadFromFile("/home/artable/svoboda_ws/src/itable_demo/data/fail.wav"))
    {
        ROS_ERROR("Cannot open fail.wav file");
    }
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
        rect.setFillColor( sf::Color (192, 192, 192, 200) );
        rect.setScale(1.2,1.5);
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


        if ( !objects.empty() && last_icon_id != 2 && icon_valid() ) // 2 ~ back
        {
            if ( trig_prague->update( objects[0]) )
            {
                //ROS_ERROR("TRUE");
                //std::cout<< "LAST ICON ID <<<< " << last_icon_id << " <<< "<<std::endl;
                if ( last_icon_id == 1 && icon_valid() )
                    game_state = s_prague_hist;
                else if ( last_icon_id == 0 && icon_valid() )
                    game_state = s_prague_movie;
            }
            else if ( trig_brno->update( objects[0]) )
            {
                //ROS_ERROR("TRUEBB");
                //std::cout<< "LAST ICON ID <<<< " << last_icon_id << " <<< "<<std::endl;
                if ( last_icon_id == 1 && icon_valid() )
                    game_state = s_brno_hist;
                else if ( last_icon_id == 0 && icon_valid() )
                    game_state = s_brno_movie;
            }

        }
        else
        {
            trig_brno->timer_restart();
            trig_prague->timer_restart();
        }

        if ( !objects.empty())
        {
            if ( last_icon_id == 5 && icon_valid())
            {
                homo_valid = false;
                game_state = s_quiz;
            }
        }
        break;

    case s_quiz:
    {
/*
        sf::CircleShape c( 10 );
        c.setOrigin(0,0);
        c.setFillColor(sf::Color::Red);
        cv::Point2f Krkonose = bitmap_to_projector( 579,379);
        c.setPosition( Krkonose.x, Krkonose.y);
        window->draw(c);
        break;
*/

        if ( questions.empty() )
            questions = answered_q;

        int index = rand() % questions.size();
        actual_q = questions[index];

        questions.erase( questions.begin() + index );
        answered_q.push_back( actual_q );


        if ( homo_valid )
            game_state = s_asked;
        else
            game_state = s_map_search;

        safe_time_done = false;

        break;
    }

    case s_map_search:

        window->clear();
        quiz_text.setString(L"Hledám mapu...\nChviličku strpení :)");
        quiz_text.setColor( sf::Color::Red );
        window->draw(quiz_text);

        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid())
            {
                game_state = s_init;
            }
        }

        if ( homo_valid )
            game_state = s_asked;

        break;

    case s_asked:
    {
        //window->draw(quiz_map);
        quiz_text.setColor(sf::Color::White);
        quiz_text.setPosition(0,0);
        quiz_text.setString(L"Polož Kostku na " + actual_q.mount);


        paper_map.setPointCount(4);
        cv::Point2f left_up_corner = bitmap_to_projector(0,0);
        cv::Point2f right_up_corner = bitmap_to_projector(860,0);
        cv::Point2f left_bot_corner = bitmap_to_projector(0,495);
        cv::Point2f right_bot_corner = bitmap_to_projector(860,495);
        //paper_map.setPosition(left_up_corner.x,left_up_corner.y);
        //paper_map.setSize( sf::Vector2f(abs(left_up_corner.x - right_up_corner.x), abs(left_up_corner.y - left_bot_corner.y) ) );

        window->draw(panorama);
        paper_map.setPoint(0, sf::Vector2f(left_up_corner.x, left_up_corner.y));
        paper_map.setPoint(1, sf::Vector2f(right_up_corner.x, right_up_corner.y));
        paper_map.setPoint(2, sf::Vector2f(right_bot_corner.x, right_bot_corner.y));
        paper_map.setPoint(3, sf::Vector2f(left_bot_corner.x, left_bot_corner.y));
        paper_map.setFillColor( sf::Color::Black);
        window->draw(paper_map);
        window->draw(quiz_text);

        if ( safe_time_done == false)
        {
            safe_time.restart();
            safe_time_done = true;
            ticking = false;
        }
        else
        {
            if ( safe_time.getElapsedTime().asSeconds() < 2.0f)
                break;
        }

        object obj;
        if (objects.empty())
        {
            ticking = false;
            break;
        }
        else
            obj = objects[0];

        sf::FloatRect bbox = paper_map.getGlobalBounds();
        if ( bbox.contains( sf::Vector2f( obj.x, obj.y) ) )
        {
            // the cube is still on the "previous question" position
            if ( abs(obj.x - last_map_position.x) < 20 || abs( obj.y - last_map_position.y) < 20
                 && last_map_position.x > 0 )
            {
                ticking = false;
                break;
            }
            // the cube is on the paper map, start timer
            if ( ticking == false )
            {
                timer.restart();
                ticking = true;
            }
            else
            {
                // stop the timer, the cube has moved
                if ( abs(obj.x - last_obj_x) > 20 || abs( obj.y - last_obj_y) > 20 )
                {
                    ticking = false;
                    last_obj_x = obj.x;
                    last_obj_y = obj.y;
                    break;
                }
            }

            // scaling sprite and render timer
            sprite.setScale(   (obj.width ) / sprite.getLocalBounds().width,   (obj.height) / sprite.getLocalBounds().height );
            sprite.setOrigin ( sprite.getLocalBounds().width / 2.0 , sprite.getLocalBounds().height / 2.0);
            sprite.setRotation(obj.angle);
            if ( timer.getElapsedTime().asSeconds() < 0.4f )
            {
                sprite.setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (window)->draw(sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 0.8f )
            {
                sprite.setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (window)->draw(sprite);
                sprite.setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (window)->draw(sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.2f )
            {
                sprite.setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (window)->draw(sprite);
                sprite.setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (window)->draw(sprite);
                sprite.setPosition( obj.x , obj.y + obj.height + (obj.height / 2.0) );
                (window)->draw(sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.6f )
            {
                sprite.setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (window)->draw(sprite);
                sprite.setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (window)->draw(sprite);
                sprite.setPosition( obj.x , obj.y + obj.height + (obj.height / 2.0) );
                (window)->draw(sprite);
                sprite.setPosition( obj.x - obj.width - (obj.width / 2.0) , obj.y );
                (window)->draw(sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() > 1.6f )
            {
                timeout_tick = false;
                if ( homo_valid )
                {
                    cv::Point2f projector_space = bitmap_to_projector(actual_q.pos.x, actual_q.pos.y);

                    sf::Vector2f mount(projector_space.x,projector_space.y);
                    last_mount = mount;

                    if ( abs( mount.x - obj.x ) < 100 && abs( mount.y - obj.y) < 100 )
                        game_state = s_answered;
                    else
                        game_state = s_not_answered;

                    last_map_position = sf::Vector2f(obj.x,obj.y);

                    safe_time_done = false;
                    break;
                }
            }
        }
        else
        {
            //last_map_position = sf::Vector2f( -100.0,-100.0);

            ticking = false;
        }


        last_obj_x = obj.x;
        last_obj_y = obj.y;

        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid() )
            {
                game_state = s_init;
                homo_valid = false;
            }
        }
    }
        break;

    case s_answered:
    {
        //window->draw(quiz_map);
        //quiz_text.setColor(sf::Color(255,128,0));
        quiz_text.setColor(sf::Color::Green);
        quiz_text.setString(L"Správně!!!");

        window->draw(panorama);
        window->draw(paper_map);
        window->draw(quiz_text);
        if ( timeout_tick == false )
        {
            sound.setBuffer(succ);
            sound.play();
            timeout_tick = true;
            timeout.restart();
        }
        else if ( timeout.getElapsedTime().asSeconds() > 2 )
        {
            timeout_tick = false;
            game_state = s_quiz;
        }


        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid() )
            {
                game_state = s_init;
                homo_valid = false;
            }
        }
    }
        break;

    case s_not_answered:
    {
        //window->draw(quiz_map);
        quiz_text.setColor(sf::Color::Red);
        quiz_text.setString(L"Špatně!!!");

        //window->draw(panorama);
        window->draw(paper_map);
        window->draw(quiz_text);

        sf::CircleShape c( 70 );
        c.setOrigin(70,70);
        c.setPosition( last_mount.x, last_mount.y );
        c.setFillColor(sf::Color::Green);
        window->draw( c );

        if ( timeout_tick == false )
        {
            sound.setBuffer(fail);
            sound.play();
            timeout_tick = true;
            timeout.restart();
        }
        else if ( timeout.getElapsedTime().asSeconds() > 2 )
        {
            timeout_tick = false;
            game_state = s_quiz;
        }

        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid() )
            {
                game_state = s_init;
                homo_valid = false;
            }
        }
    }
        break;

    case s_prague_movie:

        if ( movie_prague.getStatus() != sfe::Status::Playing )
            movie_prague.play();
        movie_prague.update();
        window->draw(movie_prague);
        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid() )
            {
                movie_prague.pause();
                game_state = s_init;
            }
        }
        break;

    case s_prague_hist:

        textP1.setString(L"Praha je hlavní a současně největší\nměsto České republiky");
        textP1.setCharacterSize( 35 );
        textP1.setColor( sf::Color::White );
        textP1.setPosition( 10,10);
        window->draw(textP1);

        textP2.setString(L"Žije v ní asi  1 300 000\nobyvatel");
        textP2.setCharacterSize( 35 );
        textP2.setColor( sf::Color::White );
        textP2.setPosition( 800,100);
        window->draw(textP2);


        textP3.setString(L"Historické jádro patří do seznamu\nsvětového dědictví UNESCO");
        textP3.setCharacterSize( 35 );
        textP3.setColor( sf::Color::White );
        textP3.setPosition( 20 ,550);
        window->draw(textP3);

        textP4.setString(L"Sídlí zde  prezident republiky,\nparlament, vláda, ústřední státní\norgány a jeden ze dvou vrchních\nsoudů");
        textP4.setCharacterSize( 35 );
        textP4.setColor( sf::Color::White );
        textP4.setPosition( 700 ,800);
        window->draw(textP4);

        prague2.setScale(0.7,0.7);
        prague2.setPosition( 0, 100);
        window->draw(prague2);

        prague1.setScale(0.9,0.9);
        prague1.setPosition(10, 700);
        window->draw(prague1);

        prague.setScale(0.30,0.30);
        prague.setPosition( 660, 320);
        window->draw(prague);



        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid())
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
            if ( last_icon_id == 2 && icon_valid() )
            {
                movie_brno.pause();
                game_state = s_init;
            }
        }

        break;

    case s_brno_hist:
        brno3.setScale( 0.23, 0.23);
        brno3.setPosition( 0,0);
        window->draw( brno3 );

        textB1.setString(L"Brno je největší město na Moravě\na bývalé hlavní město Moravy");
        textB1.setCharacterSize( 35 );
        textB1.setColor( sf::Color::White );
        textB1.setPosition( 700,15);
        window->draw(textB1);

        brno1.setScale( 0.6 , 0.6 );
        brno1.setPosition( 600, 180);
        window->draw(brno1);

        brno2.setScale( 0.65, 0.65);
        brno2.setPosition ( 850, 730);
        window->draw ( brno2);

        brno4.setScale( 0.58, 0.58);
        brno4.setPosition ( 0, 640);
        window->draw ( brno4);

        textB2.setString(L"V Brně sídlí celkem\n33 fakult \n13 různých univerzit\na vysokých škol");
        textB2.setCharacterSize( 35 );
        textB2.setColor( sf::Color::White );
        textB2.setPosition( 500,730);
        window->draw(textB2);

        textB3.setString(L"Brno je centrem soudní moci\nČeské republiky, stalo se totiž\nsídlem jak Ústavního soudu, tak\nNejvyššího soudu, Nejvyššího\nsprávního soudu i Nejvyššího\nstátního zastupitelství");
        textB3.setCharacterSize( 35 );
        textB3.setColor( sf::Color::White );
        textB3.setPosition( 30,330);
        window->draw(textB3);

        if ( !objects.empty() )
        {
            if ( last_icon_id == 2 && icon_valid() )
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
        }
        else
        {
            //std::cout << obj.width << "   " << obj.height << std::endl;
            //draw_object(obj);
            sprite->setScale(   (obj.width ) / sprite->getLocalBounds().width,   (obj.height) / sprite->getLocalBounds().height );
            sprite->setOrigin ( sprite->getLocalBounds().width / 2.0 , sprite->getLocalBounds().height / 2.0);
            sprite->setRotation(obj.angle);
            if ( timer.getElapsedTime().asSeconds() < 0.4f )
            {
                sprite->setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 0.8f )
            {
                sprite->setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.2f )
            {
                sprite->setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y + obj.height + (obj.height / 2.0) );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() < 1.6f )
            {
                sprite->setPosition( obj.x  , obj.y - obj.height - obj.height/2.0);
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x + obj.width + (obj.width / 2.0), obj.y  );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x , obj.y + obj.height + (obj.height / 2.0) );
                (*window)->draw(*sprite);
                sprite->setPosition( obj.x - obj.width - (obj.width / 2.0) , obj.y );
                (*window)->draw(*sprite);
            }
            else if ( timer.getElapsedTime().asSeconds() > 1.6f )
            {
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

cv::Point3f itable_demo::backproject_pixel_to_3D( cv:: Point2f cam_2D, float depth)
{
    if ( !cam_info_set )
        return cv::Point3f();

    float fx     = 1.f / cam_intrinsic.at<double>(0,0);
    float fy     = 1.f / cam_intrinsic.at<double>(1,1);
    float cx     = cam_intrinsic.at<double>(0,2);
    float cy     = cam_intrinsic.at<double>(1,2);
    float factor = 1.f/1000.f;

    float dist = (depth ) * factor;
    float X    = (cam_2D.x - cx) * dist * fx;
    float Y    = (cam_2D.y - cy) * dist * fy;

    return cv::Point3f(X,Y,dist);
}

cv::Point2f itable_demo::bitmap_to_projector( int x, int y)
{
    if ( (cam_info_set && proj_cam_set && homo_valid) == false )
        return cv::Point2f();

    std::vector< cv::Point2f> bitmap_points,camera_points,projector_points;

    bitmap_points.push_back ( cv::Point2f( x, y) );
    // from bitmap to camera space (RGB)
    perspectiveTransform( bitmap_points, camera_points, homography);

    // from camera space (RGB) to pointcloud point
    cv::Point3f pointcloud = backproject_pixel_to_3D(camera_points[0], homo_depth);

    std::vector< cv::Point3f> pclp;
    pclp.push_back(pointcloud);
    // from pointcloud to projector space
    projectPoints(pclp, rot, trans, intrinsic, dist, projector_points);

    return projector_points[0];
}

void itable_demo::caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo)
{
    if ( cam_info_set )
        return;

    cam_intrinsic = cv::Mat(3,3,CV_64F,cvScalar(0.));
    double *ptrI = cam_intrinsic.ptr<double>(0, 0);

    for ( int i = 0; i < 9 ; i++, ptrI++)
        *ptrI = msg_camerainfo.K[i];

    cam_info_set = true;
    ROS_INFO("Camera intrinsic and dist_coeffs loaded from CameraInfo");
}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "itable_demo");
    itable::itable_demo demo;

    /*
     * Cout all possible video modes
    std::vector<sf::VideoMode> modes = sf::VideoMode::getFullscreenModes();
    for (std::size_t i = 0; i < modes.size(); ++i)
    {
        sf::VideoMode mode = modes[i];
        std::cout << "Mode #" << i << ": "
                  << mode.width << "x" << mode.height << " - "
                  << mode.bitsPerPixel << " bpp" << std::endl;
    }
    return 0;
    */
    demo.ros_init();
    demo.create_window("Game window",true);
    //demo.window = new sf::RenderWindow(sf::VideoMode(960,540),"temp",sf::Style::Resize);
    demo.load_data();


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
