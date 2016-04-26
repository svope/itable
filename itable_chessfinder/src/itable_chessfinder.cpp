/*
 * Developed by dcgm-robotics@FIT group
 * Author: Petr Svoboda
 * Date: 01.04.2012 (version 0.1)
 *
 * License: BUT OPEN SOURCE LICENSE (http://www.fit.vutbr.cz/~lampa/ipv6/LICENSE)
 *-------------------------------------------------------------------------------
 */


#include "itable_chessfinder/itable_chessfinder.h"

namespace itable_chessfinder {

    iTable_chessfinder::iTable_chessfinder(ros::NodeHandle nh) : nh_(nh)
    {
        ros_init();
        pattern_size = cv::Size(8,5);
        capture = false;
        cam_info_set = false;

        cv::FileStorage fs("/home/artable/svoboda_ws/calibration_data.yaml", cv::FileStorage::READ);
        fs["camera_matrix"] >> cameraMatrix ;
        fs["dist_coeffs"] >> distCoeffs;

        fs ["rotation_vec"] >> rvec;
        fs [ "trans_vec"] >> tvec;
        fs.release();

        SDL_screen = cv::Mat::zeros(1024, 1280, CV_32F);

    }


    void iTable_chessfinder::image_cb(const sensor_msgs::ImageConstPtr &msg_rgb,
                                            const sensor_msgs::ImageConstPtr& msg_depth,const sensor_msgs::CameraInfo& msg_camerainfo)
    {
        ROS_INFO_STREAM_ONCE("First image arrived, depth encoding: " << msg_depth->encoding.c_str());
        if ( !cam_info_set )
            return;



        cv::namedWindow( "Display window", CV_WINDOW_NORMAL );// Create a window for display.
        cvSetWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        cv::imshow( "Display window", SDL_screen );                   // Show our image inside it.

        int escape = cv::waitKey(1);
        if ( (escape % 256) == 27 || (escape % 256) == 'q' ) //modulo bcs of opencv bug
            exit(0);


        cv::Mat image;
        cv::Mat depth_image;
        try
          {
            image/*, image_org*/ = cv_bridge::toCvShare(msg_rgb, "bgr8")->image;
            depth_image = cv_bridge::toCvCopy(msg_depth)->image;
            ROS_INFO_STREAM_ONCE("Depth image  width x height " << depth_image.cols << " x "<<depth_image.rows);
            //cv::imshow("asdf", depth_image);
            //cv::waitKey(10);

          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_rgb->encoding.c_str());
          }

        if ( capture == false )
        {
            ROS_INFO("capturing...");
            std::vector < cv::Point2f > temp_object;
            std::vector < cv::Point3f > temp_object3D;
            std::vector < cv::Point3f > obj;

            bool object_found = cv::findChessboardCorners(image,pattern_size,temp_object,CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);

            if ( object_found == false )
                return;

                ROS_INFO("FOUND CHESSBOARD");
                for ( int i = 0; i < temp_object.size() ; i++)
                {
                    float depth_value = static_cast<float>( depth_image.at<unsigned short>(temp_object[i].y,temp_object[i].x));
                    cv::Point3f object3D(temp_object[i].x,temp_object[i].y,depth_value);
                    temp_object3D.push_back(object3D);
                }


                image_geometry::PinholeCameraModel camera_model;
                camera_model.fromCameraInfo(msg_camerainfo);

                cv::Point2d point2D;
                cv::Point3d world_point;
                for ( int i = 0; i < temp_object3D.size(); i++)
                {
                    
                    float fx = 1.f / cam_intrinsic.at<double>(0,0);
                    float fy = 1.f / cam_intrinsic.at<double>(1,1);
                    float cx = cam_intrinsic.at<double>(0,2);
                    float cy = cam_intrinsic.at<double>(1,2);
                    float factor = 1.f/1000.f;

                    float dist = temp_object3D[i].z * factor;
                    float X = (temp_object3D[i].x - cx) * dist * fx;
                    float Y = (temp_object3D[i].y - cy) * dist * fy;

                    cv::Point3f world_point = cv::Point3f(X,Y,dist);
                    //std::cout << final << std::endl;
                    obj.push_back(world_point);
                    /*
                    point2D.x = temp_object3D[i].x;
                    point2D.y = temp_object3D[i].y;

                    world_point = camera_model.projectPixelTo3dRay(point2D);
                    world_point.z = temp_object3D[i].z / 1000.f;
                    obj.push_back(world_point);
*/
                }


                // by matrix multiply
                /*

                cv::Mat RT(3, 4, CV_64F);
                for ( int i = 0; i < 3; i++)
                {
                    for ( int j = 0;j < 3;j++)
                    {
                        RT.at<double>(i,j) = rotatMat.at<double>(i,j);
                    }
                }
                RT.at<double>(0,3) = tvec.at<double>(0,0);
                RT.at<double>(1,3) = tvec.at<double>(1,0);
                RT.at<double>(2,3) = tvec.at<double>(2,0);

                   cv::Mat P = cameraMatrix * RT;

                    cv::Mat pp = P * pointMat ;
                    xx = pp.at<double>(0,0) / pp.at<double>(2,0);
                    yy = pp.at<double>(1,0) / pp.at<double>(2,0);


                  */


                // custom project points
                /*

                    cv::Mat temp = (rotatMat * pointXYZ) + tvec;
                    cv::Point2d xy = cv::Point2d(temp.at<double>(0,0) / temp.at<double>(2,0),temp.at<double>(1,0) / temp.at<double>(2,0));

                // distortion
                cv::Point2d xxyy;
                double r2 = xy.x*xy.x + xy.y * xy.y;
                double r4 = r2*r2;
                double r6 = r2*r4;
                double zlomek_top = 1.0 + distCoeffs.at<double>(0,0) * r2 + distCoeffs.at<double>(0,1) * r4 + distCoeffs.at<double>(0,4) * r6;
                double zlomek_bot = 1.0 + distCoeffs.at<double>(0,5) * r2 + distCoeffs.at<double>(0,6) * r4 + 0.0;
                double zlomek = zlomek_top / zlomek_bot;

                xxyy.x = xy.x * zlomek + 2.0 * distCoeffs.at<double>(0,2) * xy.x * xy.y + distCoeffs.at<double>(0,3) * ( r2 + 2.0 * xy.x * xy.x  );
                xxyy.y = xy.y * zlomek + distCoeffs.at<double>(0,2) * ( r2 + xy.y * xy.y * 2.0) + 2.0 * distCoeffs.at<double>(0,3) * xy.x * xy.y;




                cv::Point2d uv;

                uv.x = cameraMatrix.at<double>(0,0) * xxyy.x + cameraMatrix.at<double>(0,2);
                uv.y = cameraMatrix.at<double>(1,1) * xxyy.y + cameraMatrix.at<double>(1,2);

                  */


                imagePoints.clear();

                projectPoints(obj,rvec, tvec, cameraMatrix,distCoeffs,imagePoints );


                //cv::Mat SDL_screen(screen->h,screen->w,CV_8UC4,screen->pixels);

                SDL_screen = cv::Mat::zeros(1024, 1280, CV_32F);
                drawChessboardCorners(SDL_screen, pattern_size, imagePoints, object_found);

        }

    }


    void iTable_chessfinder::ros_init()
    {

        kinect_image_sub.subscribe(nh_, "/kinect2/qhd/image_color_rect", 1);
        kinect_depth_sub.subscribe(nh_, "/kinect2/qhd/image_depth_rect", 1);
        kinect_caminfo_sub.subscribe(nh_,"/kinect2/qhd/camera_info",1);

        kinect_caminfo_sub.registerCallback(&iTable_chessfinder::caminfo_callback, this);

        exact_sync_.reset( new tExactSync(tExactPolicy(10), kinect_image_sub, kinect_depth_sub,kinect_caminfo_sub));
        exact_sync_->registerCallback(&iTable_chessfinder::image_cb, this);


        ROS_INFO("kinect2_superpixels node is running");

    }

    void iTable_chessfinder::caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo)
    {

        if ( cam_info_set )
            return;

        cam_intrinsic = cv::Mat(3,3,CV_64F,cvScalar(0.));
        double *ptrI = cam_intrinsic.ptr<double>(0, 0);

        for ( int i = 0; i < 9 ; i++, ptrI++)
            *ptrI = msg_camerainfo.K[i];

        // Expecting plum bob model -> 5 parameters
        cam_dist_coeffs = cv::Mat(5,1,CV_64F, cvScalar(0.));
        double *ptrD = cam_dist_coeffs.ptr<double>(0);

        for ( int i = 0; i < 5; i++, ptrD++)
            *ptrD = msg_camerainfo.D[i];

        cam_info_set = true;
        ROS_INFO("Camera intrinsic and dist_coeffs loaded from CameraInfo");

    }




}


int main(int argc, char **argv) {
	ros::init(argc, argv, "kinect2_superpixels");

    itable_chessfinder::iTable_chessfinder node( (ros::NodeHandle()) );

    /* SDL things */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = NULL;
    SDL_Surface *pattern_surface = NULL;
    SDL_Surface* screen = NULL;
    SDL_Rect position;
    SDL_Event event;
    bool quit = false;

    position.x = 0;
    position.y = 0;
    position.w = 100;
    position.h = 100;
/*
    window = SDL_CreateWindow(
            "An SDL2 window",                  // window title
            SDL_WINDOWPOS_UNDEFINED,           // initial x position
            SDL_WINDOWPOS_UNDEFINED,           // initial y position
            1280,     // width, in pixels
            800,     // height, in pixels
            SDL_WINDOW_FULLSCREEN_DESKTOP                  // flags - see below
        );
    if (window == NULL)
    {
            std::cerr<< "Could not create window:"<< SDL_GetError() << std::endl;
            return 1;
    }
    pattern_surface = IMG_Load("/home/robolab/catkin_ws/src/kinect2_chessfinder/src/pattern.png");
    if ( pattern_surface == NULL )
    {
        std::cerr << "Could not load picture " << std::endl;
        return 1;
    }
    screen = SDL_GetWindowSurface(window);
    node.screen = screen;
*/
    while ( quit == false )
    {

        while( SDL_PollEvent( &event ) )
        {
            switch (event.type)
            {
            case SDL_QUIT:
                quit = true;
                break;

            case SDL_KEYDOWN:
                switch (event.key.keysym.sym)
                {
                    case SDLK_LEFT:  position.x -= 5; break;
                    case SDLK_RIGHT: position.x += 5; break;
                    case SDLK_UP:    position.y -= 5; break;
                    case SDLK_DOWN:  position.y += 5; break;
                    case SDLK_ESCAPE: quit = true; break;
                    case SDLK_KP_PLUS:  position.h*=1.07;position.w*=1.07;break;
                    case SDLK_KP_MINUS: position.h*=0.97;position.w*=0.97;break;
                }
                break;

            default:break;
            }
        }

        if ( node.capture == true )
        {
            position.x = node.imagePoints[0].x;
            position.y = node.imagePoints[0].y;
            std::cout << position.x << " " << position.y << std::endl;
            node.capture = false;
        }

        //SDL_FillRect(screen,NULL, SDL_MapRGBA(screen->format,0,0,0,255));

        //SDL_BlitScaled(pattern_surface,NULL , screen, &position);

        //SDL_UpdateWindowSurface(window);

        ros::spinOnce();
    }
	return 0;
}
