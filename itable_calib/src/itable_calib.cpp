/*
 * ROS node for kinect2 - projector calibration
 * Author: Petr Svoboda
 * Date: 1.5.2016
 */


#include "itable_calib/itable_calib.h"

namespace itable_calib
{

    iTable_calibration::iTable_calibration(ros::NodeHandle nh) : nh_(nh)
    {
        private_handle = ros::NodeHandle("~");
        pair_found = false;
        find_next_chessboard = false;
        end_calibration = false;
        pairs_counter = 0;
        cam_info_set = false;
        screen = NULL;
        ros_init();
    }

    void iTable_calibration::image_cb(const sensor_msgs::ImageConstPtr &msg_rgb,
                                            const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::CameraInfoConstPtr& msg_camerainfo)
    {
        ROS_INFO_STREAM_ONCE("First images arrived (this message won't be showed again)");


        if ( !cam_info_set )
            return;

        cv::Mat Kinect_camera_image;
        cv::Mat Kinect_depth_image;
        try
        {
            Kinect_camera_image = cv_bridge::toCvShare(msg_rgb, "bgr8")->image;
            Kinect_depth_image = cv_bridge::toCvCopy(msg_depth)->image;
            ROS_INFO_STREAM_ONCE("Color image  width x height " << Kinect_camera_image.cols << " x "<<Kinect_camera_image.rows);
            ROS_INFO_STREAM_ONCE("Depth image  width x height " << Kinect_depth_image.cols  << " x "<<Kinect_depth_image.rows);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_rgb->encoding.c_str());
        }


        if ( find_next_chessboard )
        {
            if ( screen == NULL )
                return;

            cv::Mat SDL_screen(screen->h,screen->w,CV_8UC4,screen->pixels);

		
		
            bool SDL_screen_found;
            bool Kinect_camera_found;

            std::vector < cv::Point2f > SDL_screen_points,Kinect_camera_points;
            std::vector < cv::Point3f >Kinect_camera_plus_depth_points;

            // Find chessboard in SDL_screen
            SDL_screen_found = cv::findChessboardCorners(SDL_screen,pattern_size,SDL_screen_points,CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);

            if ( SDL_screen_found)
            {
                ROS_INFO("Chessboard pattern found at SDL screen");
                cv::Mat gray_image;
                cv::cvtColor(SDL_screen, gray_image, CV_BGR2GRAY);
                cornerSubPix(gray_image, SDL_screen_points, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                                                                                                500, // max number of iterations
                                                                                                0.0001)); //max epsilon
            }

            // Find chessboard in Kinect RGB
            Kinect_camera_found = cv::findChessboardCorners(Kinect_camera_image,pattern_size,Kinect_camera_points,CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
            if ( Kinect_camera_found)
            {
                ROS_INFO("Chessboard pattern found at Kinect camera screen");
                cv::Mat gray_image;
                cv::cvtColor(Kinect_camera_image, gray_image, CV_BGR2GRAY);
                cornerSubPix(gray_image, Kinect_camera_points, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
                                                                                                500, // max number of iterations
                                                                                                0.0001)); //max epsilon
            }

            if ( SDL_screen_found && Kinect_camera_found )
            {
                ROS_INFO_STREAM("Found a new pair, total: " << pairs_counter + 1 );
                for ( int i = 0; i < Kinect_camera_points.size() ; i++)
                {
                    float depth_value = static_cast<float>( Kinect_depth_image.at<unsigned short>(Kinect_camera_points[i].y,Kinect_camera_points[i].x));
                    cv::Point3d rgb_plus_depth(Kinect_camera_points[i].x,Kinect_camera_points[i].y,depth_value);
                    Kinect_camera_plus_depth_points.push_back(rgb_plus_depth);
                }

                collection_camera_plus_depth_points.insert(collection_camera_plus_depth_points.end(),Kinect_camera_plus_depth_points.begin(),Kinect_camera_plus_depth_points.end());
                collection_SDL_screen_points.insert(collection_SDL_screen_points.end(),SDL_screen_points.begin(),SDL_screen_points.end());

                pair_found = true;
                pairs_counter++;
            }

            find_next_chessboard = false;
        }

        if ( end_calibration )
        {

            // pomoci CameraCalibrate
            std::vector< std::vector<cv::Point3f> > arr_of_arr_3D;
            std::vector< std::vector<cv::Point2f> > arr_of_arr_2D;
            std::vector< cv::Point3f > pointcloud_points;

            cv::Mat cameraMatrix2 = cam_intrinsic;
            cv::Mat distCoeffs2 = cam_dist_coeffs;
            std::vector<cv::Mat> rvec2,tvec2;

            cv::Point3f pointcloud_point;
            for ( int i = 0; i < collection_camera_plus_depth_points.size(); i++)
            {

                float fx = 1.f / cam_intrinsic.at<double>(0,0);
                float fy = 1.f / cam_intrinsic.at<double>(1,1);
                float cx = cam_intrinsic.at<double>(0,2);
                float cy = cam_intrinsic.at<double>(1,2);
                float factor = 1.f/1000.f;

                float dist = collection_camera_plus_depth_points[i].z * factor;
                float X = (collection_camera_plus_depth_points[i].x - cx) * dist * fx;
                float Y = (collection_camera_plus_depth_points[i].y - cy) * dist * fy;

                pointcloud_point = cv::Point3f(X,Y,dist);
                pointcloud_points.push_back(pointcloud_point);

            }
            arr_of_arr_3D.push_back(pointcloud_points);
            arr_of_arr_2D.push_back(collection_SDL_screen_points);

            

            float projY = projector_resolution.height / 2;
            float projX = projector_resolution.width / 2;

            cv::Point2d principal_point = cv::Point2d(projector_resolution.width/2,projector_resolution.height/2);
            cameraMatrix2 = (cv::Mat1d(3,3) << projX,0,principal_point.x,
                                                0,projY,principal_point.y,
                                                0,0,1
                                                );

            double error = cv::calibrateCamera(arr_of_arr_3D, arr_of_arr_2D, projector_resolution, cameraMatrix2, distCoeffs2, rvec2, tvec2, CV_CALIB_USE_INTRINSIC_GUESS );

            std::cout << "Calibration error: " << error << std::endl;

            cv::FileStorage fcamout(output_path + "/calibration_data.yaml", cv::FileStorage::WRITE);
            fcamout << "camera_matrix" << cameraMatrix2;
            fcamout << "dist_coeffs" << distCoeffs2;
            fcamout << "rotation_vec" << rvec2[0];
            fcamout << "trans_vec" << tvec2[0];
            fcamout << "error" << error;
            fcamout.release();

            exit(0);
        }

    }


    void iTable_calibration::ros_init()
    {
        int proj_width,proj_height;
        // get parameters from launch file
        private_handle.getParam("package_dir_path"  , output_path);
        private_handle.getParam("RGB_topic"         , rgb_topic);
        private_handle.getParam("depth_topic"       , depth_topic);
        private_handle.getParam("camerainfo_topic"  , caminfo_topic);
        private_handle.getParam("projector_width"   , proj_width);
        private_handle.getParam("projector_height"  , proj_height);
        private_handle.getParam("capture_delay"     , delay);

        projector_resolution = cv::Size(proj_width,proj_height);

        kinect_image_sub.subscribe  (nh_, rgb_topic    , 1);
        kinect_depth_sub.subscribe  (nh_, depth_topic  , 1);
        kinect_caminfo_sub.subscribe(nh_, caminfo_topic, 1);

        kinect_caminfo_sub.registerCallback(&iTable_calibration::caminfo_callback, this);

        exact_sync_.reset( new tExactSync(tExactPolicy(10), kinect_image_sub, kinect_depth_sub,kinect_caminfo_sub));
        exact_sync_->registerCallback(&iTable_calibration::image_cb, this);

        ROS_INFO("itable_calibration node is running");
    }

    void iTable_calibration::caminfo_callback(const sensor_msgs::CameraInfo& msg_camerainfo)
    {
        ROS_INFO_STREAM_ONCE("CameraInfo data arrived");
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "itable_calib");

    itable_calib::iTable_calibration node( (ros::NodeHandle()) );
    node.Set_chessboard_size(cv::Size(4,5));

    /* SDL things */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = NULL;
    SDL_Surface *pattern_surface = NULL;
    SDL_Surface* screen = NULL;
    SDL_Rect position;
    SDL_Event event;
    bool quit = false;
    bool capture = false;

    int screen_width  = node.Get_projector_size().width;
    int screen_height = node.Get_projector_size().height;

    position.x = 0;
    position.y = 0;
    position.w = 320;
    position.h = 240;

    window = SDL_CreateWindow(
            "Itable calibration window",                  // window title
            SDL_WINDOWPOS_UNDEFINED,           // initial x position
            SDL_WINDOWPOS_UNDEFINED,           // initial y position
            screen_width,     // width, in pixels
            screen_height,     // height, in pixels
            SDL_WINDOW_FULLSCREEN_DESKTOP                  // flags - see below
        );
    if (window == NULL)
    {
            std::cerr<< "Could not create window:"<< SDL_GetError() << std::endl;
            return 1;
    }
    pattern_surface = IMG_Load( (node.Get_package_dir_path() + "/data/pattern1.jpg").c_str() );
    if ( pattern_surface == NULL )
    {
        std::cerr << "Could not load picture " << node.Get_package_dir_path() + "/data/pattern1.jpg" << std::endl;
        return 1;
    }
    screen = SDL_GetWindowSurface(window);
    node.Set_SDL_Surface_pointer(screen);

    while ( quit == false )
    {

        screen = SDL_GetWindowSurface(window);
        node.Set_SDL_Surface_pointer(screen);

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
                    case SDLK_LEFT:     position.x -= 5; break;
                    case SDLK_RIGHT:    position.x += 5; break;
                    case SDLK_UP:       position.y -= 5; break;
                    case SDLK_DOWN:     position.y += 5; break;
                    case SDLK_ESCAPE:   node.End_calibration(); break;
                    case SDLK_KP_PLUS:  position.h*=1.07; position.w*=1.07; break;
                    case SDLK_KP_MINUS: position.h*=0.97; position.w*=0.97; break;
                    case SDLK_SPACE:    capture = true; break;
                }
                break;

            default:break;
            }
        }

        // fill with white color
        SDL_FillRect(screen,NULL, SDL_MapRGBA(screen->format,255,255,255,255));

        // draw chessboard at some position
        SDL_BlitScaled(pattern_surface,NULL , screen, &position);

        SDL_UpdateWindowSurface(window);
        if ( capture )
        {
            SDL_Delay( node.Get_delay() );
            node.Start_capturing();
            capture = false;
        }

        if ( node.Found_pair() )
        {
            SDL_FillRect(screen,NULL, SDL_MapRGBA(screen->format,0,0,0,255));
            SDL_UpdateWindowSurface(window);
            SDL_Delay(300);
        }

        ros::spinOnce();
    }

	return 0;
} //namespace
