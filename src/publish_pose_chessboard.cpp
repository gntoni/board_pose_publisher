
#include <ros/ros.h>
#include "publish_pose_chessboard.h"


publish_pose::publish_pose()
{
    camParamsReceived = false;
    std::string camera_info_topic, camera_image_topic;
    ros::param::get("board_pose/frame_name", frame_name);
    ros::param::get("board_pose/camera_info_topic", camera_info_topic);
    ros::param::get("board_pose/camera_image_topic", camera_image_topic);
    board_pose_pub = n.advertise<geometry_msgs::PoseStamped>("chessboard_pose", 1000);
    board_pose_img_pub = n.advertise<sensor_msgs::Image>("chessboard_pose_img", 1000);
    camera_sub = n.subscribe(camera_image_topic, 1000, &publish_pose::imageCallback, this);
    camera_info_sub = n.subscribe(camera_info_topic, 1000, &publish_pose::cameraInfoCallback, this);

    ros::param::get("board_pose/squaresX", squaresX);
    ros::param::get("board_pose/squaresY", squaresY);
    ros::param::get("board_pose/squareLength", squareLength);

    //Generate chessboard 3D points
    for(int i = 0; i < squaresX; ++i)
       for(int j = 0; j < squaresY; ++j)
          object_corners.push_back(cv::Point3f(float(i*squareLength), float(j*squareLength), 0.f));
}

publish_pose::~publish_pose()
{
}

void publish_pose::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg)
{
  ROS_INFO("Got camera info");
  CameraMatrix = (cv::Mat_<double>(3,3) <<  cam_info_msg->K[0], 
                                            cam_info_msg->K[1],
                                            cam_info_msg->K[2],
                                            cam_info_msg->K[3],
                                            cam_info_msg->K[4],
                                            cam_info_msg->K[5],
                                            cam_info_msg->K[6],
                                            cam_info_msg->K[7],
                                            cam_info_msg->K[8]);

  DistCoeffs = (cv::Mat_<double>(1,5) <<    cam_info_msg->D[0], // k1
                                            cam_info_msg->D[1], // k2
                                            cam_info_msg->D[2], // t1
                                            cam_info_msg->D[3], // t2
                                            cam_info_msg->D[4]); // k3
  camParamsReceived = true;

  camera_info_sub.shutdown();  // unuscribe once params are known
}

void publish_pose::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!camParamsReceived)
    {
        ROS_INFO("Camera Parameters not yet configured. Waiting for camera_info topic.");
        return;
    }
    cv_bridge::CvImagePtr img_ptr;
    try
    {
      img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image_bgr = img_ptr->image;
    cvtColor ( image_bgr, image_grey, CV_BGR2GRAY, 0 );
    //bool found = cv::findChessboardCorners( image_grey, cv::Size(squaresX,squaresY), image_corners, CV_CALIB_CB_ADAPTIVE_THRESH );
    bool found = cv::findChessboardCorners( image_grey, cv::Size(squaresX,squaresY), image_corners);

    if (found)
    {
        ROS_INFO("FOUND");
        cv::Vec3d rotation_vec;
        cv::Vec3d translation_vec;

        solvePnP ( object_corners, image_corners, CameraMatrix, DistCoeffs, rotation_vec, translation_vec );

        // generate rotation matrix from vector
        extrinsic_matrix = cv::Mat_<double>::eye ( 4,4 );
        extrinsic_matrix ( 0,3 ) = translation_vec ( 0 );
        extrinsic_matrix ( 1,3 ) = translation_vec ( 1 );
        extrinsic_matrix ( 2,3 ) = translation_vec ( 2 );
        cv::Rodrigues ( rotation_vec, cv::Mat ( extrinsic_matrix, cv::Rect ( 0, 0, 3, 3 ) ), cv::noArray() );
        
        projection_matrix = CameraMatrix * extrinsic_matrix;
        
        // generate tf model to camera
        tf::Matrix3x3 R ( extrinsic_matrix( 0, 0 ), extrinsic_matrix( 0, 1 ), extrinsic_matrix( 0, 2 ),
                          extrinsic_matrix( 1, 0 ), extrinsic_matrix( 1, 1 ), extrinsic_matrix( 1, 2 ),
                          extrinsic_matrix( 2, 0 ), extrinsic_matrix( 2, 1 ), extrinsic_matrix( 2, 2 ) );

        tf::Vector3 t ( translation_vec ( 0 ), translation_vec ( 1 ), translation_vec ( 2 ) );
        transform_ =  tf::Transform ( R, t );
        tf::Quaternion q = transform_.getRotation();

        // TF transform
        tf_broadcaster_->sendTransform ( tf::StampedTransform ( transform_, msg->header.stamp, msg->header.frame_id, frame_name ) );

        // Publish pose
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose.pose.position.x = t.x();
        pose.pose.position.y = t.y();
        pose.pose.position.z = t.z();
        board_pose_pub.publish(pose);

        // Show image
        double square_size = squareLength;
        double nr_of_square =  std::max ( squaresX, squaresY );
        double size =  square_size * nr_of_square;

        int font = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 1.0;
        double thickness = 1.0;
        double lineType = CV_AA;
        double lineThickness = 3;

        cv::Mat_<double> Pi0 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 1 );
        cv::Point2d pi0 ( Pi0 ( 0,0 ) / Pi0 ( 0,2 ), Pi0 ( 0,1 ) / Pi0 ( 0,2 ) );
        cv::circle ( image_bgr, pi0, 3, CV_RGB ( 255,255,255 ) );

        cv::Mat_<double> Pi1 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << size, 0, 0, 1 );;
        cv::Point2d pi1 ( Pi1 ( 0,0 ) / Pi1 ( 0,2 ), Pi1 ( 0,1 ) / Pi1 ( 0,2 ) );
        cv::circle ( image_bgr, pi1, 3, CV_RGB ( 255,0,0 ) );
        putText ( image_bgr, "X", pi1, font, fontScale, CV_RGB ( 255,0,0 ), thickness, CV_AA );
        cv::line ( image_bgr, pi0, pi1, CV_RGB ( 255,0,0 ), lineThickness );

        cv::Mat_<double> Pi2 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, size, 0, 1 );
        cv::Point2d pi2 ( Pi2 ( 0,0 ) / Pi2 ( 0,2 ), Pi2 ( 0,1 ) / Pi2 ( 0,2 ) );
        cv::circle ( image_bgr, pi2, 3, CV_RGB ( 0,255,0 ) );
        putText ( image_bgr, "Y", pi2, font, fontScale, CV_RGB ( 0,255,0 ), thickness, CV_AA );
        cv::line ( image_bgr, pi0, pi2, CV_RGB ( 0,255,0 ), lineThickness );

        cv::Mat_<double> Pi3 = projection_matrix * ( cv::Mat_<double> ( 4,1 ) << 0, 0, size, 1 );
        cv::Point2d pi3 ( Pi3 ( 0,0 ) / Pi3 ( 0,2 ), Pi3 ( 0,1 ) / Pi3 ( 0,2 ) );
        cv::circle ( image_bgr, pi3, 3, CV_RGB ( 0,0,255 ) );
        putText ( image_bgr, "Z", pi3, font, fontScale, CV_RGB ( 0,0,255 ) , thickness, CV_AA );
        cv::line ( image_bgr, pi0, pi3, CV_RGB ( 0,0,255 ), lineThickness );

        drawChessboardCorners ( image_bgr, cv::Size(squaresX,squaresY), cv::Mat ( image_corners ), found );
        board_pose_img_pub.publish(img_ptr->toImageMsg());
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CharucoPosePublisher");
    publish_pose pose_publisher;
    ros::spin();
    return 0;
}
