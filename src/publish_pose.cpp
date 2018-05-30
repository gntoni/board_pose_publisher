
#include <ros/ros.h>
#include "publish_pose.h"


publish_pose::publish_pose()
{
    camParamsReceived = false;
    std::string camera_info_topic, camera_image_topic;
    ros::param::get("charuco_board_pose/frame_name", frame_name);
    ros::param::get("charuco_board_pose/camera_info_topic", camera_info_topic);
    ros::param::get("charuco_board_pose/camera_image_topic", camera_image_topic);
    board_pose_pub = n.advertise<geometry_msgs::PoseStamped>("charuco_board_pose", 1000);
    board_pose_img_pub = n.advertise<sensor_msgs::Image>("charuco_board_pose_img", 1000);
    camera_sub = n.subscribe(camera_image_topic, 1000, &publish_pose::imageCallback, this);
    camera_info_sub = n.subscribe(camera_info_topic, 1000, &publish_pose::cameraInfoCallback, this);

    ros::param::get("charuco_board_pose/squaresX", squaresX);
    ros::param::get("charuco_board_pose/squaresY", squaresY);
    ros::param::get("charuco_board_pose/squareLength", squareLength);
    ros::param::get("charuco_board_pose/markerLength", markerLength);
    ros::param::get("charuco_board_pose/dictionaryId", dictionaryId);
    ros::param::get("charuco_board_pose/showRejected", showRejected);
    ros::param::get("charuco_board_pose/refindStrategy", refindStrategy);

    detectorParams = cv::aruco::DetectorParameters::create(); // ToDo: Load parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    axisLength = 0.5f * ((float)std::min(squaresX, squaresY) * (squareLength));

    // create charuco board object
    charucoboard = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    board = charucoboard.staticCast<cv::aruco::Board>();

    // draw board
    cv::Mat boardImage;
    charucoboard->draw( cv::Size(480, 720), boardImage, 10, 1 );
    cv::imwrite("BoardIM.jpg",boardImage);

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

    cv::Mat imageCopy;
    std::vector< int > markerIds, charucoIds;
    std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
    std::vector< cv::Point2f > charucoCorners;
    cv::Vec3d rvec, tvec;

    // detect markers
    cv::aruco::detectMarkers(img_ptr->image, dictionary, markerCorners, markerIds, detectorParams,
                         rejectedMarkers);

    // refind strategy to detect more markers
    if(refindStrategy)
        cv::aruco::refineDetectedMarkers(img_ptr->image, board, markerCorners, markerIds, rejectedMarkers,
                                         CameraMatrix, DistCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(markerIds.size() > 0)
        interpolatedCorners =
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img_ptr->image, charucoboard,
                                                 charucoCorners, charucoIds, CameraMatrix, DistCoeffs);

    // estimate charuco board pose
    bool validPose = false;
    if(CameraMatrix.total() != 0)
        validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                    CameraMatrix, DistCoeffs, rvec, tvec);

    // draw results
    img_ptr->image.copyTo(imageCopy);
    if(markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(img_ptr->image, markerCorners);
    }
    if(showRejected && rejectedMarkers.size() > 0)
        cv::aruco::drawDetectedMarkers(img_ptr->image, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));

    if(interpolatedCorners > 0) {
        cv::Scalar color;
        color = cv::Scalar(255, 0, 0);
        cv::aruco::drawDetectedCornersCharuco(img_ptr->image, charucoCorners, charucoIds, color);
    }

    if(validPose)
        cv::aruco::drawAxis(img_ptr->image, CameraMatrix, DistCoeffs, rvec, tvec, axisLength);

    // Publish image
    board_pose_img_pub.publish(img_ptr->toImageMsg());

    // Publish pose
    static tf::TransformBroadcaster br;
    double angle = cv::norm(rvec);
    rvec = rvec / angle;
    double s = std::sin(angle/2);
    double qx = rvec[0]*s;
    double qy = rvec[1]*s;
    double qz = rvec[2]*s;
    double qw = std::cos(angle/2);

    pose_msg.header.frame_id = frame_name;
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];

    pose_msg.pose.orientation.x = qx;
    pose_msg.pose.orientation.y = qy;
    pose_msg.pose.orientation.z = qz;
    pose_msg.pose.orientation.w = qw;

    board_pose_pub.publish(pose_msg);


    tf::Transform board_transform;
    board_transform.setOrigin( tf::Vector3(tvec[0], tvec[1], tvec[2]) );
    tf::Quaternion q(qx,qy,qz,qw);
    //q.setRotation(tf::Vector3(rvec[0], rvec[1], rvec[2]), angle);
    board_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(board_transform, ros::Time::now(), frame_name, "board_frame"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CharucoPosePublisher");
    publish_pose pose_publisher;
    ros::spin();
    return 0;
}
