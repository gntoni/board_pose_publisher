#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>


class publish_pose
{
    public:
        publish_pose();
        ~publish_pose();
        ros::NodeHandle n;
        bool camParamsReceived;
        cv::Mat CameraMatrix;
        cv::Mat DistCoeffs;

        // Board parameters
        int squaresX;
        int squaresY;
        float squareLength;
        float markerLength;
        int dictionaryId;
        bool showRejected;
        bool refindStrategy;

        cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
        cv::Ptr<cv::aruco::Dictionary> dictionary;

        float axisLength;

        cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
        cv::Ptr<cv::aruco::Board> board;

    private:
        ros::Publisher board_pose_pub;
        ros::Publisher board_pose_img_pub;
        ros::Subscriber camera_info_sub;
        ros::Subscriber camera_sub;
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr&);
        void imageCallback(const sensor_msgs::Image::ConstPtr&);
    
};