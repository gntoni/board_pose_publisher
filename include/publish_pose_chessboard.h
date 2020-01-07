#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

class publish_pose
{
    public:
        publish_pose();
        ~publish_pose();
        ros::NodeHandle n;
        bool camParamsReceived;
        cv::Mat CameraMatrix;
        cv::Mat DistCoeffs;
        std::string frame_name;

        // Board parameters
        int squaresX;
        int squaresY;
        float squareLength;



    private:
        geometry_msgs::PoseStamped pose_msg;
        std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

        std::vector<cv::Point2f> image_corners;
        std::vector<cv::Point3f> object_corners;        
        cv::Mat image_bgr;
        cv::Mat image_grey;
        tf::Transform transform_;
        cv::Mat_<double> extrinsic_matrix;
        cv::Mat_<double> projection_matrix;

        ros::Publisher board_pose_pub;
        ros::Publisher board_pose_img_pub;
        ros::Subscriber camera_info_sub;
        ros::Subscriber camera_sub;
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr&);
        void imageCallback(const sensor_msgs::Image::ConstPtr&);
    
};
