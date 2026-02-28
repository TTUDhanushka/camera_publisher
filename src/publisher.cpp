#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>


int main(int argc, char **argv){

    ros::init(argc, argv, "publisher");

    ros::NodeHandle n;

    image_transport::ImageTransport tp(n);

    // ros::Publisher camera_pub = n.advertise<std_msgs::String>("chatter", 1000);
    image_transport::Publisher camera_pub = tp.advertise("camera/image", 1);

    cv::VideoCapture cap("rtsp://admin:Admin123@192.168.1.211:554/path");

    if(!cap.isOpened()){
        ROS_INFO("Camera didn't open.");
    }

    cv::Mat frame;

    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {
        // std_msgs::String msg;
        if(!cap.read(frame)){
            ROS_INFO("Failed to read a frame.");
        }

        cv::waitKey(30);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        // std::stringstream ss;

        // ss << "Test" << count;

        // msg.data = ss.str();

        // ROS_INFO("%s", msg.data.c_str());

        camera_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();

        ++count;

    }

    return 0;
}