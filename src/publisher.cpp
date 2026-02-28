#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>

std::queue<cv::Mat> frame_queue;
std::mutex queue_mutex;

/*
    Read camera image and store in a queue.
*/
// void readThread(){

// }

int main(int argc, char **argv){

    const int IMAGE_WIDTH = 1024;
    const int IMAGE_HEIGHT = 512;

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

    while (ros::ok())
    {
        try{
            if(!cap.read(frame)){
                ROS_INFO("Failed to read a frame.");
            }
            else{
                // Check frame read errors

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

                camera_pub.publish(msg);
                ros::spinOnce();

                loop_rate.sleep();

                cv::waitKey(30);            
            }
        }
        catch(cv::Exception& e){
            ROS_ERROR("OpenCV exception: %s", e.what());
        }



    }

    return 0;
}