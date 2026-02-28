#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <condition_variable>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>
#include <sstream>


// Global variables
std::queue<cv::Mat> frame_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;

// Control flags
std::atomic<bool> running(true);

image_transport::Publisher camera_pub;

int fps = 30;

void signalHandler(int signum){
    std::cout << "\nInterrupt signal (" << signum <<") received. \n)";
    running = false;
    queue_cv.notify_all();
}

/*
    Read camera image and store in a queue.
*/
void cameraReader(){  

    cv::VideoCapture cap("rtsp://admin:Admin123@192.168.1.211:554/path");

    if(!cap.isOpened()){
        ROS_INFO("Camera didn't open.");
    }
    else{
        ROS_INFO("Camera reader started");
    }

    cv::Mat frame;

    // Frame interval from fps
    auto frame_interval = std::chrono::milliseconds(1000/fps);
    
    // Run as long as camera not stopped.
    while(running){

        auto start_time = std::chrono::steady_clock::now();

        try{
            if(!cap.read(frame)){
                ROS_INFO("Failed to read a frame.");
            }
            else{

                if(!frame.empty()){
                    // Copy image to the queue.
                    std::unique_lock<std::mutex> lock(queue_mutex);

                    if(frame_queue.size() > 2){
                        frame_queue.pop();
                    }

                    frame_queue.push(frame.clone());
                }

                // Notify the publisher thread.
                queue_cv.notify_one();
                cv::waitKey(30);  
            }
        }
        catch(cv::Exception& e){
            ROS_ERROR("OpenCV exception: %s", e.what());
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;

        if(elapsed < frame_interval){
            std::this_thread::sleep_for(frame_interval - elapsed);
        }
    }

    // Release video capture object
    cap.release();
}

/*
Image publisher
*/
void imagePublisher(){

    const int IMAGE_WIDTH = 1024;
    const int IMAGE_HEIGHT = 512;

    ros::NodeHandle n;

    image_transport::ImageTransport tp(n);

    // ros::Publisher camera_pub = n.advertise<std_msgs::String>("chatter", 1000);
    camera_pub = tp.advertise("camera/image", 1);

    ros::Rate loop_rate(10);

    while(running){

        cv::Mat frame;

        {
            std::unique_lock<std::mutex> lock(queue_mutex);

            if (queue_cv.wait_for(lock, std::chrono::milliseconds(1000),
                [&] {return !frame_queue.empty() || !running;})){
                    if(!frame_queue.empty()){
                        frame = frame_queue.front();
                        frame_queue.pop();
                    }
                }
        }

        // Check frame read errors
        if(!frame.empty()){
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

            camera_pub.publish(msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));


        // ros::spinOnce();

        // loop_rate.sleep();
    }

}


int main(int argc, char **argv){

    ros::init(argc, argv, "publisher", ros::init_options::NoSigintHandler);

    // Register signal handlers
    signal(SIGINT, signalHandler); // Ctrl + C

    // Start camera reading thread
    std::thread camera_reader_thread(cameraReader);

    std::thread image_publishing_thread(imagePublisher);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Wait for shutdown signal
    while (running && ros::ok())
    {
  
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the threads

    spinner.stop();
    ros::shutdown();

    return 0;
}