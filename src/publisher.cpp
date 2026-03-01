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
ros::Publisher compressed_image_pub;

int fps = 30;
int jpeg_quality = 80;
std::string compressed_format = "jpeg";
int frame_height = 0;
int frame_width = 0;

const int IMAGE_WIDTH = 1024;
const int IMAGE_HEIGHT = 512;
const int X_start = 0;
const int Y_start = 128;

std::string cameraUrl, webUiPreviewImgTopic, rawImageTopic;

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

        // Update the camera frame parameters.
        frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);

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

    double scaleFactor = 0;
    double resizedHeight = 0, resizedWidth = 0;

    ros::NodeHandle n;

    image_transport::ImageTransport tp(n);

    // ros::Publisher camera_pub = n.advertise<std_msgs::String>("chatter", 1000);
    camera_pub = tp.advertise("camera/image", 1);

    // Publish compressed image to the web UI.
    compressed_image_pub = n.advertise<sensor_msgs::CompressedImage>("cam1_image_preview", 1);

    // Setup compression parameters
    std::vector<int> compression_params;

    if(compressed_format == "jpeg" || compressed_format == "jpg"){
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality);
    }

    ros::Rate loop_rate(10);

    while(running){

        if ((frame_height > 0) && (frame_width > 0)){
            scaleFactor =  static_cast<double>(IMAGE_WIDTH) / frame_width;

            resizedHeight = (frame_height * scaleFactor);
            resizedWidth = (frame_width * scaleFactor);
        }
        else{
            // Wait until camera captures a valid frame and update frame parameters.
            continue;
        }

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

            // Resize and crop the image
            cv::Mat resizedImg;

            cv::resize(frame, resizedImg, cv::Size(int(resizedWidth), int(resizedHeight)));

            cv::Rect roi(X_start, Y_start, IMAGE_WIDTH, IMAGE_HEIGHT);
            cv::Mat croppedFrame = resizedImg(roi);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", croppedFrame).toImageMsg();

            camera_pub.publish(msg);

            // Publishing compressed image
            sensor_msgs::CompressedImage compressed_img_msg;
            compressed_img_msg.header.stamp = ros::Time::now();
            compressed_img_msg.header.frame_id = "camera_frame";
            compressed_img_msg.format = compressed_format;

            std::vector<uchar> buffer;
            cv::imencode("." + compressed_format, croppedFrame, buffer, compression_params);
            compressed_img_msg.data = buffer;

            compressed_image_pub.publish(compressed_img_msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }

}


int main(int argc, char **argv){

    ros::init(argc, argv, "publisher", ros::init_options::NoSigintHandler);

    ros::NodeHandle nodeHandleParams("~");          // Private node handle
    
    if(nodeHandleParams.getParam("url", cameraUrl)){
        ROS_INFO("Camera url %s", cameraUrl.c_str());
    }

    if(nodeHandleParams.getParam("previewImg", webUiPreviewImgTopic)){
        ROS_INFO("Preview Image %s", webUiPreviewImgTopic.c_str());
    }

    if(nodeHandleParams.getParam("rawImg", rawImageTopic)){
        ROS_INFO("Raw Image %s", rawImageTopic.c_str());
    }

    // Register signal handlers
    signal(SIGINT, signalHandler); // Ctrl + C

    // Start camera reading thread
    std::thread camera_reader_thread(cameraReader);

    // Start image publishing thread
    std::thread image_publishing_thread(imagePublisher);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Wait for shutdown signal
    while (running && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the threads
    if(camera_reader_thread.joinable()){
        camera_reader_thread.join();
    }

    if(image_publishing_thread.joinable()){
        image_publishing_thread.join();
    }

    ROS_INFO("Camera publisher shutting down.");

    spinner.stop();
    ros::shutdown();

    return 0;
}