#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

// range of color to be detected a color ball
cv::Scalar colorLower(/**Please fill**/);
cv::Scalar colorUpper(/**Please fill**/);

void detectColorBall(cv::Mat& image) {
    // Convert BGR to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, /**Please fill**/);

    // Create a mask image to segment the original image based on its color
    cv::Mat mask;
    cv::inRange(hsv, /**Please fill**/, mask); // Use hue value to specify color

    // Reduce noise from the mask image
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,   cv::Mat(), cv::Point(-1, -1), 2);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,  cv::Mat(), cv::Point(-1, -1), 2);
    cv::GaussianBlur(mask, mask, cv::Size(-1, -1), 1, 1);

    // Apply the Hough transform to find circles in the mask image
    cv::HoughCircles(mask, /**Please fill**/);

    // Draw the found circles
    /**Please fill**/

    cv::imshow(/**Please fill**/);
    cv::waitKey(10);
}

void cameraImageCb(const sensor_msgs::ImageConstPtr& image) {
    // Transform from ROS topic to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

    detectColorBall(cv_ptr->image);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_opencv_tutorial");
    ros::NodeHandle nh;

    //ros::Subscriber cameraImageSub = nh.subscribe("usb_cam/image_raw", 1, cameraImageCb); // Use this topic if you have not calibrated your camera
    ros::Subscriber cameraImageSub = nh.subscribe("usb_cam/image_rect_color", 1, cameraImageCb);

    ros::spin();
}