#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  Mat inputGrayImg, cannyImg, cannyEdges;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cv_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }

  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // create output image to be published	
    cannyImg.create(cv_ptr->image.size(),cv_ptr->image.type());	

    // gray image for canny edge detection 	
    cvtColor(cv_ptr->image, inputGrayImg, COLOR_BGR2GRAY);

    // canny edge detection of image
    Canny(inputGrayImg, cannyEdges, 30, 90, 3);

    cannyImg = Scalar::all(0);
    
    // copy only detected edges from original image
    cv_ptr->image.copyTo(cannyImg, cannyEdges);
    
    // Output modified video stream
    cannyImg.copyTo(cv_ptr->image); 
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
