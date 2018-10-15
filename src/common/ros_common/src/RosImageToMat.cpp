//
// Created by songjin on 18-5-22.
//

#include "ros_common/RosImageToMat.h"
#include <string>
RosImageToMat::RosImageToMat(const std::string &topic_name, ros::NodeHandle &nh)
        : it_(nh), topic_name_(topic_name), isNewImage_(false)
{
    image_sub_ = it_.subscribe(topic_name_, 1,
                               &RosImageToMat::imageCb, this);
}
void RosImageToMat::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
    image_ = cv_ptr->image;
    isNewImage_ = true;
}
bool RosImageToMat::getNewImage(cv::Mat &img)
{
    if (isNewImage_ && !image_.empty())
    {
        img = image_.clone();
        isNewImage_ = false;
        return true;
    } else{
        return false;
    }

}