#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "elas.h"

using namespace cv;
using namespace std;

FileStorage calib_file;
Mat Q, P1, P2, R1, R2;
Mat K1, K2, D1, D2, R;
Vec3d T;
Mat lmapx, lmapy, rmapx, rmapy;
Rect validRoi[2];
Size calib_img_size(672,376);
Size out_img_size(600, 300);
ros::Publisher point_cloud_pub;
ros::Time captured_time;
rosbag::Bag bag;

void publishPointCloud(Mat& img_left, Mat& dmap)
{
	Mat V = Mat(4, 1, CV_64FC1);
	Mat pos = Mat(4, 1, CV_64FC1);

	sensor_msgs::PointCloud2Ptr pc = boost::make_shared<sensor_msgs::PointCloud2>();

	pc->header.frame_id = "camera";
	pc->header.stamp = captured_time;
	pc->width = calib_img_size.height;
	pc->height = calib_img_size.width;
	pc->is_bigendian = false;
	pc->is_dense = false;

	sensor_msgs::PointCloud2Modifier pc_modifier(*pc);
	pc_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*pc, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*pc, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*pc, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc, "b");

//  int valid =0;
	for (int i = 0; i < img_left.cols; i++) {
		for (int j = 0; j < img_left.rows; j++) {
			float d = dmap.at<float>(j, i);
			// if low disparity, then ignore
			if (d < 3.) {
				continue;
			}
			// V is the vector to be multiplied to Q to get
			// the 3D homogenous coordinates of the image point
			V.at<double>(0, 0) = (double) (i);
			V.at<double>(1, 0) = (double) (j);
			V.at<double>(2, 0) = (double) d;
			V.at<double>(3, 0) = 1.;
			pos = Q * V; // 3D homogeneous coordinate
			double X = pos.at<double>(0, 0) / pos.at<double>(3, 0);
			double Y = pos.at<double>(1, 0) / pos.at<double>(3, 0);
			double Z = pos.at<double>(2, 0) / pos.at<double>(3, 0);

			int32_t red, blue, green;
			red = img_left.at < Vec3b > (j, i)[2];
			green = img_left.at < Vec3b > (j, i)[1];
			blue = img_left.at < Vec3b > (j, i)[0];

			*iter_x = X;
			*iter_y = Y;
			*iter_z = Z;
			*iter_r = red;
			*iter_g = green;
			*iter_b = blue;

			++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b;
		}
	}
	point_cloud_pub.publish(*pc);
	bag.write("/point_cloud", captured_time, *pc);	
}

/*
* input:两张灰度图像
* output:生成成功返回视差图
*/
bool generateDisparityMap(Mat& left, Mat& right, Mat &disparity_map) {
	if (left.empty() || right.empty())
		return false;
	const Size imsize = left.size();
	const int32_t dims[3] = { imsize.width, imsize.height, imsize.width };
	Mat leftdpf = Mat::zeros(imsize, CV_32F);
	Mat rightdpf = Mat::zeros(imsize, CV_32F);

	Elas::parameters param;
	param.postprocess_only_left = true;
	param.disp_max = 80;
	param.support_threshold = 0.7;
	param.match_texture = 32;
	Elas elas(param);
  
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	vector<Eigen::Vector3d> tri = elas.process(left.data, right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double> >(t2 - t1);
    cout <<  "genereate disparity image take time : " << time_span.count() << " seconds" << endl;

    disparity_map = leftdpf;

	// imshow left, right, and disparity map
    // find maximum disparity for scaling output disparity images to [0..255]
    double disp_min, disp_max;
    cv::minMaxLoc(leftdpf, &disp_min, &disp_max);
    // std::cout << "col = " << leftdpf.cols << " disp_max = " << disp_max << std::endl;

    Mat disparity_show;
    leftdpf.convertTo(disparity_show, CV_32FC1, 1.f/disp_max);
	// imshow("disparity_show", disparity_show);
	// waitKey(1);
	// generator rectified image
    Mat rectified_image = cv::Mat(left.rows, 2*left.cols, left.type());
    left.copyTo(rectified_image(cv::Rect(0, 0, left.cols, left.rows)));
    right.copyTo(rectified_image(cv::Rect(left.cols, 0, left.cols, left.rows)));
	std::cout << "left size = " << left.size() << std::endl;
	// imshow("rectified image", rectified_image);

	return true;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right) {

	captured_time = msg_left->header.stamp;

	// 转化ros image message到Mat
	cv_bridge::CvImagePtr cv_ptr;
	// left
	cv_ptr = cv_bridge::toCvCopy(msg_left, sensor_msgs::image_encodings::MONO8);
	Mat img_left = cv_ptr->image;
	// right
	cv_ptr = cv_bridge::toCvCopy(msg_right, sensor_msgs::image_encodings::MONO8);
	Mat img_right = cv_ptr->image;

	if (img_left.empty() || img_right.empty()) {
		ROS_WARN("image empty");
		return;
	}

    // imshow("left image", img_left);
    // imshow("right image", img_right);

	// recitify two image
    Mat l_img_rectified(out_img_size, img_left.type());
    Mat r_img_rectified(out_img_size, img_left.type());
    remap(img_left, l_img_rectified, lmapx, lmapy, cv::INTER_LINEAR);
    remap(img_right, r_img_rectified, rmapx, rmapy, cv::INTER_LINEAR);

    //imwrite("left.png", img_left);
    //imwrite("right.png", img_right);

    // grey to color
    Mat img_left_color;
    cv::cvtColor(img_left, img_left_color, CV_GRAY2BGR);

    // generate disparity map and point cloud
	Mat disparity_map;
	if(generateDisparityMap(l_img_rectified, r_img_rectified, disparity_map));
		publishPointCloud(img_left_color, disparity_map);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dense_reconstruction");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>( "/point_cloud", 1);
	// save point_cloud data to bag
	std::string bag_file_path;
	nh.param<std::string>("/dense_reconstruction/bag_file_path",bag_file_path,"~/Documents/pointclouds.bag");
	bag.open(bag_file_path, rosbag::bagmode::Write);

	// 读入标定参数
	std::string rootdir = ros::package::getPath("dense_reconstruction");
	std::string camera_config = rootdir + "/calibration_files/stereo_calibrate.yaml";
	calib_file = FileStorage(camera_config, FileStorage::READ);
	calib_file["K1"] >> K1;
	calib_file["K2"] >> K2;
	calib_file["D1"] >> D1;
	calib_file["D2"] >> D2;
	calib_file["R"] >> R;
	calib_file["T"] >> T;
	//calib_file["R1"] >> R1;
	//calib_file["R2"] >> R2;
   	//calib_file["P1"] >> P1;
   	//calib_file["P2"] >> P2;

	// 计算用于双目矫正的映射关系
    std::cout << " try to rectify" << std::endl;
	stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q,
			CV_CALIB_ZERO_DISPARITY, 0, calib_img_size, &validRoi[0], &validRoi[1]);
    // out_img_size.width = validRoi[0].width;
    // out_img_size.height = validRoi[0].height;
    ROS_INFO("rectify OK");
    cv::initUndistortRectifyMap(K1, D1, R1, P1, out_img_size, CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, out_img_size, CV_32F, rmapx, rmapy);
	cout << "validRoi0: " << validRoi[0] << "validRoi1: " << validRoi[1] << endl;
	ROS_INFO("dense_restruction done rectification");

	// 同步双目图像输入，注册ｃａｌｌｂａｃｋ函数
	message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/camera/left/image_raw", 10);
	message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/camera/right/image_raw", 10);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
	sync.registerCallback(boost::bind(&imgCallback, _1, _2));
	ros::spin();
    bag.close();
	return 0;
}
