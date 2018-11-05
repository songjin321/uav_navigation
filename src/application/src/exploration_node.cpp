#include <ros/ros.h>
#include "application/MainController.h"
// let robot explore enviorment with frontier-based method
int main(int argc, char **argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    MainController main_controller("uav_controller_server");
    // 初始化控制
    main_controller.init();

    // exploration
    main_controller.exploration();

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}