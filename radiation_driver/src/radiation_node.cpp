/*
*file radiation.cpp
*@author yangkang
*@brief radiation driver
*@version 0.1
*@date 2022.11.17
*@email yangkang123@mails.swust.edu.cn
*/
#include "radiation.hpp"

int main(int argc, char **argv)
{
    //初始化ROS节点
    ros::init(argc, argv, "radiation_driver");
    setlocale(LC_ALL,"");
    radiation my_driver;//对象

    if(!my_driver.radiation_init())
        ROS_ERROR("radiation_driver initialized failed.");
    ROS_INFO("radiation_driver initialized successful.");
    //循环运行
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        my_driver.writeContral();
        my_driver.readradiation();

        loop_rate.sleep();
        ros::spinOnce();
    }
    
}