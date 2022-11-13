/*
 * ********************************************************************************
 * SYSTEM            | obstacle aboidance
 * Publisher         | cmd_vel_pub
 * 					 | state_pub
 * Subscriber        | scan_sub
 * ********************************************************************************
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>

ros::Publisher cmd_vel_pub;
ros::Publisher state_pub;
ros::Subscriber scan_sub;

geometry_msgs::Twist cmd_vel;
sensor_msgs::LaserScan latest_scan;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    latest_scan = *msg;
}

// 以下URM-40-LC-EW依存(使用するlidarによって正面の配列構造が変わる)
// hokuyo lidar scan範囲仕様：https://www.hokuyo-aut.co.jp/search/single.php?serial=21
// URM-40-LC-EWでは検出範囲270°、分解能0.125°、点群数2160
// Monitoring 0~90° in front
float observation_front_right()
{
    float min_range = 1000.0f;
    float min_range_angle = 0.0f;
    
    //for (int i = 0; i < 5; i++) {
    for (int i = 180; i < 270; i++) {
    //for (int i = 270; i < 360; i++) {
        /**
        ***********************************************************************
        * First condition  |  In case of an error value
        * Second condition |  Out of measurement range
        * Third condition  |  For infinity
        ***********************************************************************
        */
        if (latest_scan.ranges[i] < latest_scan.range_min || latest_scan.ranges[i] > latest_scan.range_max || std::isnan(latest_scan.ranges[i])) {
            // 測定範囲外 = 正面に十分な距離があるわけではない。(場合によっては、とても近い距離かもしれない)
            ROS_INFO("i = %d, front-range: measurement error", i);
        } else {
            if(latest_scan.ranges[i] < min_range) {
                min_range = latest_scan.ranges[i];
                ROS_INFO("i = %d, front-range: %0.3f", i, latest_scan.ranges[i]);
                //min_range_angle = i;
            }
        }
    }

    return min_range;
}

// 以下URM-40-LC-EW依存(使用するlidarによって正面の配列構造が変わる)
// hokuyo lidar scan範囲仕様：https://www.hokuyo-aut.co.jp/search/single.php?serial=21
// URM-40-LC-EWでは検出範囲270°、分解能0.125°、点群数2160
// Monitoring 90~180° in front
float observation_front_left()
{
    float min_range = 1000.0f;
    float min_range_angle = 0.0f;
    
    //for(int i = 355; i < 360; i++) {
    //for(int i = 120; i < 180; i++) {
    //for(int i = 180; i < 270; i++) {
    //for(int i = 0; i < 90; i++) { // left back
    //for(int i = 90; i < 180; i++) { // ?
    //for(int i = 270; i < 360; i++) {
    for(int i = 270; i < 360; i++) {
        /**
        ***********************************************************************
        * First condition  |  In case of an error value
        * Second condition |  Out of measurement range
        * Third condition  |  For infinity
        ***********************************************************************
        */
        if (latest_scan.ranges[i] < latest_scan.range_min || latest_scan.ranges[i] > latest_scan.range_max || std::isnan(latest_scan.ranges[i])) {
            // 測定範囲外 = 正面に十分な距離があるわけではない。(場合によっては、とても近い距離かもしれない)
            ROS_INFO("i = %d, front-range: measurement error", i);
        } else {
            if(latest_scan.ranges[i] < min_range) {
                min_range = latest_scan.ranges[i];
                ROS_INFO("i = %d, front-range: %0.3f", i, latest_scan.ranges[i]);
                //min_range_angle = i;
            }
        }
    }

    return min_range;
}

// Monitoring in front
int obstacle_detection()
{
    /* state = 0 right
       state = 1 left
       state = 2 no detection
    */
    int state;

    float right_min_range = observation_front_right();
    float left_min_range  = observation_front_left();

    if ( (right_min_range <= 0.6) || (left_min_range <= 0.6) ) {
        if (right_min_range < left_min_range) {
            state = 0;
        } else if (right_min_range > left_min_range) {
            state = 1;
        }
    } else {
        state = 2;
    }

    return state;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_rplidar");

    ros::NodeHandle nh;
    scan_sub    = nh.subscribe("/scan", 10, scan_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_obstacle_avoidance", 10);
    state_pub   = nh.advertise<std_msgs::Int8>("/state_obstacle_avoidance", 10);

    ros::Rate rate(10.0);

    while(ros::ok()) {
        std_msgs::Int8 msg;

        ros::spinOnce();

        if (latest_scan.ranges.size() > 0) {
            /* state = 0 right
               state = 1 left
               state = 2 no detection
            */
            int state = obstacle_detection();
            msg.data = state;

            if (state == 0) {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
            } else if (state == 1) {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
            } 
            
            cmd_vel_pub.publish(cmd_vel);
            state_pub.publish(msg);
        }

        rate.sleep();
    }

    return 0;
}
