//https://blog.csdn.net/datase/article/details/80535710
//https://www.pianshen.com/article/9677256744/

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

sensor_msgs::Imu imu_car;
sensor_msgs::NavSatFix nav_car;

void imu_subCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    imu_car.orientation.x = imu_msg->orientation.x;
    imu_car.orientation.y = imu_msg->orientation.y;
    imu_car.orientation.z = imu_msg->orientation.z;
    imu_car.orientation.w = imu_msg->orientation.w;
    // imu_car.orientation_covariance = imu_msg->orientation_covariance;
    // imu_car.angular_velocity.x = imu_msg->angular_velocity.x ;
    // imu_car.angular_velocity.y = imu_msg->angular_velocity.y ;
    // imu_car.angular_velocity.z = imu_msg->angular_velocity.z ;
    // imu_car.angular_velocity_covariance = imu_msg->angular_velocity_covariance ;
    // imu_car.linear_acceleration.x = imu_msg->linear_acceleration.x;
    // imu_car.linear_acceleration.y = imu_msg->linear_acceleration.y;
    // imu_car.linear_acceleration.z = imu_msg->linear_acceleration.z;
    // imu_car.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance ;
}
void gps_subCallback(const sensor_msgs::NavSatFix::ConstPtr& nav_msg)//fix
{
    // nav_car.latitude = nav_msg->latitude;
    // nav_car.longitude = nav_msg->longitude;
    // nav_car.altitude = nav_msg->altitude;
    nav_car.longitude += 0.0000001;//116.355846121;
    nav_car.latitude +=0.0000001; //39.992046299;    
    nav_car.altitude = 65.233340007;
    nav_car.position_covariance = nav_msg->position_covariance;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber imu_sub = n.subscribe("imu", 10, imu_subCallback);
    ros::Subscriber gps_sub = n.subscribe("fix", 10, gps_subCallback);



    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(100);
    while(n.ok())
    {


        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt*0.5;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        //since all odometry is 6DOF we'll need a quaternion created from yaw

        //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = imu_car.orientation;


        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = imu_car.orientation;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;
        r.sleep();

        }
}
