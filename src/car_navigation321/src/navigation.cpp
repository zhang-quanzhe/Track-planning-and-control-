#include <tf/transform_broadcaster.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>

#include <math.h>

double degToRad(double deg);
void lglt2xyze(double longitude, double latitude, double altitude , double xyz[3]);
void xyze2xyzg(double longitude, double latitude, double xyz_e[3], double xyz_g[3]);
void lglt2xyzg(double longitude, double latitude, double altitude , double xyz_g[3]);
void map_projection_project(double refLatRad, double refLonRad,double latDeg,double lonDeg,double xyz[3]);

double distance(double xyz_e[3], double xyz_setp[3]);
double Cgeget(double longitude, double latitude, double Cge[3][3]);
double yaw_error(double yaw_car, double xyz_g[3], double xyz_setp[3]);

sensor_msgs::Imu imu_car;
sensor_msgs::NavSatFix nav_car;
sensor_msgs::NavSatFix nav_car_00;
geometry_msgs::Vector3Stamped euler_angels;//欧拉角/*!< Roll, Pitch and Yaw angles in rad. */
geometry_msgs::Vector3Stamped car_error_msg;

double pi = 3.1415926535897932;
double(*Cge)[3] = new double[3][3];//转换矩阵

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
    nav_car.latitude = nav_msg->latitude;
    nav_car.longitude = nav_msg->longitude;
    nav_car.altitude = nav_msg->altitude;

    // nav_car.longitude += 0.00000001;//116.355846121;
    // nav_car.latitude += 0.00000001; //39.992046299;    
    // nav_car.altitude += 0.0000001;//65.233340007;
    nav_car.position_covariance = nav_msg->position_covariance;

}

void pose_subCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // pose_msg.pose.orientation.x = pose_msg.pose.orientation.x ;
    // pose_msg.pose.orientation.y ;
    // pose_msg.pose.orientation.z ;
    // pose_msg.pose.orientation.w ;
}

void euler_angels_subCallback(const geometry_msgs::Vector3Stamped::ConstPtr& euler_angels_msg)
{/*!< Roll, Pitch and Yaw angles in rad. */
    euler_angels.vector.x = euler_angels_msg->vector.x;
    euler_angels.vector.y = euler_angels_msg->vector.y;
    euler_angels.vector.z = euler_angels_msg->vector.z;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "car_navigation");
    ros::NodeHandle n;

        // 创建一个publisher

    ros::Publisher car_error_pub = n.advertise<geometry_msgs::Vector3Stamped>("car_error_msg", 10);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher car_gps_path_pub = n.advertise<nav_msgs::Path>("car_gps_path", 10);

    nav_msgs::Path car_gps_path;
        car_gps_path.header.stamp=ros::Time::now(); 
        car_gps_path.header.frame_id="odom"; 



    ros::Subscriber imu_sub = n.subscribe("imu", 10, imu_subCallback);
    ros::Subscriber gps_sub = n.subscribe("fix", 10, gps_subCallback);
    ros::Subscriber pose_sub = n.subscribe("imu_pose", 10, pose_subCallback);
    ros::Subscriber euler_angels_sub = n.subscribe("euler_angels", 10, euler_angels_subCallback);

    double *xyz_e = new double[3] ;//地球系、e系
    double *xyz_g = new double[3] ;//地理系、g系
    double *xyz_setp = new double[3] ;//目标点的xyz
    double *xyz_startp = new double[3] ;//起点的xyz

//***********目标位置
    double setp_matrix[3] = {116.35420997355268, 39.992029859681146,  0};
//***********起始位置
    double startp_matrix[3] = {116.35586757872389, 39.99203807940514, 0};

    double sp_longtitude = setp_matrix[0];
    double sp_latitude = setp_matrix[1];
    double sp_altitude = setp_matrix[2];

    nav_car_00.longitude = startp_matrix[0];//35586757872389
    nav_car_00.latitude  = startp_matrix[1];    //99203807940509
    nav_car_00.altitude  = startp_matrix[2];

//
    // nav_car.longitude = start_matrix[0];//35586757872389
    // nav_car.latitude = start_matrix[1];    //99203807940509
    // nav_car.altitude = start_matrix[2];
//if(nav_car.position_covariance>)
    Cgeget(nav_car_00.longitude, nav_car_00.latitude, Cge);//位置矩阵


    // 设置循环的频率10=10Hz
    ros::Rate loop_rate(100);
    while(ros::ok())
    {


        ros::spinOnce();               // check for incoming messages
        
        lglt2xyzg(nav_car_00.longitude, nav_car_00.latitude, nav_car_00.altitude, xyz_startp);//原点 经纬度转地理系xyz（END）
        lglt2xyzg(nav_car.longitude, nav_car.latitude, nav_car.altitude, xyz_g);//车经纬度转地理系xyz（END）
        lglt2xyzg(sp_longtitude, sp_latitude, sp_altitude, xyz_setp);//目标点 经纬度转地理系xyz（END）
        
    //测试
        xyz_g[0] = xyz_startp[0];
        xyz_g[1] = xyz_startp[1];
        xyz_g[2] = xyz_startp[2];
        
/******************用相对坐标给命令**********************/
        xyz_setp[0] = xyz_startp[0]+10;
        xyz_setp[1] = xyz_startp[1]+10;
        xyz_setp[2] = xyz_startp[2];
/******************用相对坐标给命令**********************/

        double Error_distance = distance(xyz_g, xyz_setp);
        double Error_yaw = yaw_error(euler_angels.vector.z, xyz_g, xyz_setp);
        // ROS_INFO("nav_car longitude is %lf",nav_car.longitude);
        // ROS_INFO("nav_car latitude is %f",nav_car.latitude);
        // ROS_INFO("nav_car altitude is %f",nav_car.altitude);

        // ROS_INFO("longitude is %lf",sp_longtitude);
        // ROS_INFO("latitude is %lf",sp_latitude);
        // ROS_INFO("altitude is %lf",sp_altitude);

        // ROS_INFO("x_g is %f",xyz_g[0]);
        // ROS_INFO("y_g is %lf",xyz_g[1]);
        // ROS_INFO("z_g is %lf",xyz_g[2]);

        // ROS_INFO("x_sp is %lf",xyz_setp[0]);
        // ROS_INFO("y_sp is %lf",xyz_setp[1]);
        // ROS_INFO("z_sp is %lf",xyz_setp[2]);

        ROS_INFO("yaw_error_ca is %f",(Error_yaw)/pi*180);

        ROS_INFO("Roll is %f",euler_angels.vector.x/pi*180);
        ROS_INFO("Pitch is %f",euler_angels.vector.y/pi*180);
        ROS_INFO("Yaw is %f",euler_angels.vector.z/pi*180);
        //ROS_INFO("nav_car.position_covariance is %f", nav_car.position_covariance);

        // 发布消息
        car_error_msg.vector.x = Error_distance;
        car_error_msg.vector.y = Error_yaw;

        car_error_msg.header.stamp = ros::Time::now();
        car_error_pub.publish(car_error_msg);

/*********************************************odometry****************************************************/
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = 0;//xyz_g[0]-xyz_startp[0];
        odom_trans.transform.translation.y = 0;//xyz_g[1]-xyz_startp[1];
        odom_trans.transform.translation.z = 0;//xyz_g[2]-xyz_startp[2];

        odom_trans.transform.rotation = imu_car.orientation;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        //set the position    srand(9);

        odom.pose.pose.position.x = xyz_g[0]-xyz_startp[0];
        odom.pose.pose.position.y = xyz_g[1]-xyz_startp[1];
        odom.pose.pose.position.z = xyz_g[2]-xyz_startp[2];
        odom.pose.pose.orientation = imu_car.orientation;

        //set the velocity
        odom.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

/*********************************************odometry****************************************************/
        
/*********************************************path****************************************************/
        geometry_msgs::PoseStamped car_gps_path_poses_stamped; 
        car_gps_path_poses_stamped.pose.position.x = xyz_g[0]-xyz_startp[0];
        car_gps_path_poses_stamped.pose.position.y = xyz_g[1]-xyz_startp[1];
        car_gps_path_poses_stamped.pose.position.z = xyz_g[2]-xyz_startp[2];

        car_gps_path_poses_stamped.header.stamp=ros::Time::now(); 
        car_gps_path_poses_stamped.header.frame_id="odom"; 

        car_gps_path.poses.push_back(car_gps_path_poses_stamped); 

        // // 发布消息
        car_gps_path_pub.publish(car_gps_path);
/*********************************************path****************************************************/


        //usleep(10000);//延时10ms。
        loop_rate.sleep();

        }
}


/************************************************
 * 经纬度转地球系xyz
 * @param longitude 经度
 * @param latitude 纬度
 * @param radius 半径
 * @param xyz[3] 地球系、e系
 https://blog.csdn.net/ruangong1203/article/details/75994424
*************************************************/
void lglt2xyze(double longitude, double latitude, double altitude , double xyz[3])
{
    double lg = degToRad(longitude) ;
    double la = degToRad(latitude);

    double e2 = 0.006693421622966;//椭圆偏心率平方
    double Re = 6378245;//椭圆长半轴
    double Rn = Re/sqrt(1-e2*sin(la)*sin(la));//子午圈曲率半径

    double temp = (Rn+altitude)*cos(la);
    xyz[0] = temp*cos(lg);
    xyz[1] = temp*sin(lg);
    xyz[2] = (Rn*(1-e2)+altitude)*sin(la);

}

void xyze2xyzg(double longitude, double latitude, double xyz_e[3], double xyz_g[3])//地球系e转地理系g
{   
    double lg = degToRad(longitude) ;//经度
    double la = degToRad(latitude) ;//纬度

    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            xyz_g[i] +=  Cge[i][j] * xyz_e[j] ;
        }
    }
}

void lglt2xyzg(double longitude, double latitude, double altitude , double xyz_g[3])//经纬度转地理系g
{
    double lg = degToRad(longitude) ;//经度
    double la = degToRad(latitude) ;//纬度

    double e2 = 0.006693421622966;//椭圆偏心率平方
    double Re = 6378245;//椭圆长半轴
    double Rn = Re/sqrt(1-e2*sin(la)*sin(la));//子午圈曲率半径
    double xyz_e[3];

    double temp = (Rn+altitude)*cos(la);
    xyz_e[0] = temp*cos(lg);
    xyz_e[1] = temp*sin(lg);
    xyz_e[2] = (Rn*(1-e2)+altitude)*sin(la);

    for ( int i = 0; i < 3; i++) 
    {
        xyz_g[i] =  Cge[i][0] * xyz_e[0] + Cge[i][1] * xyz_e[1] + Cge[i][2] * xyz_e[2];
    }
}

double Cgeget(double longitude, double latitude, double Cge[3][3])
{
    double lg = degToRad(longitude) ;//经度
    double la = degToRad(latitude) ;//纬度
    double Cge_raw[3][3] = {{-sin(lg), cos(lg), 0}, {-sin(la)*cos(lg), -sin(la)*sin(lg), cos(la)}, {cos(la)*cos(lg), cos(la)*sin(lg), sin(la)}};
    
    for ( int i = 0; i < 3; i++) 
    {
        for ( int j = 0; j < 3; j++) 
        {
            Cge[i][j] = Cge_raw[i][j] ;
        }
    }
}

double degToRad(double deg)
{
    double rad = deg * pi/180;
    return rad;
}

double distance(double xyz_g[3], double xyz_setp[3])
{
    double disx = xyz_g[0] - xyz_setp[0];
    double disy = xyz_g[1] - xyz_setp[1];
    double disz = xyz_g[2] - xyz_setp[2];
    double dis = sqrt(disx*disx+disy*disy+disz*disz);
    return dis;
}

double yaw_error(double yaw_car,double xyz_g[3], double xyz_setp[3])
{
    double disx = -(xyz_g[0] - xyz_setp[0]);
    double disy = -(xyz_g[1] - xyz_setp[1]);

    double yaw_error = atan2(disx,disy) - yaw_car;
    return yaw_error;
}
