#ifndef IMU_INTEGRATOR_H
#define IMU_INTEGRATOR_H

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <lgsvl_msgs/VehicleOdometry.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include <fstream>

struct Pose
{
    Eigen::Vector3d pos;
    Eigen::Matrix3d orientation;
};

class ImuIntegrator
{
public:
    ImuIntegrator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~ImuIntegrator();

    void imu_callback(const sensor_msgs::ImuConstPtr& msg);

    void vehicle_odom_callback(const lgsvl_msgs::VehicleOdometryPtr& msg);

private:
    std::ofstream output_file;
    bool fisrtTime;
    double dt;
    double v_speed;
    Pose pose;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;

    visualization_msgs::Marker path;

    ros::Time time;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber imu_sub;
    ros::Subscriber vehicle_odom_sub;
    ros::Publisher imu_odometry;
    
    void setGravity(const geometry_msgs::Vector3& msg);

    void calcOrientation(const geometry_msgs::Vector3& msg); 

    void calcPosition(const geometry_msgs::Vector3& msg);

    void publishMessage();
};


#endif
