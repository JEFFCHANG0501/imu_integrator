#include "imu_integrator/imu_integrator.h"

ImuIntegrator::ImuIntegrator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    imu_sub = nh_.subscribe("/imu_raw", 1000, &ImuIntegrator::imu_callback, this);
    vehicle_odom_sub = nh_.subscribe("/lgsvl/vehicle_odom", 100, &ImuIntegrator::vehicle_odom_callback, this);

    imu_odometry = nh_.advertise<visualization_msgs::Marker>("/imu_odometry", 1000);
    
    ROS_INFO("Starting to subscribe imu data.\n");
    ROS_INFO("Starting to subscribe lgsvl vehicle state.\n");

    fisrtTime = true;
    Eigen::Vector3d zero(0, 0, 0);
    pose.pos = zero;
    pose.orientation = Eigen::Matrix3d::Identity();
    velocity = zero;

    // intial path
    path.color.r = 1.0;
    path.color.a = 1.0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id = "world";
    path.ns = "points_and_lines";
    path.action = visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.3;
    geometry_msgs::Point p;
    p.x = 0; p.y = 0; p.z = 0;
    path.points.push_back(p);
    
    output_file.open("/home/jeff/imu_integrator.csv");
    output_file << "x,y,z\n";
    output_file << p.x << "," << p.y << "," << p.z << '\n';
}

ImuIntegrator::~ImuIntegrator()
{
    output_file.close();
}

void ImuIntegrator::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    if(fisrtTime)
    {
        time = msg->header.stamp;
        dt = 0;
        setGravity(msg->linear_acceleration);
        fisrtTime = false;
    }
    else
    {
        dt = (msg->header.stamp - time).toSec();
        time = msg->header.stamp;
        calcOrientation(msg->angular_velocity);
        calcPosition(msg->linear_acceleration);
        publishMessage();
    }
}

void ImuIntegrator::vehicle_odom_callback(const lgsvl_msgs::VehicleOdometryPtr& msg)
{
    v_speed = msg->velocity;
    // Eigen::Vector3d vv(v_speed, 0, 0);
    // std::cout << vv << std::endl;
}

void ImuIntegrator::setGravity(const geometry_msgs::Vector3& msg)
{
    gravity[0] = msg.x;
    gravity[1] = msg.y;
    gravity[2] = msg.z;
}

void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3& msg)
{
    Eigen::Matrix3d B;
    // std::cout << dt << std::endl;
    B <<      0     , -msg.z * dt,  msg.y * dt, 
          msg.z * dt,      0     , -msg.x * dt,
         -msg.y * dt,  msg.x * dt,      0     ;

    double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * dt;

    pose.orientation = pose.orientation * (Eigen::Matrix3d::Identity() + 
                       (std::sin(sigma) / sigma) * B + ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B); 

}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3& msg)
{
    Eigen::Vector3d acc_1(msg.x, msg.y, msg.z);
    Eigen::Vector3d acc_g = pose.orientation * acc_1;
    
    // calculate velocity from imu linear acceleration, then get imu position
    velocity = velocity + dt * (acc_g - gravity);
    Eigen::Vector3d vel(velocity[0], 0, 0);
    pose.pos = pose.pos + dt * pose.orientation * vel;
    // std::cout << vel << std::endl << std::endl;

    // get speed from /lgsvl/vehicle_odom, then calculate imu position
    // Eigen::Vector3d speed(v_speed, 0, 0);
    // pose.pos = pose.pos + dt * pose.orientation * speed;
} 

void ImuIntegrator::publishMessage()
{
    geometry_msgs::Point p;
    p.x = pose.pos[0];
    p.y = pose.pos[1];
    p.z = pose.pos[2];
    path.points.push_back(p);
    imu_odometry.publish(path);

    output_file << p.x << "," << p.y << "," << p.z << '\n';
}
