#include "imu_integrator/imu_integrator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_integrator_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // ImuIntegrator imu_integrator(nh, nh_private);
    ImuIntegrator *imu_integrator = new ImuIntegrator(nh, nh_private);
    ros::spin();

    return 0;
}