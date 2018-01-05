//////////////////////////////////////////////////////////////////////////
//
// Msp flight controller speed test
//
// This node connects to the FC using the MSP protocol and finds
// how fast packets can be sent back and forth.
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "iarc7_fc_comms/MspFcComms.hpp"
#include "iarc7_fc_comms/CommonConf.hpp"
#include "iarc7_msgs/OrientationThrottleStamped.h"

using namespace FcComms;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FC_comms");

    ros::NodeHandle nh;
    //ros::start

    ROS_INFO("FC_comms begin");

    MspFcComms comms(nh);

    FcCommsReturns status = comms.connect();

    if(status == FcCommsReturns::kReturnOk)
    {
        ROS_INFO("Successful connection");
    }
    else
    {
        ROS_ASSERT_MSG(false, "Failed connection");
    }

    while(ros::ok())
    {

        ros::Time start_time = ros::Time::now();
        float battery;
        
        for(int i = 0; i < 100; i++)
        {
            if(comms.getBattery(battery) != FcCommsReturns::kReturnOk)
            {
                ROS_ASSERT_MSG(false, "Error sending battery packet");
            }
        }
        ros::Duration delta = ros::Time::now() - start_time;
        ROS_INFO("Battery packet rate: %f hz", 100.0/delta.toSec());
        
        ros::spinOnce();

        start_time = ros::Time::now();
        for(int i = 0; i < 100; i++)
        {
            iarc7_msgs::OrientationThrottleStamped* direction = new iarc7_msgs::OrientationThrottleStamped();
            status = comms.processDirectionCommandMessage(
                static_cast<iarc7_msgs::OrientationThrottleStamped::ConstPtr>(direction));

            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ASSERT_MSG(false, "Error sending direction packet");
            }

            double attitude[3];
            ros::Time attitude_stamp = ros::Time::now();
            status = comms.getAttitude(attitude, attitude_stamp);

            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ASSERT_MSG(false, "Error sending attitude packet");
            }
        }

        delta = ros::Time::now() - start_time;
        ROS_INFO("Direction/attitude packet rate: %f hz", 200.0/delta.toSec());
        ros::spinOnce();

        start_time = ros::Time::now();
        for(int i = 0; i < 100; i++)
        {
            double accelerations[3];
            double angular_velocities[3];
            status = comms.getIMU(accelerations, angular_velocities);

            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ASSERT_MSG(false, "Error sending get acceleration packet");
            }
        }

        delta = ros::Time::now() - start_time;
        ROS_INFO("Acceleration packet rate: %f hz", 100.0/delta.toSec());
        ros::spinOnce();
    }

    status = comms.disconnect();

    if(status == FcCommsReturns::kReturnOk)
    {
        ROS_INFO("Successful disconnection");
    }
    else
    {
        ROS_ASSERT_MSG(false, "Failed disconnection");
    }

    // All is good.
    return 0;
}
