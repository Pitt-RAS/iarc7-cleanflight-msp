//////////////////////////////////////////////////////////////////////////
//
// PX4 through mavros flight controller node.
//
// This node connects to a PX4 through mavros.
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "iarc7_fc_comms/CommonFcComms.hpp"
#include "iarc7_fc_comms/PX4FcComms.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FC_comms_px4");

    ROS_INFO("FC_comms_px4 begin");

    CommonFcComms<PX4FcComms>& fc = CommonFcComms<PX4FcComms>::getInstance();

    if (fc.init() != FcCommsReturns::kReturnOk) {
        ROS_ERROR("FcComms PX4 initialization failed");
        return 1;
    }

    if(fc.run() != FcCommsReturns::kReturnOk)
    {
        ROS_ERROR("FcComms PX4 exited with error");
        return 1;
    }

    // All is good.
    return 0;
}
