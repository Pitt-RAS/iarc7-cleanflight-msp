#ifndef PX4_FC_COMMS_HPP
#define PX4_FC_COMMS_HPP

////////////////////////////////////////////////////////////////////////////
//
// PX4 flight controller node.
//
// Implements node behaviours specific to PX4 FC
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <string>

#include "CommonConf.hpp"
#include "MspConf.hpp"

#include "iarc7_msgs/BoolStamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>

namespace FcComms
{
    // Inherit from CommonFcComms to get the stuff any Fc is required to do.
    class PX4FcComms
    {
    public:
        PX4FcComms(ros::NodeHandle& nh);
        ~PX4FcComms() = default;

        // Used to find and connect to the serial port
        FcCommsReturns __attribute__((warn_unused_result))
            connect();

        // Disconnect from FC, should be called before destructor.
        FcCommsReturns  __attribute__((warn_unused_result))
            disconnect();

        // Handle periodically updating polled info.
        FcCommsReturns  __attribute__((warn_unused_result))
            handleComms();

        // Find out if the FC is armed.
        FcCommsReturns  __attribute__((warn_unused_result))
            isArmed(bool& armed);

        // Perform any post arming actions
        FcCommsReturns  __attribute__((warn_unused_result))
            postArm(bool arm);

        // Calibrate the accelerometer
        FcCommsReturns  __attribute__((warn_unused_result))
            calibrateAccelerometer();

        // Find out if the FC is in failsafe
        FcCommsReturns  __attribute__((warn_unused_result))
            isFailsafe(bool& failsafe);

        // Get the battery voltage of the FC.
        FcCommsReturns  __attribute__((warn_unused_result))
            getBattery(float& voltage);

        // Get the attitude of the FC in the order roll pitch yaw
        FcCommsReturns  __attribute__((warn_unused_result))
            getAttitude(double (&attitude)[3], ros::Time& stamp);

        // Getter for current connection status
        inline const FcCommsStatus& getConnectionStatus() const
        {
            return fc_comms_status_;
        }

        // Send the flight controller RX values
        FcCommsReturns  __attribute__((warn_unused_result))
            processDirectionCommandMessage(
                const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message);
        FcCommsReturns  __attribute__((warn_unused_result))
            setArm(bool arm);

        // Get the acceleration in m/s^2 and the angular velocities in rad/s
        FcCommsReturns  __attribute__((warn_unused_result))
            getIMU(double (&accelerations)[3], double (&angular_velocities)[3]);

        FcCommsReturns  __attribute__((warn_unused_result))
            isAutoPilotAllowed(bool& allowed);

        FcCommsReturns  __attribute__((warn_unused_result))
            safetyLand();
        
    private:
        // Don't allow the copy constructor or assignment.
        PX4FcComms(const PX4FcComms& rhs) = delete;
        PX4FcComms& operator=(const PX4FcComms& rhs) = delete;

        void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);

        void mavrosImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        ros::NodeHandle nh_;

        // Subscriber for the drones state
        ros::Subscriber mavros_state_sub_;

        // Subscriber for the orientation
        ros::Subscriber mavros_imu_sub_;

        // Attitude publisher
        ros::Publisher mavros_attitude_pub_;

        // Mavros arming service
        ros::ServiceClient mavros_arming_client_;

        // Mavros arming mode service
        ros::ServiceClient mavros_set_mode_client_;

        // Current fc comms state
        FcCommsStatus fc_comms_status_;

        // Current mavros state
        mavros_msgs::State mavros_current_state_;

        // Current imu message
        sensor_msgs::Imu mavros_imu_;

        // Current orientation throttle cammed
        iarc7_msgs::OrientationThrottleStamped 
            current_orientation_throttle_stamped_;
    };
} // End namspace

#endif
