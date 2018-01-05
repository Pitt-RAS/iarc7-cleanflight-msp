#ifndef COMMON_FC_COMMS_HPP
#define COMMON_FC_COMMS_HPP

////////////////////////////////////////////////////////////////////////////
//
// Common flight controller node.
//
// Contains declarations for objects common to all flight
// control communication nodes. Handles a lot of ROS specifics.
//
////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include "iarc7_safety/SafetyClient.hpp"
#include "iarc7_msgs/Arm.h"
#include "iarc7_msgs/BoolStamped.h"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/LandingGearContactsStamped.h"
#include "iarc7_msgs/OrientationAnglesStamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include <ros_utils/ParamUtils.hpp>

#include <sensor_msgs/Imu.h>
#include "std_srvs/SetBool.h"

#include "CommonConf.hpp"

//mspfccomms actually handles the serial communication
namespace FcComms{

    template<class T>
    class CommonFcComms
    {
    public:
        // Used to make sure this class remains a singleton.
        static CommonFcComms<T>& getInstance();

        FcCommsReturns __attribute__((warn_unused_result)) init();

        FcCommsReturns __attribute__((warn_unused_result)) run();

        //Delete copy constructors, assignment operator
        CommonFcComms(const CommonFcComms& rhs) = delete;
        CommonFcComms& operator=(const CommonFcComms& rhs) = delete;

    private:

        // Class can only be made by class
        CommonFcComms();

        // Publish to the FC sensor topics
        FcCommsReturns __attribute__((warn_unused_result)) publishTopics();

        // Attempt reconnection
        void reconnect();

        // Update information from flight controller and send information
        void update();

        // Calibrate the FC's accelerometer
        void calibrateAccelerometer();

        // Update flight controller armed information
        void updateArmed();

        // Update flight controller auto pilot status information
        void updateAutoPilotEnabled();

        // Update flight controller battery information
        void updateBattery();

        // Update flight controller attitude information
        void updateAttitude();

        // Send arm or direction message to flight controller
        void updateDirection();

        // Get acceleration data from the flight controller
        void updateAccelerations();

        // Activate the safety response of the flight controller impl
        void activateFcSafety();

        // Publish the flight controller status
        void publishFcStatus();

        // Send out the orientation data for the quad
        void sendOrientation(double (&attitude)[3], ros::Time attitude_stamp);

        // Send out the accelerations from the FC
        void sendIMU(double (&accelerations)[3], double (&angular_velocities)[3]);

        inline void uavDirectionCommandMessageHandler(
                const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message) {
            last_direction_command_message_ptr_ = message;
            have_new_direction_command_message_ = true;
        }

        inline void contactSwitchMessageHandler(
                const iarc7_msgs::LandingGearContactsStamped::ConstPtr& message) {
            last_contact_switch_message_ptr_ = message;
        }

        bool uavArmServiceHandler(
                iarc7_msgs::Arm::Request& request,
                iarc7_msgs::Arm::Response& response);

        // This node's handle
        ros::NodeHandle nh_;

        // NodeHandle in this node's namespace
        ros::NodeHandle private_nh_;

        // Safety client
        Iarc7Safety::SafetyClient safety_client_;

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;
        ros::Publisher imu_publisher;
        ros::Publisher orientation_pub_;

        // Just use the default constructor
        T flightControlImpl_;

        // Subscriber for uav_angle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for the contact switch values
        ros::Subscriber contact_switch_subscriber;

        // Service to arm copter
        ros::ServiceServer uav_arm_service;

        iarc7_msgs::OrientationThrottleStamped::ConstPtr last_direction_command_message_ptr_;

        iarc7_msgs::LandingGearContactsStamped::ConstPtr last_contact_switch_message_ptr_;

        ros::Duration valid_contact_switch_message_delay_;

        ros::Duration orientation_timestamp_offset_;

        bool have_new_direction_command_message_ = false;

        typedef void (CommonFcComms::*CommonFcCommsMemFn)();

        std::vector<CommonFcCommsMemFn> sequenced_updates = {
                &CommonFcComms::updateArmed,
                &CommonFcComms::updateAutoPilotEnabled
            };

        uint32_t current_sequenced_update = 0;

        bool publish_imu_;

        bool fc_armed_ = false;

        bool fc_failsafe_ = false;

        bool fc_auto_pilot_enabled_ = false;

        bool initial_heading_as_offset_ = false;

        bool calibrate_accelerometer_ = false;

        double initial_heading_offset_ = std::nan("");

    };
}

using namespace FcComms;

template<class T>
CommonFcComms<T>::CommonFcComms() :
nh_(),
private_nh_("~"),
safety_client_(nh_, "fc_comms_msp"),
battery_publisher(),
status_publisher(),
imu_publisher(),
orientation_pub_(),
flightControlImpl_(private_nh_),
uav_angle_subscriber(),
contact_switch_subscriber(),
uav_arm_service(),
last_direction_command_message_ptr_(),
last_contact_switch_message_ptr_(),
valid_contact_switch_message_delay_(),
orientation_timestamp_offset_(),
publish_imu_(ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "publish_imu"))
{
    if (ros_utils::ParamUtils::getParam<bool>(private_nh_,
                                              "publish_fc_battery")) {
        sequenced_updates.push_back(&CommonFcComms::updateBattery);
    }

    initial_heading_as_offset_ = 
                            ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "initial_heading_as_offset");

    calibrate_accelerometer_ = 
                            ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "calibrate_accelerometer");

    valid_contact_switch_message_delay_ = ros::Duration(
                        ros_utils::ParamUtils::getParam<double>(
                        private_nh_, "valid_contact_switch_message_delay"));

    orientation_timestamp_offset_ = ros::Duration(
                                  ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_orientation_timestamp_offset"));
}

template<class T>
CommonFcComms<T>& CommonFcComms<T>::getInstance()
{
    // Allow only one instance
    static CommonFcComms<T>* instance = nullptr;
    if (instance == nullptr) {
        instance = new CommonFcComms<T>;
    }
    return *instance;
}

// Use to connect to topics
template<class T>
FcCommsReturns CommonFcComms<T>::init()
{
    ROS_INFO("fc_comms_msp: Forming bond with safety client");
    ROS_ASSERT_MSG(safety_client_.formBond(), "fc_comms_msp: Could not form bond with safety client");
    ROS_INFO("fc_comms_msp: Formed bond with safety client");

    uav_arm_service = nh_.advertiseService("uav_arm",
                                       &CommonFcComms::uavArmServiceHandler,
                                       this);
    if (!uav_arm_service) {
        ROS_ERROR("CommonFcComms failed to create arming service");
        return FcCommsReturns::kReturnError;
    }

    battery_publisher = nh_.advertise<iarc7_msgs::Float64Stamped>("fc_battery", 50);
    if (!battery_publisher) {
        ROS_ERROR("CommonFcComms failed to create battery publisher");
        return FcCommsReturns::kReturnError;
    }

    status_publisher = nh_.advertise<iarc7_msgs::FlightControllerStatus>(
            "fc_status", 50);
    if (!status_publisher) {
        ROS_ERROR("CommonFcComms failed to create status publisher");
        return FcCommsReturns::kReturnError;
    }

    if(publish_imu_)
    {
        imu_publisher = nh_.advertise<sensor_msgs::Imu>(
                "fc_imu", 50);
        if (!imu_publisher) {
            ROS_ERROR("CommonFcComms failed to create imu publisher");
            return FcCommsReturns::kReturnError;
        }
    }

    orientation_pub_ = nh_.advertise<iarc7_msgs::OrientationAnglesStamped>(
            "fc_orientation", 50);
    if (!orientation_pub_) {
        ROS_ERROR("CommonFcComms failed to create orientation publisher");
        return FcCommsReturns::kReturnError;
    }

    uav_angle_subscriber = nh_.subscribe("uav_direction_command",
                                         100,
                                         &CommonFcComms::uavDirectionCommandMessageHandler,
                                         this);
    if (!uav_angle_subscriber) {
        ROS_ERROR("CommonFcComms failed to create angle subscriber");
        return FcCommsReturns::kReturnError;
    }

    contact_switch_subscriber = nh_.subscribe("landing_gear_contact_switches",
                                         100,
                                         &CommonFcComms::contactSwitchMessageHandler,
                                         this);

    ROS_INFO("FC Comms registered and subscribed to topics");

    if (calibrate_accelerometer_)
    {
        const ros::Time start_time = ros::Time::now();
        while (ros::ok()
               && last_contact_switch_message_ptr_ == nullptr
               && ros::Time::now()
                  < start_time
                    + ros::Duration(CommonConf::kContactSwitchStartupTimeout)) {
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }

        if (last_contact_switch_message_ptr_ == nullptr)
        {
            ROS_ERROR("Contact switch message not received within the startup timeout");
            return FcCommsReturns::kReturnError;
        }
        else
        {
            ROS_INFO("FC Comms received initial contact switch message succesfully");
        }

    }

    return FcCommsReturns::kReturnOk;
}

// Main run loop of node.
template<class T>
FcCommsReturns CommonFcComms<T>::run()
{

    ros::Rate rate(CommonConf::kFcSensorsUpdateRateHz);

    while(ros::ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }

    // Disconnect from FC.
    return flightControlImpl_.disconnect();
}

// Attempt to arm the flight controller
template<class T>
bool CommonFcComms<T>::uavArmServiceHandler(
                iarc7_msgs::Arm::Request& request,
                iarc7_msgs::Arm::Response& response)
{
    ROS_INFO("Uav arm service handler called");

    bool auto_pilot;
    FcCommsReturns status = flightControlImpl_.isAutoPilotAllowed(auto_pilot);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to find out if auto pilot is enabled");
        return false;
    }

    // If auto pilot is not enabled we have no power
    if(!auto_pilot)
    {
        ROS_INFO("Failed to arm or disarm the FC: auto pilot is disabled");
        response.success = false;
        response.message = "disabled";
        return true;
    }

    // Now attempt to arm or disarm
    status = flightControlImpl_.setArm(request.data);
    if (status != FcCommsReturns::kReturnOk)
    {
        ROS_ERROR("iarc7_fc_comms: Failed to send arm message");
        return false;
    }

    // Check to see if the craft actually armed
    ros::Time start_time = ros::Time::now();
    while(ros::ok() && ((ros::Time::now() - start_time) < ros::Duration(CommonConf::kMaxArmDelay)))
    {
        ros::spinOnce();
        bool armed;
        FcCommsReturns status = flightControlImpl_.isArmed(armed);
        if (status == FcCommsReturns::kReturnOk) {
            if(armed == request.data)
            {
                ROS_INFO("FC arm or disarm set succesfully");

                status = flightControlImpl_.postArm(request.data);
                if(status != FcCommsReturns::kReturnOk) {
                    ROS_ERROR("iarc7_fc_comms: Post arm action failed");
                    return false;
                }

                response.success = true;
                return true;
            }
        }
        else
        {
            ROS_ERROR("Failed to retrieve flight controller arm status");
        }
        update();
    }

    ROS_INFO("Failed to arm or disarm the FC: timed out");
    response.success = false;
    response.message = "timed out";
    return true;
}

// Update flight controller arming information
template<class T>
void CommonFcComms<T>::updateArmed()
{
    bool temp_armed;
    FcCommsReturns status = flightControlImpl_.isArmed(temp_armed);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to retrieve flight controller status");
    }
    else
    {
        ROS_DEBUG("Armed: %d", temp_armed);
        fc_armed_ = temp_armed;
    }
}

// Update flight controller auto pilot status information
template<class T>
void CommonFcComms<T>::updateAutoPilotEnabled()
{
    bool temp_auto_pilot;
    FcCommsReturns status = flightControlImpl_.isAutoPilotAllowed(temp_auto_pilot);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to find out if auto pilot is enabled");
    }
    else
    {
        ROS_DEBUG("Autopilot_enabled: %d", temp_auto_pilot);
        fc_auto_pilot_enabled_ = temp_auto_pilot;
    }
}

// Update flight controller battery information
template<class T>
void CommonFcComms<T>::updateBattery()
{
    float voltage;
    FcCommsReturns status = flightControlImpl_.getBattery(voltage);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve flight controller battery info");
    }
    else
    {
        iarc7_msgs::Float64Stamped battery_msg;
        battery_msg.header.stamp = ros::Time::now();
        battery_msg.data = voltage;
        ROS_DEBUG("iarc7_fc_comms: Battery level: %f", battery_msg.data);
        battery_publisher.publish(battery_msg);
    }
}

// Update flight controller attitude information
template<class T>
void CommonFcComms<T>::updateAttitude()
{
    double attitude[3];
    ros::Time attitude_stamp = ros::Time::now();
    FcCommsReturns status = flightControlImpl_.getAttitude(attitude, attitude_stamp);

    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve attitude from flight controller");
    }
    else
    {
        if(initial_heading_as_offset_) {
            if(std::isnan(initial_heading_offset_)) {
                initial_heading_offset_ = attitude[2];
                ROS_DEBUG("Initial heading: %f", initial_heading_offset_);
            }

            double yaw = attitude[2] - initial_heading_offset_;

            // Limit to 0 and 2*pi
            if(yaw < 0.0) {
                yaw += 2.0* M_PI;
            }
            else if(yaw >= 2.0 * M_PI) {
                yaw -= 2.0 * M_PI;
            }
            attitude[2] = yaw;
        }

        ROS_DEBUG("iarc7_fc_comms: Attitude: %f %f %f",
                  attitude[0],
                  attitude[1],
                  attitude[2]);

        sendOrientation(attitude, attitude_stamp);
    }
}

// Send direction message to flight controller
template<class T>
void CommonFcComms<T>::updateDirection()
{
    FcCommsReturns status{FcCommsReturns::kReturnOk};

    if(have_new_direction_command_message_ && fc_armed_)
    {
        status = flightControlImpl_.processDirectionCommandMessage(
                     last_direction_command_message_ptr_);
        if (status == FcCommsReturns::kReturnOk)
        {
            have_new_direction_command_message_ = false;
        }
        else
        {
            ROS_ERROR("iarc7_fc_comms: Failed to send direction message");
        }
    }
}

// Get the accelerations from the IMU on the flight controller
template<class T>
void CommonFcComms<T>::updateAccelerations()
{
    FcCommsReturns status{FcCommsReturns::kReturnOk};
    double accelerations[3];
    double angular_velocities[3];

    status = flightControlImpl_.getIMU(accelerations, angular_velocities);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve IMU info from flight controller");
    }
    else
    {
        ROS_DEBUG("iarc7_fc_comms: Accelerations: %f %f %f",
                  accelerations[0],
                  accelerations[1],
                  accelerations[2]);

        ROS_DEBUG("iarc7_fc_comms: Angular Velocities: %f %f %f",
                  angular_velocities[0],
                  angular_velocities[1],
                  angular_velocities[2]);

        sendIMU(accelerations, angular_velocities);
    }
}

// Activate the safety response of the flight controller impl
template<class T>
void CommonFcComms<T>::activateFcSafety()
{
    FcCommsReturns status = flightControlImpl_.safetyLand();
    
    if (status == FcCommsReturns::kReturnOk) {
        ROS_WARN("iarc7_fc_comms: succesfully sent safety land request");
    } else {
        ROS_ERROR("iarc7_fc_comms: failed to send safety land request");
    }
}

// Attempt to reconnect, blocking until successful
template<class T>
void CommonFcComms<T>::reconnect() {
    FcCommsReturns status;
    ros::Time start_time = ros::Time::now();

    status = flightControlImpl_.disconnect();
    if(status == FcCommsReturns::kReturnOk)
    {
        status = flightControlImpl_.connect();
    }

    if(status == FcCommsReturns::kReturnOk)
    {
        ROS_INFO("iarc7_fc_comms: Succesful reconnection to flight controller");
        calibrateAccelerometer();
    }
    else
    {
        ROS_ERROR("iarc7_fc_comms: Failed reconnection to flight controller");
    }
}

// Send direction message to flight controller
template<class T>
void CommonFcComms<T>::calibrateAccelerometer()
{
    if(calibrate_accelerometer_)
    {
        if(last_contact_switch_message_ptr_->header.stamp
           > ros::Time::now() - valid_contact_switch_message_delay_)
        {
            if(last_contact_switch_message_ptr_->front
                && last_contact_switch_message_ptr_->back
                && last_contact_switch_message_ptr_->right
                && last_contact_switch_message_ptr_->left)
            {
                FcCommsReturns status{FcCommsReturns::kReturnOk};

                status = flightControlImpl_.calibrateAccelerometer();

                if (status != FcCommsReturns::kReturnOk)
                {
                    ROS_ERROR("iarc7_fc_comms: Failed to calibrate accelerometer");
                }
            }
            else
            {
                ROS_WARN("Can't calibrate accelerometer, not on ground");
            }
        }
        else
        {
            ROS_ERROR("Skipping accleration calibration. No contact message within timeout");
        }
    }
}

// Update information from flight controller and send information
template<class T>
void CommonFcComms<T>::update()
{
    ros::Time times = ros::Time::now();

    // Check the safety client before updating anything
    ROS_ASSERT_MSG(!safety_client_.isFatalActive(), "iarc7_fc_comms: fatal event from safety");

    FcCommsReturns status;

    // Do different things based on the current connection status.
    switch(flightControlImpl_.getConnectionStatus())
    {
        case FcCommsStatus::kDisconnected:
            ROS_WARN("FC_Comms disconnected");
            reconnect();
            break;

        case FcCommsStatus::kConnected:
            status = flightControlImpl_.handleComms();
            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ERROR("iarc7_fc_comms: flight controller impl could not handle comms");
            }

            // Check if we need to have a safety response
            if(safety_client_.isSafetyActive())
            {
                activateFcSafety();
            }
            else
            {
                updateDirection();
                updateAttitude();

                if(publish_imu_)
                {
                    updateAccelerations();
                }
            }

            (this->*sequenced_updates[current_sequenced_update])();
            current_sequenced_update = (current_sequenced_update + 1) % sequenced_updates.size();

            publishFcStatus();

            ROS_DEBUG("iarc7_fc_comms: Time to update FC sensors: %f", (ros::Time::now() - times).toSec());
            break;

        case FcCommsStatus::kConnecting:
            break;

        default:
            ROS_ASSERT_MSG(false, "iarc7_fc_comms: FC_Comms has undefined state.");
    }
}

template<class T>
void CommonFcComms<T>::publishFcStatus()
{
    iarc7_msgs::FlightControllerStatus status_message;

    status_message.armed = fc_armed_;
    status_message.auto_pilot = fc_auto_pilot_enabled_;
    status_message.failsafe = fc_failsafe_;

    status_publisher.publish(status_message);
}

// Send out the orientation date from the quad
template<class T>
void CommonFcComms<T>::sendOrientation(double (&attitude)[3], ros::Time attitude_stamp)
{
    iarc7_msgs::OrientationAnglesStamped orientation_msg;

    orientation_msg.header.stamp = attitude_stamp + orientation_timestamp_offset_;

    orientation_msg.data.roll = attitude[0];
    orientation_msg.data.pitch = attitude[1];
    orientation_msg.data.yaw = attitude[2];

    orientation_pub_.publish(orientation_msg);
}

// Send out the accelerations from the quad FC
template<class T>
void CommonFcComms<T>::sendIMU(double (&accelerations)[3], double (&angular_velocities)[3])
{
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = CommonConf::kTfChildName;

  imu.linear_acceleration.x = accelerations[0];
  imu.linear_acceleration.y = accelerations[1];
  imu.linear_acceleration.z = accelerations[2];

  imu.angular_velocity.x = angular_velocities[0];
  imu.angular_velocity.y = angular_velocities[1];
  imu.angular_velocity.z = angular_velocities[2];

  imu.orientation_covariance[0] = -1;
  imu.angular_velocity_covariance[0] = -1;
  imu.linear_acceleration_covariance[0] = CommonConf::kAccelerationVariance[0];
  imu.linear_acceleration_covariance[4] = CommonConf::kAccelerationVariance[1];
  imu.linear_acceleration_covariance[8] = CommonConf::kAccelerationVariance[2];

  imu_publisher.publish(imu);
}

#endif
