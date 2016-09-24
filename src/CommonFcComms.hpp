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
#include <ros/ros.h>
#include "CommonConf.hpp"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "iarc7_msgs/UavControl.h"
#include "iarc7_msgs/UavThrottle.h"
#include "std_msgs/Float32.h"

namespace FcComms{

    template<class T>
    class CommonFcComms
    {
    public:
        // Used to make sure this class remains a singleton.
        static CommonFcComms<T>& getInstance();

        FcCommsReturns init();
        
        FcCommsReturns run(int argc, char **argv);

        //Delete copy constructors, assignment operator
        CommonFcComms(const CommonFcComms& rhs) = delete;
        CommonFcComms& operator=(const CommonFcComms& rhs) = delete;

    private:

        // Class can only be made by class
        CommonFcComms() = default;

        // This nodes handle.
        ros::NodeHandle nh_; 

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;

        // Publish to the FC sensor topics
        void publishTopics();

        // Callback to update the sensors on the FC
        void updateSensors(const ros::TimerEvent&);

        // Send FC angles
        void sendFcAngles(const iarc7_msgs::UavControl::ConstPtr& message);

        // Send FC throttle
        void sendFcThrottle(const iarc7_msgs::UavThrottle::ConstPtr& message);
        // Just use the default constructor
        T flightControlImpl_;

        // Subscriber for UavAngle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for UavThrottle valuess
        ros::Subscriber uav_throttle_subscriber;

    };
}

using namespace FcComms;

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
    // Should error check or something

    battery_publisher = nh_.advertise<std_msgs::Float32>("fc_battery", 50);
    status_publisher = nh_.advertise<iarc7_msgs::FlightControllerStatus>("fc_status", 50);
    uav_angle_subscriber = nh_.subscribe("uav_angles", 100, &T::sendFcAngles, &flightControlImpl_);
    uav_throttle_subscriber = nh_.subscribe("uav_throttle", 100, &T::sendFcThrottle, &flightControlImpl_);

    ROS_INFO("FC Comms registered and subscribed to topics");

    return FcCommsReturns::kReturnOk;
}

// Main run loop of node.
template<class T>
FcCommsReturns CommonFcComms<T>::run(int argc, char **argv)
{

    // Set up a 10hz Timer to call updateSensors
    ros::Timer timer = nh_.createTimer(ros::Duration(CommonConf::kFcSensorsUpdatePeriod), &CommonFcComms::updateSensors, this);

    // Wait till we get told to stop and service all callbacks
    ros::spin();

    // Disconnect from FC.
    //TODO handle if something went wrong disconnecting.
    #pragma GCC warning "TODO handle failure"
    (void)flightControlImpl_.disconnect();

    return FcCommsReturns::kReturnOk;
}

// Push the sensor data to the appropriate topics.
template<class T>
void CommonFcComms<T>::publishTopics()
{
    iarc7_msgs::FlightControllerStatus fc;
    #pragma GCC warning "TODO handle failure"
    flightControlImpl_.getStatus(fc.armed, fc.auto_pilot, fc.failsafe);
    
    std_msgs::Float32 battery;
    #pragma GCC warning "TODO handle failure"
    flightControlImpl_.getBattery(battery.data);

    ROS_INFO("Armed: %d", fc.armed);
    status_publisher.publish(fc);
    ROS_INFO("Battery level: %f", battery.data);
    battery_publisher.publish(battery);
}

// Update the sensors on the flight controller
template<class T>
void CommonFcComms<T>::updateSensors(const ros::TimerEvent&)
{
    // Do different things based on the current connection status.
    switch(flightControlImpl_.getConnectionStatus())
    {
        case FcCommsStatus::kDisconnected:
            ROS_WARN("FC_Comms disconnected");
            // We don't care about the return value we just reconnect.
            (void)flightControlImpl_.connect();
            break;
        
        case FcCommsStatus::kConnected:
            ROS_INFO("FC_comms updating FC sensors");
            #pragma GCC warning "TODO handle failure"
            flightControlImpl_.handleComms();
            publishTopics();
            break;

        case FcCommsStatus::kConnecting:
            break;

        default:
            ROS_ASSERT_MSG(false, "FC_Comms has undefined state.");
    }
}

#endif
