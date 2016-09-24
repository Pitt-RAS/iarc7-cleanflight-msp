#ifndef MSP_FC_COMMS_HPP
#define MSP_FC_COMMS_HPP

////////////////////////////////////////////////////////////////////////////
//
// Msp flight controller node.
//
// Implements node behaviours specific to MSP FC
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <string>
#include "CommonConf.hpp"
#include "iarc7_msgs/UavControl.h"
#include "iarc7_msgs/UavThrottle.h"
#include "serial/serial.h"

namespace FcComms
{
    // Inherit from CommonFcComms to get the stuff any Fc is required to do.
    class MspFcComms
    {
    public:
        MspFcComms();
        ~MspFcComms();

        // Used to find and connect to the serial port
        FcCommsReturns connect();

        // Disconnect from FC, should be called before destructor.
        FcCommsReturns disconnect();

        // Handle periodically updating polled info.
        FcCommsReturns handleComms();

        // Get the flight status of the FC.
        FcCommsReturns getStatus(uint8_t& armed, uint8_t& auto_pilot, uint8_t& failsafe);

        // Get the battery voltage of the FC.
        FcCommsReturns getBattery(float& voltage);

        // Getter for current connection status
        inline const FcCommsStatus getConnectionStatus()
        {
            return fc_comms_status_;
        }

        // Send the flight controller RX values
        void sendFcAngles(const iarc7_msgs::UavControl::ConstPtr& message);
        void sendFcThrottle(const iarc7_msgs::UavThrottle::ConstPtr& message);

    private:
        // Don't allow the copy constructor or assignment.
        MspFcComms(const MspFcComms& rhs) = delete;
        MspFcComms& operator=(const MspFcComms& rhs) = delete;

        // Find the FC from a list of serial ports using its hardware ID.
        FcCommsReturns findFc(std::string& serial_port);

        // Connect to the serial port and identify FC.
        FcCommsReturns connectFc();

        // Send the rc commands to the FC using the member array of rc values.
        FcCommsReturns sendRc();

        // Send message using the MSP protocol
        template<typename T>
        FcCommsReturns sendMessage(T& message);


        // Serial object used to communicate with FC
        serial::Serial* fc_serial_;

        // State of communication with flight controller
        FcCommsStatus fc_comms_status_ = FcCommsStatus::kDisconnected;

        // FC implementation specific to hold intermediate rc values
        uint16_t translated_rc_values_[8]{0};
    };
} // End namspace

#endif
