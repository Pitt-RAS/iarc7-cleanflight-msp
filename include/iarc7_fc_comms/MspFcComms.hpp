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
#include "MspConf.hpp"
#include "serial/serial.h"

#include "iarc7_msgs/BoolStamped.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace FcComms
{
    // Inherit from CommonFcComms to get the stuff any Fc is required to do.
    class MspFcComms
    {
    public:
        MspFcComms();
        ~MspFcComms() = default;

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

        // Find out if the FC is in failsafe
        FcCommsReturns  __attribute__((warn_unused_result))
            isFailsafe(bool& failsafe);

        // Find out if the FC has enabled auto_pilot
        FcCommsReturns  __attribute__((warn_unused_result))
            isAutoEnabled(bool& auto_pilot);

        // Get the battery voltage of the FC.
        FcCommsReturns  __attribute__((warn_unused_result))
            getBattery(float& voltage);

        // Get the attitude of the FC in the order roll pitch yaw
        FcCommsReturns  __attribute__((warn_unused_result))
            getAttitude(double (&attitude)[3]);

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
            processArmMessage(
                const iarc7_msgs::BoolStamped::ConstPtr& message);

        FcCommsReturns  __attribute__((warn_unused_result))
            printRawRC();

        FcCommsReturns  __attribute__((warn_unused_result))
            isAutoPilotAllowed(bool& allowed);

        FcCommsReturns  __attribute__((warn_unused_result))
            safetyLand();
        
    private:
        // Don't allow the copy constructor or assignment.
        MspFcComms(const MspFcComms& rhs) = delete;
        MspFcComms& operator=(const MspFcComms& rhs) = delete;

        // Find the FC from a list of serial ports using its hardware ID.
        FcCommsReturns  __attribute__((warn_unused_result))
            findFc(std::string& serial_port);

        // Connect to the serial port and identify FC.
        FcCommsReturns  __attribute__((warn_unused_result))
            connectFc();

        FcCommsReturns  __attribute__((warn_unused_result))
            getRawRC(
                uint16_t (&rc_values)[FcCommsMspConf::kMspReceivableChannels]);


        // Send the rc commands to the FC using the member array of rc values.
        FcCommsReturns  __attribute__((warn_unused_result))
            sendRc();

        // Send message using the MSP protocol
        template<typename T>
        FcCommsReturns  __attribute__((warn_unused_result))
            sendMessage(T& message);

        // Receive response using the MSP protocol
        FcCommsReturns  __attribute__((warn_unused_result))
            receiveResponseAfterSend(
                uint8_t packet_id,
                uint8_t (&response)[FcCommsMspConf::kMspMaxDataLength]);

        // Serial object used to communicate with FC
        std::unique_ptr<serial::Serial> fc_serial_;

        // State of communication with flight controller
        FcCommsStatus fc_comms_status_ = FcCommsStatus::kDisconnected;

        // FC implementation specific to hold intermediate rc values
        uint16_t translated_rc_values_[8]{0};
    };
} // End namspace

#endif
