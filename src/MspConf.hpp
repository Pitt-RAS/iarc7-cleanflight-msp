#ifndef MSP_CONF_HPP
#define MSP_CONF_HPP

////////////////////////////////////////////////////////////////////////////
//
// Configuration for MSP flight controller node.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{

    struct FcCommsMspConf
    {
        static const uint32_t kBaudRate{115200};

        // Needs space on end to bug in serial package
        static constexpr char const * const kHardwareId{"USB VID:PID=10c4:ea60 SNR=0001"};

        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};

        // Non Data MSP packet length in bytes
        static const uint8_t kMspNonDataLength = 6;

        // Non Data MSP packet length in bytes
        static const uint8_t kMspPacketDataOffset = 5;

        // Longest MSP data section
        static const uint8_t kMspMaxDataLength = 127;

        // Length of packet header
        static const uint8_t kMspHeaderSize = 3;

        // Header for receiving a message
        static constexpr char const * const kMspReceiveHeader{"$M>"};

        // Header for sending a message
        static constexpr char const * const kMspSendHeader{"$M<"};

        // MSP RC scaling factors
        static constexpr const float kMspMidPoint{1500.0};
        static constexpr const float kMspPitchScale{10.0}; // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const float kMspRollScale{10.0};  // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const float kMspYawScale{10.0};   // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const float kMspThrottleScale{(2000.0-1000.0) / 100.0};  // (Max-Min)/100 e.g. scale 0% to 100% to min/max throttle

    };

}

#endif // End MSP_CONF_HPP
