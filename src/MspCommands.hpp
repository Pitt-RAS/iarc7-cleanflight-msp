#ifndef MSP_COMMANDS_HPP
#define MSP_COMMANDS_HPP

////////////////////////////////////////////////////////////////////////////
//
// Msp command packets
//
////////////////////////////////////////////////////////////////////////////
#include "MspConf.hpp"

namespace FcComms
{
    struct MSP_IDENT
    {
        // Default constructor an destructor
        MSP_IDENT() = default;
        ~MSP_IDENT() = default;

        // Don't allow the copy constructor or assignment.
        MSP_IDENT(const MSP_IDENT& rhs) = delete;
        MSP_IDENT& operator=(const MSP_IDENT& rhs) = delete;

        static const uint8_t message_id{100};
        static const uint8_t data_length{0};
        static const bool has_response{true};

        static constexpr char const * const string_name{"MSP_IDENT"};

        const uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];
    };

    struct MSP_RC
    {
        // Default constructor an destructor
        MSP_RC() = default;
        ~MSP_RC() = default;

        // Don't allow the copy constructor or assignment.
        MSP_RC(const MSP_RC& rhs) = delete;
        MSP_RC& operator=(const MSP_RC& rhs) = delete;

        static const uint8_t message_id{105};
        static const uint8_t data_length{8*2}; // 8 channels 2 bytes each
        static const bool has_response{false};

        static constexpr char const * const string_name{"MSP_RC"};

        uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        void packRc(uint16_t (&rc_values)[8])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            uint8_t* data_ptr = reinterpret_cast<uint8_t*>(rc_values[0]);
            send[0] = data_ptr[1];
            send[0] = data_ptr[0];
            send[0] = data_ptr[3];
            send[0] = data_ptr[2];
            send[0] = data_ptr[5];
            send[0] = data_ptr[4];
            send[0] = data_ptr[7];
            send[0] = data_ptr[6];
            send[0] = data_ptr[9];
            send[0] = data_ptr[8];
            send[0] = data_ptr[11];
            send[0] = data_ptr[10];
            send[0] = data_ptr[13];
            send[0] = data_ptr[12];
            send[0] = data_ptr[15];
            send[0] = data_ptr[14];
        }
    };

    struct MSP_ANALOG
    {
        // Default constructor an destructor
        MSP_ANALOG() = default;
        ~MSP_ANALOG() = default;

        // Don't allow the copy constructor or assignment.
        MSP_ANALOG(const MSP_ANALOG& rhs) = delete;
        MSP_ANALOG& operator=(const MSP_ANALOG& rhs) = delete;

        static const uint8_t message_id{110};
        static const uint8_t data_length{0};
        static const bool has_response{true};

        static constexpr char const * const string_name{"MSP_ANALOG"};

        const uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        float getVolts()
        {
            return static_cast<float>(response[0]/10.0);
        }
    };

    struct MSP_STATUS
    {
        // Default constructor an destructor
        MSP_STATUS() = default;
        ~MSP_STATUS() = default;

        // Don't allow the copy constructor or assignment.
        MSP_STATUS(const MSP_STATUS& rhs) = delete;
        MSP_STATUS& operator=(const MSP_STATUS& rhs) = delete;

        static const uint8_t message_id{101};
        static const uint8_t data_length{0};
        static const bool has_response{true};

        static constexpr char const * const string_name{"MSP_STATUS"};

        const uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        bool getArmed()
        {
            uint32_t flags{0};
            flags |= static_cast<uint32_t>(response[6]);
            flags |= static_cast<uint32_t>(response[7]) << 8;
            flags |= static_cast<uint32_t>(response[8]) << 16;
            flags |= static_cast<uint32_t>(response[9]) << 24;

            // The armed flag happens to be the first one
            return flags & 0x1;
        }
    };
}

#endif