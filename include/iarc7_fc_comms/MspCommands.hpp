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
        static const uint8_t data_length{0};

        static constexpr char const * const string_name{"MSP_RC"};

        uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        void getRc(uint16_t (&rc_values)[FcCommsMspConf::kMspReceivableChannels])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            // Jetson sends back 18 channels, two bytes each.
            for(int i = 0; i < FcCommsMspConf::kMspReceivableChannels; i++)
            {
                rc_values[i] = response[i*2] | (response[(i*2)+1] << 8);
            }
        }
    };

    struct MSP_SET_RAW_RC
    {
        // Default constructor an destructor
        MSP_SET_RAW_RC() = default;
        ~MSP_SET_RAW_RC() = default;

        // Don't allow the copy constructor or assignment.
        MSP_SET_RAW_RC(const MSP_SET_RAW_RC& rhs) = delete;
        MSP_SET_RAW_RC& operator=(const MSP_SET_RAW_RC& rhs) = delete;

        static const uint8_t message_id{200};

        // 8 channels 2 bytes each
        static const uint8_t data_length{FcCommsMspConf::kMspSettableChannels*2};

        static constexpr char const * const string_name{"MSP_SET_RAW_RC"};

        uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        void packRc(uint16_t (&rc_values)[FcCommsMspConf::kMspSettableChannels])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&rc_values[0]);

            // channels are two bytes each
            std::copy(data_ptr,
                      data_ptr + 2*FcCommsMspConf::kMspSettableChannels,
                      send);
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

    struct MSP_ATTITUDE
    {
        // Default constructor an destructor
        MSP_ATTITUDE() = default;
        ~MSP_ATTITUDE() = default;

        // Don't allow the copy constructor or assignment.
        MSP_ATTITUDE(const MSP_ATTITUDE& rhs) = delete;
        MSP_ATTITUDE& operator=(const MSP_ATTITUDE& rhs) = delete;

        static const uint8_t message_id{108};
        static const uint8_t data_length{0};

        static constexpr char const * const string_name{"MSP_ATTITUDE"};

        uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        // Returns the attitude in terms of roll pitch and yaw in radians
        void getAttitude(double (&attitude_values)[3])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            // Jetson sends back 18 channels, two bytes each.
            // Attitude is always 3 angles (e.g. roll, pitch, and yaw)
            for(uint32_t i = 0; i < 3; i++)
            {
                // Unpack the value
                uint16_t unpacked_value = response[i*2] | (response[(i*2)+1] << 8);

                // Reinterpret as signed
                int16_t* temp = reinterpret_cast<int16_t*>(&unpacked_value);

                // Convert to doubles
                attitude_values[i] = static_cast<double>(*temp);
            }

            // Roll and pitch are in tenths of degrees, heading is in degrees
            attitude_values[0] = attitude_values[0] / 10.0 * M_PI / 180.0;

            // Invert pitch to match right handed coordinate system
            attitude_values[1] = -1.0 * attitude_values[1] / 10.0 * M_PI / 180.0;
            attitude_values[2] = attitude_values[2] * M_PI / 180.0;
        }
    };

    struct MSP_RAW_IMU
    {
        // Default constructor an destructor
        MSP_RAW_IMU() = default;
        ~MSP_RAW_IMU() = default;

        // Don't allow the copy constructor or assignment.
        MSP_RAW_IMU(const MSP_RAW_IMU& rhs) = delete;
        MSP_RAW_IMU& operator=(const MSP_RAW_IMU& rhs) = delete;

        static const uint8_t message_id{102};
        static const uint8_t data_length{0};

        static constexpr char const * const string_name{"MSP_RAW_IMU"};

        uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];

        // Returns the IMU values in m/s^2
        void getAcc(double (&acc_values)[3])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            // Jetson sends back 9 IMU data integers. 16bits each.
            for(uint32_t i = 0; i < 3; i++)
            {
                // Unpack the value
                uint16_t unpacked_value = response[i*2] | (response[(i*2)+1] << 8);

                // Reinterpret as signed
                int16_t* temp = reinterpret_cast<int16_t*>(&unpacked_value);

                // Convert to doubles
                // IARC 2016-2017 is using an spf3 evo, which has an MPU9250,
                // which has an mpu6500 in it
                // In cleanflight accgyro_mpu6500.c in function mpu6500AccInit
                // The scale for 1g of acceleration for is set to 512 * 4
                // We are also using a hacked up version of cleanflight that does not
                // apply this scaling factor using a bitshift hack before sending so
                // we need to do it here.
                acc_values[i] = 9.8 * static_cast<double>(*temp)/(512.0 * 4.0);
            }
        }

        // Returns the Gyro values
        void getGyro(double (&gyro_values)[3])
        {
            // Jetson runs in little endian mode and the FC
            // Receives data in big endian
            // Jetson sends back 9 IMU data integers. 16bits each.
            for(uint32_t i = 0; i < 3; i++)
            {
                // Unpack the value
                uint16_t unpacked_value = response[6+i*2] | (response[6+(i*2)+1] << 8);

                // Reinterpret as signed
                int16_t* temp = reinterpret_cast<int16_t*>(&unpacked_value);

                // Convert to doubles
                gyro_values[i] = static_cast<double>(*temp) * (1.0/16.4) * (M_PI / 180.0);

            }
        }
    };

    struct MSP_ACC_CALIBRATION
    {
        // Default constructor an destructor
        MSP_ACC_CALIBRATION() = default;
        ~MSP_ACC_CALIBRATION() = default;

        // Don't allow the copy constructor or assignment.
        MSP_ACC_CALIBRATION(const MSP_ACC_CALIBRATION& rhs) = delete;
        MSP_ACC_CALIBRATION& operator=(const MSP_ACC_CALIBRATION& rhs) = delete;

        static const uint8_t message_id{205};
        static const uint8_t data_length{0};

        static constexpr char const * const string_name{"MSP_ACC_CALIBRATION"};

        const uint8_t send[FcCommsMspConf::kMspMaxDataLength]={};

        uint8_t response[FcCommsMspConf::kMspMaxDataLength];
    };
}

#endif
