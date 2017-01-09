#ifndef COMMON_CONF_HPP
#define COMMON_CONF_HPP

////////////////////////////////////////////////////////////////////////////
//
// Configuration Common to all flight controllers.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{

// Used for the result of a command
enum class FcCommsReturns
{
    kReturnOk,
    kReturnError
};

// Used for the current state of communication with flight controller
enum class FcCommsStatus
{
    kDisconnected,
    kConnected,
    kConnecting
};

struct CommonConf
{
    static constexpr const float kFcSensorsUpdateRateHz{30};

    static constexpr const double kMinAllowedRoll = -0.25;
    static constexpr const double kMaxAllowedRoll = 0.25;
    static constexpr const double kMinAllowedPitch = -0.25;
    static constexpr const double kMaxAllowedPitch = 0.25;
    static constexpr const double kMinAllowedThrottle = 0;
    static constexpr const double kMaxAllowedThrottle = 1;
    static constexpr const double kMinAllowedYawRate = -0.5;
    static constexpr const double kMaxAllowedYawRate = 0.5;

    static constexpr const char* kTfParentName{"level_quad"};
    static constexpr const char* kTfChildName{"quad"};

};

} // namespace FcComms

#endif // End COMMON_CONF_HPP
