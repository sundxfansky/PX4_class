// MESSAGE SENS_POWER support class

#pragma once

namespace mavlink {
namespace ASLUAV {
namespace msg {

/**
 * @brief SENS_POWER message
 *
 * Voltage and current sensor data
 */
struct SENS_POWER : mavlink::Message {
    static constexpr msgid_t MSG_ID = 201;
    static constexpr size_t LENGTH = 16;
    static constexpr size_t MIN_LENGTH = 16;
    static constexpr uint8_t CRC_EXTRA = 218;
    static constexpr auto NAME = "SENS_POWER";


    float adc121_vspb_volt; /*<  Power board voltage sensor reading in volts */
    float adc121_cspb_amp; /*<  Power board current sensor reading in amps */
    float adc121_cs1_amp; /*<  Board current sensor 1 reading in amps */
    float adc121_cs2_amp; /*<  Board current sensor 2 reading in amps */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  adc121_vspb_volt: " << adc121_vspb_volt << std::endl;
        ss << "  adc121_cspb_amp: " << adc121_cspb_amp << std::endl;
        ss << "  adc121_cs1_amp: " << adc121_cs1_amp << std::endl;
        ss << "  adc121_cs2_amp: " << adc121_cs2_amp << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << adc121_vspb_volt;              // offset: 0
        map << adc121_cspb_amp;               // offset: 4
        map << adc121_cs1_amp;                // offset: 8
        map << adc121_cs2_amp;                // offset: 12
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> adc121_vspb_volt;              // offset: 0
        map >> adc121_cspb_amp;               // offset: 4
        map >> adc121_cs1_amp;                // offset: 8
        map >> adc121_cs2_amp;                // offset: 12
    }
};

} // namespace msg
} // namespace ASLUAV
} // namespace mavlink
