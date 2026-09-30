#include "GCS.h"

#include "AP_ServoRelayEvents/AP_ServoRelayEvents.h"

#if defined(HAL_BOREALIS_BENCH_TEST)
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;
extern bool borealis_bench_ready;
void borealis_bench_output_timeout();
static uint8_t borealis_bench_channel;
static uint32_t borealis_bench_last_command_ms;

void borealis_bench_output_timeout()
{
    if (borealis_bench_channel != 0 &&
        AP_HAL::millis() - borealis_bench_last_command_ms >= 2500) {
        hal.rcout->disable_ch(borealis_bench_channel - 1);
        hal.rcout->force_safety_on();
        borealis_bench_channel = 0;
    }
}
#endif

#if AP_MAVLINK_SERVO_RELAY_ENABLED

MAV_RESULT GCS_MAVLINK::handle_servorelay_message(const mavlink_command_int_t &packet)
{
    AP_ServoRelayEvents *handler = AP::servorelayevents();
    if (handler == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (packet.command) {
    case MAV_CMD_DO_SET_SERVO:
#if defined(HAL_BOREALIS_BENCH_TEST)
        if (AP_BoardConfig::in_config_error()) {
            if (!borealis_bench_ready || packet.param1 < 1 || packet.param1 > 6) {
                return MAV_RESULT_DENIED;
            }
            const uint8_t channel = uint8_t(packet.param1);
            const uint16_t pwm = uint16_t(packet.param2);
            if (fabsf(packet.param1 - channel) > 0.001f ||
                fabsf(packet.param2 - pwm) > 0.001f) {
                return MAV_RESULT_DENIED;
            }
            // The motor-connected throttle is restricted to low pulses;
            // servo rows retain their small movement test range.
            if ((channel == 1 && (pwm < 1000 || pwm > 1150)) ||
                (channel != 1 && (pwm < 1300 || pwm > 1700))) {
                return MAV_RESULT_DENIED;
            }
            if (borealis_bench_channel != 0 && borealis_bench_channel != channel) {
                hal.rcout->disable_ch(borealis_bench_channel - 1);
            }
            hal.rcout->enable_ch(channel - 1);
            hal.rcout->force_safety_off();
            hal.rcout->write(channel - 1, pwm);
            borealis_bench_channel = channel;
            borealis_bench_last_command_ms = AP_HAL::millis();
            return MAV_RESULT_ACCEPTED;
        }
#endif
        if (handler->do_set_servo(packet.param1, packet.param2)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        if (handler->do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4 * 1000)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

#if AP_RELAY_ENABLED
    case MAV_CMD_DO_SET_RELAY:
        if (handler->do_set_relay(packet.param1, packet.param2)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        if (handler->do_repeat_relay(packet.param1, packet.param2, packet.param3 * 1000)) {
            result = MAV_RESULT_ACCEPTED;
        }
        break;
#endif

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    return result;
}

#endif  // AP_MAVLINK_SERVO_RELAY_ENABLED
