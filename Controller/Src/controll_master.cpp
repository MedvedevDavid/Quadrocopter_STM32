#include "controll_master.hpp"
#include "parameters.hpp"

ControllSignal_data_type
ControllMessageManeger::decode_new_message(uint8_t *new_message_Buffer,
                                           uint8_t buffer_size) {

  uint16_t ibus_data[10];

  if (!ibus_is_valid(new_message_Buffer) ||
      !ibus_checksum(new_message_Buffer)) {
    controll_message_failure_count++;
    if (controll_message_failure_count > 10) {
      state_return.state = Controll_state::stop;
    } else {
    }
    return (state_return);
  } else {
    if (controll_message_failure_count > 0) {
      controll_message_failure_count--;
    }
  }
  for (int ch_index = 0, bf_index = 2; ch_index < 10;
       ch_index++, bf_index += 2) {
    ibus_data[ch_index] =
        new_message_Buffer[bf_index + 1] << 8 | new_message_Buffer[bf_index];
  }

  state_return.acceleration = MAXIMUM_ACCELERATION *
                              (ibus_data[2] - STICK_VALUE_AT_MINIMUM) /
                              STICK_VALUE_FROM_MIN_TO_MAX;
  state_return.angle_pitch = MAXIMUM_CONTROLL_ANGLE *
                             (int32_t(ibus_data[1]) - STICK_VALUE_AT_MIDDLE) /
                             STICK_VALUE_FROM_MIN_TO_MAX_HALF;
  state_return.angle_roll = MAXIMUM_CONTROLL_ANGLE *
                            (int32_t(ibus_data[0]) - STICK_VALUE_AT_MIDDLE) /
                            STICK_VALUE_FROM_MIN_TO_MAX_HALF;
  state_return.angle_yaw = MAXIMUM_CONTROLL_ANGLE *
                           (int32_t(ibus_data[3]) - STICK_VALUE_AT_MIDDLE) /
                           STICK_VALUE_FROM_MIN_TO_MAX_HALF;

  if (ibus_data[4] < STICK_VALUE_AT_MIDDLE &&
      ibus_data[5] > STICK_VALUE_AT_MIDDLE &&
      ibus_data[6] < STICK_VALUE_AT_MIDDLE &&
      ibus_data[7] > STICK_VALUE_AT_MIDDLE) {
    state_return.state = Controll_state::calib;
  } else if (ibus_data[4] > STICK_VALUE_AT_MIDDLE) {
    state_return.state = Controll_state::start;
  } else {
    state_return.state = Controll_state::stop;
  }

  return (state_return);
}

bool ControllMessageManeger::ibus_is_valid(uint8_t *ibus_raw_data) {
  // is it ibus?
  return (ibus_raw_data[0] == IBUS_LENGTH &&
          ibus_raw_data[1] == IBUS_COMMAND40);
}

bool ControllMessageManeger::ibus_checksum(uint8_t *ibus_raw_data) {
  uint16_t checksum_cal = 0xffff;
  uint16_t checksum_ibus;

  for (int i = 0; i < 30; i++) {
    checksum_cal -= ibus_raw_data[i];
  }

  checksum_ibus =
      ibus_raw_data[31] << 8 | ibus_raw_data[30]; // checksum value from ibus
  return (checksum_ibus == checksum_cal);
}