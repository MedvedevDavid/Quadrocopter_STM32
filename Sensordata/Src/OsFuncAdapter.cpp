#include "OsFuncAdapter.hpp"
extern "C" {
#include "cmsis_os.h"
}

extern osEventFlagsId_t i2cDataRecievedHandle;
extern osEventFlagsId_t i2cDataTransmittedHandle;

bool OsFuncProvider ::os_event_transmitt_wait(int wait_time) {
  osEventFlagsWait(i2cDataTransmittedHandle, 0x00000001U, osFlagsWaitAny,
                   wait_time);

  return true;
}

bool OsFuncProvider ::os_event_receive_wait(int wait_time) {
  osEventFlagsWait(i2cDataRecievedHandle, 0x00000001U, osFlagsWaitAny,
                   wait_time);
  return true;
}

bool OsFuncProvider ::os_delay(uint32_t delay_time) {
  osDelay(delay_time);
  return true;
}
