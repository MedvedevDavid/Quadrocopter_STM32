#ifndef _OSFUNC_INTERFACE_HPP_
#define _OSFUNC_INTERFACE_HPP_

#include <stdint.h>

class OsFunc_Interface {
public:
  virtual bool os_event_transmitt_wait(int wait_time) = 0;
  virtual bool os_event_receive_wait(int wait_time) = 0;
  virtual bool os_delay(uint32_t delay_time) = 0;
};

#endif