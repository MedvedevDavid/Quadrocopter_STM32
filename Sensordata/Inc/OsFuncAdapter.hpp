#ifndef _OSFUNCADPTER_HPP_
#define _OSFUNCADPTER_HPP_

#include "OsFunc_interface.hpp"

class OsFuncProvider : public OsFunc_Interface {
public:
  bool os_event_transmitt_wait(int wait_time) override;
  bool os_event_receive_wait(int wait_time) override;
  bool os_delay(uint32_t delay_time) override;
};

#endif