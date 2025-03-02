#ifndef __CONTROLL_MASTER_HPP
#define __CONTROLL_MASTER_HPP

#include "data_types.hpp"

class ControllMessageManeger
{
    private:
        uint8_t bits_til_start =0;
        uint8_t bits_til_stop =0;
        ControllSignal_data_type state_return;
        uint8_t controll_message_failure_count = 0;
    public:
        ControllSignal_data_type decode_new_message(uint8_t *new_message_Buffer, uint8_t buffer_size);
        bool ibus_is_valid(uint8_t *ibus_raw_data);
        bool ibus_checksum(uint8_t *ibus_raw_data);

};

#endif // __CONTROLL_MASTER_HPP