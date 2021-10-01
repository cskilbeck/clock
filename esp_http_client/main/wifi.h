#pragma once


// enum wifi_event_bits
// {
//     wifi_event_connected = 1,
//     wifi_event_scan_complete = 2,
//     wifi_event_connecting = 4,
//     wifi_event_scanning = 8,
//     wifi_event_any = 15,

//     wifi_event_time_set = 16
// };

// enum wifi_startup_action
// {
//     wifi_action_connect = 1,
//     wifi_action_scan = 2
// };

// enum wifi_state_enum
// {
//     wifi_state_invalid = -1,
//     wifi_state_idle = 0,
//     wifi_state_connecting = 1,
//     wifi_state_scanning = 2,
//     wifi_state_connected = 3
// };

namespace wifi
{
    extern ip4_addr_t ip_address;
    extern ip4_addr_t gateway;
    extern ip4_addr_t subnet_mask;

    constexpr int CONNECTED = BIT0;
    constexpr int SMARTCONFIG_DONE = BIT1;

    void init();
    uint32 wait_until(uint32 bits, int timeout);

}    // namespace wifi
