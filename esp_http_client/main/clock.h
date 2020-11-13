#pragma once

//////////////////////////////////////////////////////////////////////

enum clock_state_enum
{
    clock_state_idle,
    clock_state_initializing,
    clock_state_ready
};

enum clock_event_t
{
    clock_minute = 1,
    clock_hour = 2
};

//////////////////////////////////////////////////////////////////////

void clock_task(void *pvParameters);
