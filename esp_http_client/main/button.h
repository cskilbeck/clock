//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

void button_init();
uint32 button_update();

//////////////////////////////////////////////////////////////////////

static bool btn_held(uint32 button_events, int button_id)
{
    return ((button_events >> (button_id * 4)) & 1) != 0;
}

static bool btn_pressed(uint32 button_events, int button_id)
{
    return ((button_events >> (button_id * 4)) & 2) != 0;
}

static bool btn_released(uint32 button_events, int button_id)
{
    return ((button_events >> (button_id * 4)) & 4) != 0;
}
