#pragma once

typedef enum {
    SYSTEM_STATE_INITIALISING   = 0,
    SYSTEM_STATE_READY          = (1 << 7)
} systemState_e;

extern uint8_t systemState;

void init(void);
