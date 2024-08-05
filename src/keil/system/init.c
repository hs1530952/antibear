#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/system.h"

#include "system/tasks.h"

#include "system/init.h"

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void init(void)
{
    systemInit();

    // Initialize task data as soon as possible. Has to be done before tasksInit()
    // and any init code that may try to modify task behaviour before tasksInit().
    tasksInitData();

    tasksInit();

    systemState |= SYSTEM_STATE_READY;
}
