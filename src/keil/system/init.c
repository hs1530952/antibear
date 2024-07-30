#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/system.h"

#include "system/init.h"

void init(void)
{
    systemInit();
}
