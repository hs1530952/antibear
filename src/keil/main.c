#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "system/init.h"

void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        __NOP();
    }
}
