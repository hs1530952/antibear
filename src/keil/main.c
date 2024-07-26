#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

void run(void);

int main(void)
{
    run();

    return 0;
}

void FAST_CODE run(void)
{
    while (true) {
        __NOP();
    }
}
