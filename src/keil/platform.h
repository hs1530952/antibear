#pragma once

#define NOINLINE __attribute__((noinline))

// MCU specific platform from drivers/mcu/XX
#include "platform_mcu.h"

#include "target/common_post.h"
