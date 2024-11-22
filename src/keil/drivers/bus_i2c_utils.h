#pragma once

/*
 * common functions shared by different I2C implementations
 */

bool i2cUnstick(pinDef_t *scl, pinDef_t *sda);
