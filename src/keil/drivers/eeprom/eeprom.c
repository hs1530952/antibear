#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus_i2c.h"

#include "drivers/eeprom/eeprom_impl.h"
#include "drivers/eeprom/eeprom.h"
#include "drivers/eeprom/eeprom_at24c02.h"

static eepromDevice_t eepromDevice[EEPROM_ID_COUNT];

bool eepromIsReady(eepromId_e id)
{
    return eepromDevice[id].vTable->isReady();
}

bool eepromWaitForReady(eepromId_e id)
{
    return eepromDevice[id].vTable->waitForReady();
}

void eepromProgram(eepromId_e id, uint32_t address, const uint8_t *buffer, uint16_t length)
{
    eepromDevice[id].vTable->program(address, buffer, length);
}

int eepromReadBytes(eepromId_e id, uint32_t address, uint8_t *buffer, uint16_t length)
{
    return eepromDevice[id].vTable->readBytes(address, buffer, length);
}

const eepromGeometry_t *eepromGetGeometry(eepromId_e id)
{
    return eepromDevice[id].vTable->getGeometry(&eepromDevice[id]);
}

void eepromInit(void)
{
    for (uint8_t index = 0; index < EEPROM_ID_COUNT; index++) {
        switch (index) {
            case EEPROM_ID_AT24C02: {
                at24c02_init(&eepromDevice[index]);
            } break;
        }
    }
}