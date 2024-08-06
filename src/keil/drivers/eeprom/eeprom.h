#pragma once

typedef enum {
    EEPROM_ID_AT24C02 = 0,
    EEPROM_ID_COUNT,
} eepromId_e;

void eepromInit(void);

bool eepromIsReady(eepromId_e id);
bool eepromWaitForReady(eepromId_e id);
void eepromProgram(eepromId_e id, uint32_t address, const uint8_t *buffer, uint16_t length);
int eepromReadBytes(eepromId_e id, uint32_t address, uint8_t *buffer, uint16_t length);
const eepromGeometry_t *eepromGetGeometry(eepromId_e id);