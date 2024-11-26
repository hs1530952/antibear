#pragma once

typedef enum {
    EEPROM_AT24C02 = 0,
    EEPROM_COUNT,
} eeprom_e;

typedef struct eepromGeometry_s {
    uint16_t pageSize; // In bytes
    uint32_t totalSize;
} eepromGeometry_t;

bool eepromInit(void);

bool eepromIsReady(void);
bool eepromWaitForReady(void);
void eepromProgram(uint32_t address, const uint8_t *buffer, uint16_t length);
int eepromReadBytes(uint32_t address, uint8_t *buffer, uint16_t length);
const eepromGeometry_t *eepromGetGeometry(void);
