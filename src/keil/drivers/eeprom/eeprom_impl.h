#pragma once

struct eepromVTable_s;

typedef struct eepromDevice_s {
    //
    // members to be configured by the eeprom chip implementation
    //

    const struct eepromVTable_s *vTable;
    eepromGeometry_t geometry;

    extDevice_t *dev;
} eepromDevice_t;

typedef struct eepromVTable_s {
    bool (*isReady)(eepromDevice_t *edevice);
    bool (*waitForReady)(eepromDevice_t *edevice);

    void (*program)(eepromDevice_t *edevice, uint32_t address, const uint8_t *data, uint16_t length);

    int (*readBytes)(eepromDevice_t *edevice, uint32_t address, uint8_t *buffer, uint16_t length);

    const eepromGeometry_t *(*getGeometry)(eepromDevice_t *edevice);
} eepromVTable_t;