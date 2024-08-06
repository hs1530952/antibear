#pragma once

struct eepromVTable_s;

typedef struct eepromGeometry_s {
    uint16_t pageSize; // In bytes
    uint32_t totalSize;
} eepromGeometry_t;

typedef struct eepromDevice_s {
    //
    // members to be configured by the eeprom chip implementation
    //

    const struct eepromVTable_s *vTable;
    eepromGeometry_t geometry;
} eepromDevice_t;

typedef struct eepromVTable_s {
    bool (*isReady)(void);
    bool (*waitForReady)(void);

    void (*program)(uint32_t address, const uint8_t *data, uint16_t length);

    int (*readBytes)(uint32_t address, uint8_t *buffer, uint16_t length);

    const eepromGeometry_t *(*getGeometry)(eepromDevice_t *edevice);
} eepromVTable_t;