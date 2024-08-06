#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/i2c.h"
#include "drivers/time.h"

#include "drivers/eeprom/eeprom_impl.h"
#include "drivers/eeprom/eeprom.h"
#include "drivers/eeprom/eeprom_at24c02.h"

#define AT24C02_I2C_ADDRESS         0x50

#define AT24C02_WRITE_DELAY_MAX_MS  5

// Device size parameters
#define AT24C02_PAGE_SIZE           32
#define AT24C02_PAGE_BYTE_SIZE      8
#define AT24C02_MAX_BYTES           (AT24C02_PAGE_SIZE * AT24C02_PAGE_BYTE_SIZE)

static bool at24c02_isReady(void)
{
    return !i2cBusy(I2CDEV_1, NULL);
}

static bool at24c02_hasTimeOut(void)
{
    uint32_t nowMs = microsISR() / 1000;
    if (cmp32(nowMs, AT24C02_WRITE_DELAY_MAX_MS) >= 0) {
        return true;
    }
    return false;
}

static bool at24c02_waitForReady(void)
{
    bool ready = true;
    while (!at24c02_isReady()) {
        if (at24c02_hasTimeOut()) {
            ready = false;
            break;
        }
    }

    return ready;
}

static void at24c02_program(uint32_t address, const uint8_t *data, uint16_t length)
{
    uint8_t currentPage;
    uint8_t currentPageIndex;
    uint8_t pageDataCount;
    uint8_t offset;

    currentPage = address / AT24C02_PAGE_BYTE_SIZE;
    currentPageIndex = address % AT24C02_PAGE_BYTE_SIZE;

    if (length + currentPageIndex > AT24C02_PAGE_BYTE_SIZE) {
        pageDataCount = AT24C02_PAGE_BYTE_SIZE - currentPageIndex;
    } else {
        pageDataCount = length;
    }

    offset = 0;

    i2cWriteBuffer(I2CDEV_1, AT24C02_I2C_ADDRESS, (currentPage * AT24C02_PAGE_BYTE_SIZE + currentPageIndex), pageDataCount, (uint8_t *)&data[offset]);
    delay(AT24C02_WRITE_DELAY_MAX_MS);
    offset += pageDataCount;
    length -= pageDataCount;
    currentPage++;

    while (length > 0) {
        pageDataCount = length;
        if (pageDataCount > AT24C02_PAGE_BYTE_SIZE) {
            pageDataCount = AT24C02_PAGE_BYTE_SIZE;
        }

        i2cWriteBuffer(I2CDEV_1, AT24C02_I2C_ADDRESS, (currentPage * AT24C02_PAGE_BYTE_SIZE + currentPageIndex), pageDataCount, (uint8_t *)&data[offset]);
        delay(AT24C02_WRITE_DELAY_MAX_MS);
        offset += pageDataCount;
        length -= pageDataCount;
        currentPage++;
    }
}

static int at24c02_readBytes(uint32_t address, uint8_t *buffer, uint16_t length)
{
    i2cReadBuffer(I2CDEV_1, AT24C02_I2C_ADDRESS, address, length, buffer);

    return length < AT24C02_MAX_BYTES ? length : AT24C02_MAX_BYTES;
}

const eepromGeometry_t *at24c02_getGeometry(eepromDevice_t *edevice)
{
    return &edevice->geometry;
}

const eepromVTable_t at24c02_vTable = {
    .isReady = at24c02_isReady,
    .waitForReady = at24c02_waitForReady,
    .program = at24c02_program,
    .readBytes = at24c02_readBytes,
    .getGeometry = at24c02_getGeometry,
};

bool at24c02_init(eepromDevice_t *edevice)
{
    edevice->geometry.pageSize = AT24C02_PAGE_BYTE_SIZE;
    edevice->geometry.totalSize = AT24C02_MAX_BYTES;

    edevice->vTable = &at24c02_vTable;

    return true;
}