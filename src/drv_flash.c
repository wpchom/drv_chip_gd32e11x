/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_flash.c
 * @brief       gd32e11x flash driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_flash.h"
#include "drv_chip.h"

/* Function ---------------------------------------------------------------- */
static void FLASH_Unlock(void)
{
    if ((FMC_CTL & FMC_CTL_LK) != 0x00) {
        FMC_KEY = UNLOCK_KEY0;
        FMC_KEY = UNLOCK_KEY1;
    }
}

static void FLASH_Lock(void)
{
    FMC_CTL |= FMC_CTL_LK;
}

static MDS_Err_t FLASH_WaitForLastOperation(uint32_t timeout)
{
    MDS_Err_t err = MDS_EBUSY;

    do {
        if ((FMC_STAT & FMC_STAT_BUSY) != 0x00) {
            timeout -= 1;
            continue;
        }
        if ((FMC_STAT & (FMC_STAT_WPERR | FMC_STAT_PGERR | FMC_STAT_PGAERR)) != 0x00) {
            err = MDS_EIO;
        } else {
            err = MDS_EOK;
        }
        break;
    } while (timeout > 0);

    return (err);
}

static uint32_t FLASH_GetAlignPadData(const uint8_t *data, size_t len)
{
    if (len >= sizeof(uint32_t)) {
        return (*(uint32_t *)data);
    }

    union {
        uint8_t dat8[sizeof(uint32_t)];
        uint32_t dat32;
    } val = {.dat32 = 0xFFFFFFFFUL};

    for (size_t i = 0; i < len; i++) {
        val.dat8[i] = data[i];
    }

    return (val.dat32);
}

size_t DRV_FLASH_Sector(uintptr_t addr, size_t *align)
{
    if (align != NULL) {
        *align = addr - (addr % DRV_FLASH_PAGE_SIZE);
    }

    return (DRV_FLASH_PAGE_SIZE);
}

MDS_Err_t DRV_FLASH_Read(uintptr_t addr, uint8_t *data, size_t len, size_t *read)
{
    size_t size = MDS_MemBuffCopy(data, len, (uint8_t *)addr, len);
    if (read != NULL) {
        *read = size;
    }

    return ((size == len) ? (MDS_EOK) : (MDS_EIO));
}

MDS_Err_t DRV_FLASH_Program(uintptr_t addr, const uint8_t *data, size_t len, size_t *write)
{
    MDS_Err_t err;
    size_t cnt = 0;

    FLASH_Unlock();
    for (err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT); (err == MDS_EOK) && (cnt < len);
         cnt += sizeof(uint32_t)) {
        FMC_STAT = FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR;

        FMC_CTL |= FMC_CTL_PG;

        uint32_t val = FLASH_GetAlignPadData(data + cnt, len - cnt);
        REG32(addr + cnt) = val;

        err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT);

        FMC_CTL &= ~FMC_CTL_PER;
    }
    FLASH_Lock();

    if (write != NULL) {
        *write = cnt;
    }

    return (err);
}

MDS_Err_t DRV_FLASH_Erase(uintptr_t addr, size_t size, size_t *erase)
{
    size_t ofs = addr;
    MDS_Err_t err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT);

    FLASH_Unlock();
    while (err == MDS_EOK) {
        uintptr_t align = 0;
        uintptr_t sector = DRV_FLASH_Sector(ofs, &align);
        if ((align != ofs) || (sector > size)) {
            break;
        }

        FMC_STAT = FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR;

        FMC_CTL |= FMC_CTL_PER;
        FMC_ADDR = align;
        FMC_CTL |= FMC_CTL_START;
        __DSB();
        __ISB();

        err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT);
        if (err == MDS_EOK) {
            ofs += sector;
            size -= sector;
        }

        FMC_CTL &= ~FMC_CTL_PER;
    }
    FLASH_Lock();

    if (erase != NULL) {
        *erase = (ofs - addr);
    }

    return (err);
}

/* Driver ------------------------------------------------------------------ */
static MDS_Err_t DDRV_FLASH_Control(const DEV_STORAGE_Adaptr_t *storage, MDS_Item_t cmd,
                                    MDS_Arg_t *arg)
{
    MDS_ASSERT(storage != NULL);

    UNUSED(arg);

    switch (cmd) {
        case MDS_DEVICE_CMD_INIT:
        case MDS_DEVICE_CMD_DEINIT:
        case MDS_DEVICE_CMD_HANDLESZ:
            return (MDS_EOK);
        case MDS_DEVICE_CMD_OPEN:
        case MDS_DEVICE_CMD_CLOSE:
            return (MDS_EOK);
    }

    return (MDS_EACCES);
}

static MDS_Err_t DDRV_FLASH_Read(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs, uint8_t *buff,
                                 size_t len, size_t *read)
{
    return (DRV_FLASH_Read(periph->object.baseAddr + ofs, buff, len, read));
}

static MDS_Err_t DDRV_FLASH_Write(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs,
                                  const uint8_t *buff, size_t len, size_t *write)
{
    return (DRV_FLASH_Program(periph->object.baseAddr + ofs, buff, len, write));
}

static MDS_Err_t DDRV_FLASH_Erase(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs, size_t size,
                                  size_t *erase)
{
    return (DRV_FLASH_Erase(periph->object.baseAddr + ofs, size, erase));
}

static size_t DDRV_FLASH_Sector(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs, size_t *align)
{
    return (DRV_FLASH_Sector(periph->object.baseAddr + ofs, align));
}

const DEV_STORAGE_Driver_t G_DRV_GD32E11X_FLASH = {
    .control = DDRV_FLASH_Control,
    .read = DDRV_FLASH_Read,
    .write = DDRV_FLASH_Write,
    .erase = DDRV_FLASH_Erase,
    .sector = DDRV_FLASH_Sector,
};
