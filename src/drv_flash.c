/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_flash.c
 * @brief       gd32e11x flash driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_flash.h"
#include "drv_chip.h"
#include "mds_log.h"

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

MDS_Err_t DRV_FLASH_Erase(uintptr_t addr, size_t blks, size_t *erase)
{
    MDS_Err_t err;
    size_t cnt = 0;

    FLASH_Unlock();
    for (err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT); (err == MDS_EOK) && (cnt < blks); cnt++) {
        FMC_STAT = FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR;

        FMC_CTL |= FMC_CTL_PER;
        FMC_ADDR = addr + (DRV_FLASH_PAGE_SIZE * cnt);
        FMC_CTL |= FMC_CTL_START;
        __DSB();
        __ISB();

        err = FLASH_WaitForLastOperation(FMC_TIMEOUT_COUNT);

        FMC_CTL &= ~FMC_CTL_PER;
    }
    FLASH_Lock();

    if (erase != NULL) {
        *erase = cnt;
    }

    return (err);
}

/* Driver ------------------------------------------------------------------ */
static MDS_Err_t DDRV_FLASH_Control(const DEV_STORAGE_Adaptr_t *storage, MDS_Item_t cmd, MDS_Arg_t *arg)
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

static size_t DDRV_FLASH_BlockSize(const DEV_STORAGE_Adaptr_t *storage, size_t block)
{
    UNUSED(storage);
    UNUSED(block);

    return (DRV_FLASH_PAGE_SIZE);
}

static MDS_Err_t DDRV_FLASH_Read(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs, uint8_t *buff, size_t len,
                                 size_t *read)
{
    size_t cnt = MDS_MemBuffCopy(buff, len, (uint8_t *)(periph->object.baseAddr + ofs), len);

    if (read != NULL) {
        *read = cnt;
    }

    return (MDS_EOK);
}

static MDS_Err_t DDRV_FLASH_Program(const DEV_STORAGE_Periph_t *periph, uintptr_t ofs, const uint8_t *buff, size_t len,
                                    size_t *write)
{
    return (DRV_FLASH_Program(periph->object.baseAddr + ofs, buff, len, write));
}

static MDS_Err_t DDRV_FLASH_Erase(const DEV_STORAGE_Periph_t *periph, size_t blk, size_t nums, size_t *erase)
{
    return (DRV_FLASH_Erase(periph->object.baseAddr + blk, nums, erase));
}

const DEV_STORAGE_Driver_t G_DRV_GD32E11X_FLASH = {
    .control = DDRV_FLASH_Control,
    .blksize = DDRV_FLASH_BlockSize,
    .read = DDRV_FLASH_Read,
    .prog = DDRV_FLASH_Program,
    .erase = DDRV_FLASH_Erase,
};
