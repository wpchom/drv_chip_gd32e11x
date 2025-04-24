/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_flash.h
 * @brief       gd32e11x flash driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_FLASH_H__
#define __DRV_FLASH_H__

/* Include ----------------------------------------------------------------- */
#include "dev_storage.h"
#include "gd32e11x_fmc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define ------------------------------------------------------------------ */
#define DRV_FLASH_PAGE_SIZE 0x400U

/* Function ---------------------------------------------------------------- */
size_t DRV_FLASH_Sector(uintptr_t addr, size_t *align);
MDS_Err_t DRV_FLASH_Read(uintptr_t addr, uint8_t *data, size_t len, size_t *read);
MDS_Err_t DRV_FLASH_Program(uintptr_t addr, const uint8_t *data, size_t len, size_t *write);
MDS_Err_t DRV_FLASH_Erase(uintptr_t addr, size_t size, size_t *erase);

/* Driver ------------------------------------------------------------------ */
extern const DEV_STORAGE_Driver_t G_DRV_GD32E11X_FLASH;

#ifdef __cplusplus
}
#endif

#endif /* __DRV_FLASH_H__ */
