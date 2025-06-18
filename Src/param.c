#include "param.h"

#include <string.h>

#define PAGE_SIZE 2048

// symbol _param must be defined in .ld file and is then provided by linker
extern uint64_t _param[PAGE_SIZE/sizeof(uint64_t)];

int param_set_u64(uint64_t *pvalue, uint32_t count) {
    FLASH->KEYR = 0x45670123;   // flash unlock
    FLASH->KEYR = 0xCDEF89AB;   // flash unlock
    while(FLASH->SR & FLASH_SR_BSY);
    uint8_t page = ((uint32_t)(_param)-FLASH_BASE) / PAGE_SIZE;

    // page erase
    FLASH->CR |= FLASH_CR_PER;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | (page<<FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PER;

    // standard programming
    uint64_t* dst = (uint64_t*)(FLASH_BASE + page*PAGE_SIZE);
    FLASH->CR |= FLASH_CR_PG;
    for(int i=0; i<count; i++) {
        *dst++ = *pvalue++;
        while(FLASH->SR & FLASH_SR_BSY);
    }
    while(FLASH->SR & FLASH_SR_EOP);
    FLASH->CR &= ~FLASH_CR_PG;

    FLASH->CR |= FLASH_CR_LOCK;
    return 0;
}

int param_get_u64(uint64_t *pvalue, uint32_t count) {
    memcpy(pvalue, _param, count*sizeof(uint64_t));
    return 0;
}
