#include "main.h"

// implements a simple parameter store in the last flash page

// write parameter array to persistent flash memory
// there is no wear levelling implmented
int param_set_u64(uint64_t *pvalue, uint32_t count);

// read parameter array from persistent flash memory
// there is no validation of flash content implemented
int param_get_u64(uint64_t *pvalue, uint32_t count);
