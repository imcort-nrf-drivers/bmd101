#ifndef __BMD101_H__
#define __BMD101_H__

#include "transfer_handler.h"

void bmd101_begin(void);
void bmd101_sleep(void);
int16_t bmd101_getECG(void);
uint8_t bmd101_getHeartRate(void);
uint8_t bmd101_getSignalRate(void);

#endif
