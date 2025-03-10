#ifndef _LINROTACCELERATOR_H__
#define _LINROTACCELERATOR_H__


#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "../port/port.h"

typedef uint8_t LinRotAcceleratorParamBase;

typedef struct {
    float v_tgt, v_cur;
    float v_tgt_orig, dv_orig, dv_cur, dv;
}LinRotAccelerator;


int32_t RotAccelerator_prepareRotation(LinRotAccelerator *_accelerator, int32_t currentPosition, int32_t targetSpeed, uint32_t a, uint32_t accUpdatePeriod, float speedFactor);


int32_t RotAccelerator_overrideSpeed(LinRotAccelerator *_accelerator, float factor);

int32_t RotAccelerator_overrideAcceleration(LinRotAccelerator *_accelerator, float factor);

int32_t FUN_IN_RAM RotAccelerator_updateSpeed(LinRotAccelerator *_accelerator, int32_t curPos);

int32_t RotAccelerator_initiateStopping(LinRotAccelerator *_accelerator, int32_t curPos);


#endif
