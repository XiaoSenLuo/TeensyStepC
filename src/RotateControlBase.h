/*** 
 * @Author: xiaosenluo xiaosenluo@yandex.com
 * @Date: 2023-02-06 20:44:11
 * @LastEditors: xiaosenluo xiaosenluo@yandex.com
 * @LastEditTime: 2023-02-12 14:52:17
 * @FilePath: \wch_ch32v203_TeensyStepC\Components\TeensyStepC\src\RotateControlBase.h
 * @Description: 
 * @
 * @Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _ROTATECONTROLBASE_H__
#define _ROTATECONTROLBASE_H__

#include "MotorControlBase.h"
#include "accelerators/LinRotAccelerator.h"
#include "accelerators/SinRotAccelerator.h"

typedef struct {
    MotorControlBase_Init_TypeDef motorConfig;
    TimerField_InitTypeDef timerConfig;
    CallbackFunc controllerStopCallback;
} RotateControl_Init_TypeDef;

#define LINE_ACCELERATOR           0
#define SIN_ACCELERATOR            1

typedef struct {
    struct  {
        uint32_t stop : 1;
        uint32_t acc : 3;
    };
    CallbackFunc controllerStopCallback;
    MotorControlBase controller;
    LinRotAccelerator accelerator;
    SinRotAccelerator sinRotAccelerator;
}RotateControl;


void RotateControl_init(RotateControl *controller, const RotateControl_Init_TypeDef *config);

// Non-blocking movements ----------------

void RotateControl_rotateAsync(RotateControl *_controller, float speedFactor, uint8_t N, Stepper* *steppers);
void RotateControl_rotateAsync2(RotateControl *_controller, int selete, 
                                float speedFactor, uint8_t N, Stepper* *steppers);

void vRotateControl_rotateAsync(RotateControl *_controller, float speedFactor, uint8_t N, ...);

void RotateControl_stopAsync(RotateControl *controller);

// Blocking movements --------------------
void RotateControl_stop(RotateControl *controller);

void RotateControl_overrideSpeed(RotateControl *controller, int32_t newSpeed);
void RotateControl_overrideAcceleration(RotateControl *controller, float accFac);

void RotateControl_accTimerISR(RotateControl *_controller);

#endif
