#ifndef _MOTORCONTROLBASE_H__
#define _MOTORCONTROLBASE_H__

#include "ErrorHandler.h"
#include "Stepper.h"
#include "port/timer/TF_Handler.h"
#include "port/port.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>

#ifndef MAXMOTORS
#define MAXMOTORS         2
#endif

#ifndef ACCUPDATEPERIOD_DEFAULT
#define ACCUPDATEPERIOD_DEFAULT               5
#endif

#ifndef PULSEWIDTH_DEFAULT
#define PULSEWIDTH_DEFAULT                    5000
#endif



typedef enum {
    err_OK,
    err_movment_not_possible,
    err_too_much_motors,
    err_positive_limit,
    err_negative_limit,
    err_initialize_move,
    err_running,
}ErrCode;

typedef enum {
    MOTOR_TARGET = 0,
    MOTOR_NOTARGET = 1
}Mode;

typedef void (*ErrFunc)(Stepper*, int);
typedef void (*CallbackFunc)(int32_t);

typedef struct MotorControlBase_Def{
    volatile struct {
        int32_t OK : 2;
        int32_t mCnt : 8;
        int32_t mode : 2;
        int32_t changeDir : 2;
        int32_t lastPulse : 2;
    };

    uint32_t accUpdatePeriod;
    uint32_t pulseWidth;

    TimerField timerField;

    Stepper* motorList[MAXMOTORS + 1];
    Stepper* leadMotor;

    CallbackFunc reachedTargetCallback;
    ErrFunc errorCallback;
}MotorControlBase;


typedef struct {
    uint32_t accUpdatePeriod;
    uint32_t pulseWidth;
    CallbackFunc reachedTargetCallback;
    ErrFunc errorCallback;

}MotorControlBase_Init_TypeDef;

// void Error(ErrCode e);
// void attachErrorFunction(ErrFunc ef);

// static inline mcErr err(mcErr code){
//     return (mcErr)error(eM_MC, (int)code);
// }


static inline bool Controller_isOK(const MotorControlBase * controller){

    return controller->OK;
}

MotorControlBase* Controller_init(MotorControlBase* controller, const MotorControlBase_Init_TypeDef *config);

static inline bool Controller_isRunning(const MotorControlBase *controller){
#if defined (HAL_TIMER) && HAL_TIMER
    return controller->timerField.stepTimer.isRunning() || controller->timerField.pulseTimer.isRunning();
#else
    return TimerField_stepTimerIsRunning(&controller->timerField) ||
           TimerField_pulseTimerIsRunning(&controller->timerField);
#endif
}

static inline bool Controller_isAllocated(const MotorControlBase *controller){
#if defined (HAL_TIMER) && HAL_TIMER
    return 1;
#else
    return TimerField_stepTimerIsAllocated(&controller->timerField);
#endif
}

int32_t Controller_getCurrentSpeed(const MotorControlBase *controller);

static inline void Controller_emergencyStop(MotorControlBase *controller){
#if defined (HAL_TIMER) && HAL_TIMER
    controller->lastPulse = 1;
    controller->timerField.stepTimer.setStatus(0);
    controller->timerField.pulseTimer.setStatus(0);
    controller->timerField.accTimer.setStatus(0);
    
#else
    TimerField_end(&controller->timerField);
#endif
}

static inline void Controller_stop(MotorControlBase* controller){
    Controller_emergencyStop(controller);
}

void Controller_attachStepper(MotorControlBase *controller, uint8_t N, Stepper* *steppers);

void vController_attachStepper(MotorControlBase *controller, uint8_t N, ...);

void vvController_attachStepper(MotorControlBase *controller, uint8_t N, __builtin_va_list va);



void FUN_IN_RAM stepTimerISR(MotorControlBase *controller);

void FUN_IN_RAM pulseTimerISR(MotorControlBase *controller);


#endif
