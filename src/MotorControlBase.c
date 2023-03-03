
#include "MotorControlBase.h"

// static ErrFunc errFunc = NULL;

MotorControlBase* Controller_init(MotorControlBase* controller, const MotorControlBase_Init_TypeDef *config){
#if defined (HAL_TIMER) &&HAL_TIMER

    controller->timerField.pulseTimer.setPeriod(config->pulseWidth);
    controller->timerField.accTimer.setPeriod(config->accUpdatePeriod);
#else
    TimerField_setPulseWidth(&controller->timerField, config->pulseWidth);
    TimerField_setAccUpdatePeriod(&controller->timerField, config->accUpdatePeriod);
#endif
    controller->reachedTargetCallback = config->reachedTargetCallback;
    controller->errorCallback = config->errorCallback;
    controller->accUpdatePeriod = config->accUpdatePeriod;
    controller->pulseWidth = config->pulseWidth;
    controller->mCnt = 0;
    controller->leadMotor = NULL;
    controller->changeDir = 0;
    controller->lastPulse = 0;
    return controller;
}

int32_t Controller_getCurrentSpeed(const MotorControlBase *controller){
#if defined (HAL_TIMER) && HAL_TIMER
    return controller->timerField.stepTimer.frequency();
#else
    return TimerField_getStepFrequency(&controller->timerField);
#endif
}


void Controller_attachStepper(MotorControlBase *controller, uint8_t N, Stepper* *steppers){

    ASSERT(N <= MAXMOTORS);

    for(size_t i = 0; i < N; i++){
        controller->motorList[i] = steppers[i];
    }
    controller->motorList[N] = NULL;
}

void vController_attachStepper(MotorControlBase *controller, uint8_t N, ...){
    ASSERT(N <= MAXMOTORS);

    va_list mlist;

    va_start(mlist, N);

    for(size_t i = 0; i < N; i++){
        controller->motorList[i] = (Stepper *)va_arg(mlist, Stepper*);
    }
    va_end(mlist);

    controller->motorList[N] = NULL;
}

void vvController_attachStepper(MotorControlBase *controller, uint8_t N, __builtin_va_list va){
    ASSERT(N <= MAXMOTORS);

    for(size_t i = 0; i < N; i++){
        controller->motorList[i] = (Stepper *)va_arg(va, Stepper*);
    }
    controller->motorList[N] = NULL;
}

// void attachErrorFunction(ErrFunc ef){
//     errFunc = ef;
// }

// void Error(ErrCode e){
//     if (errFunc != NULL) errFunc(e);
// }

void FUN_IN_RAM stepTimerISR(MotorControlBase *controller){

    if(controller->leadMotor == NULL) return;
    
    if(Stepper_isClearStepPin(controller->leadMotor) == false) return;
    Stepper_doStep(controller->leadMotor);  /// leadMotor=MotorList[0]
    Stepper* *slave = controller->motorList;
    
    // move slave motors if required (https://en.wikipedia.org/wiki/Bresenham)
    while(*(++slave) != NULL){  // Skip MotorList[0]
        if((*slave)->B >= 0){
            Stepper_doStep(*slave);
            (*slave)->B -= controller->leadMotor->A;
        }
        (*slave)->B += (*slave)->A;
    }
    
    controller->timerField.pulseTimer.setStatus(1);
#if(ALLOCAT_PULSE_TIMER)
    TimerField_triggerDelay(&controller->timerField);  // start delay line to dactivate all step pins
#endif
    // stop timer and call callback if we reached MOTOR_TARGET
    if((controller->mode == MOTOR_TARGET) && 
       (controller->leadMotor->current == controller->leadMotor->target)){
#if(ALLOCAT_PULSE_TIMER)
        TimerField_stepTimerStop(&controller->timerField);
#endif
#if defined (HAL_TIMER) && HAL_TIMER
        controller->lastPulse = 1;
        controller->timerField.accTimer.setStatus(0);
#else
        // TimerField_timerEndAfterPulse(&controller->timerField);
        controller->lastPulse = 1;
        TimerField_accTimerStop(&controller->timerField);
#endif
        // printf("motor will stop\n");
        // if(!controller->reachedTargetCallback) return;
        // controller->reachedTargetCallback((int32_t)controller->leadMotor->current);
    }
}


void FUN_IN_RAM pulseTimerISR(MotorControlBase *controller){
    Stepper* *motor = controller->motorList;
    
    if(controller->leadMotor == NULL) return;

    while((*motor) != NULL){
        Stepper_clearStepPin((*motor++));
    }

    if(controller->changeDir){
        motor = controller->motorList;
        while((*motor) != NULL){
            Stepper_toggleDir((*motor++));
        }
        controller->changeDir = 0;
    }
    controller->timerField.pulseTimer.setStatus(0);
    if(controller->lastPulse){
        int32_t pos = 0;
#if defined (HAL_TIMER) && HAL_TIMER
        controller->timerField.stepTimer.setStatus(0);
        controller->timerField.pulseTimer.setStatus(0);
        controller->timerField.accTimer.setStatus(0);
        // printf("step stop\n");
#else
        TimerField_end(&controller->timerField);
#endif
        controller->lastPulse = 0;
        pos = controller->leadMotor->current;
        controller->leadMotor = NULL;

        if(controller->mode != MOTOR_TARGET){
            return;
        }
        // printf("motor stop\n");
        if(!controller->reachedTargetCallback){
            return;
        }
        controller->reachedTargetCallback((int32_t)pos);
    }
}


