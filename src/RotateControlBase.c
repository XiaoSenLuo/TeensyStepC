

#include "RotateControlBase.h"
#include "port/timer/wch//TimerField.h"

// Implementation *************************************************************************************************

#define CHECK(res)    { if(!(res)) return; }

static void doRotate(RotateControl *_controller, int N, float speedFactor)
{
    MotorControlBase *controller = (MotorControlBase*)&_controller->controller;
    LinRotAccelerator *acclerator = (LinRotAccelerator *)&_controller->accelerator;
    
    if(Controller_isRunning(controller))
    {
        // this->err(mcErr::alrdyMoving);
        if(!controller->errorCallback) return;
        controller->errorCallback(controller->leadMotor, err_running);
        return;
    }

    //Calculate Bresenham parameters ----------------------------------------------------------------
    sort_element(controller->motorList, controller->motorList + N, Stepper_cmpVmax);
    controller->leadMotor = controller->motorList[0];

    if (controller->leadMotor->vMax == 0)
        return;

    controller->leadMotor->currentSpeed = 0;

    controller->leadMotor->A = labs(controller->leadMotor->vMax);
    for (int i = 1; i < N; i++)
    {
        controller->motorList[i]->A = labs(controller->motorList[i]->vMax);
        controller->motorList[i]->B = 2 * controller->motorList[i]->A - controller->leadMotor->A;
    }
    uint32_t acceleration = (*(find_min_element(controller->motorList, controller->motorList + N, Stepper_cmpAcc)))->a; // use the lowest acceleration for the move

    // Start moving----------------------------------------------------------------------------------------------
    _controller->stop = 0;
#if defined (HAL_TIMER) && HAL_TIMER

#else
    TimerField_begin(&controller->timerField);
#endif
    controller->lastPulse = 0;
    if(_controller->acc == LINE_ACCELERATOR){
        RotAccelerator_prepareRotation(acclerator,
                                            controller->leadMotor->current, 
                                            controller->leadMotor->vMax, acceleration, 
                                            controller->accUpdatePeriod, speedFactor);
        controller->leadMotor->targetSpeed = acclerator->v_tgt;
    }else if(_controller->acc == SIN_ACCELERATOR){
        SinRotAccelerator_prepareRotation(&_controller->sinRotAccelerator, 
                                            controller->leadMotor->current, 
                                            controller->leadMotor->vMax, acceleration, speedFactor);
        controller->leadMotor->targetSpeed = _controller->sinRotAccelerator.v_tgt;

    }else{
        return;
    }
#if defined (HAL_TIMER) && HAL_TIMER
    controller->timerField.accTimer.setStatus(1);
#else
    TimerField_accTimerStart(&controller->timerField);
#endif
}


void RotateControl_init(RotateControl *controller, const RotateControl_Init_TypeDef *config)
{
#if defined (HAL_TIMER) && HAL_TIMER
    // controller->controller.timerField.stepTimer = *config->timerConfig.stepTimer;
    // controller->controller.timerField.accTimer = *config->timerConfig.accTimer;
    // controller->controller.timerField.pulseTimer = *config->timerConfig.pulseTimer;
    
    controller->controller.timerField.stepTimer.initialize();
    controller->controller.timerField.pulseTimer.initialize();
    controller->controller.timerField.accTimer.initialize();
#else
    TimerField_init(&controller->controller.timerField, &config->timerConfig);
#endif

    Controller_init(&controller->controller, &config->motorConfig);
    controller->controllerStopCallback = config->controllerStopCallback;
    controller->acc = LINE_ACCELERATOR;
    controller->controller.mode = MOTOR_NOTARGET;
}


// ISR -----------------------------------------------------------------------------------------------------------


void RotateControl_accTimerISR(RotateControl *_controller)
{

    if(_controller->controller.leadMotor == NULL) return;

    MotorControlBase *controller = (MotorControlBase*)&_controller->controller;
    LinRotAccelerator *acceletator = (LinRotAccelerator*)&_controller->accelerator;
#if defined (HAL_TIMER) && HAL_TIMER
    
#else

#endif

    int32_t newSpeed = 0;
    if(_controller->acc == LINE_ACCELERATOR){
        // get new speed for the leading motor
        newSpeed = RotAccelerator_updateSpeed(acceletator, controller->leadMotor->current); 
    }else if(_controller->acc == SIN_ACCELERATOR){
        newSpeed = SinRotAccelerator_updateSpeed(&_controller->sinRotAccelerator, controller->leadMotor->current);
    }else{
        newSpeed = 0;
    }
    
    if((newSpeed == 0) && _controller->stop)
    {
#if defined (HAL_TIMER) && HAL_TIMER
        controller->timerField.accTimer.setStatus(0);
        controller->lastPulse = 1;
#else
        TimerField_accTimerStop(&controller->timerField);
        controller->lastPulse = 1;
#endif
        _controller->stop = 0;
        if(!_controller->controllerStopCallback) return;
        _controller->controllerStopCallback(newSpeed);
        return;
    }

    if (controller->leadMotor->currentSpeed == newSpeed)
    {
        return; // nothing changed, just keep running
    }

    int dir = newSpeed >= 0 ? 1 : -1; // direction changed? -> toggle direction of all motors
    if ((dir != controller->leadMotor->dir) && !controller->changeDir)
    {
#if(0)
        Stepper** motor = controller->motorList;
        while ((*motor) != NULL)
        {
            // (*motor++)->toggleDir();
        }

        delayMicroseconds(controller->pulseWidth); // dir signal need some lead time
#else
        // printf("dir=%ld\n", dir);
        controller->changeDir = 1;
#endif
    }
#if defined (HAL_TIMER) && HAL_TIMER

    if(!controller->timerField.stepTimer.isRunning()){
        controller->timerField.stepTimer.setStatus(1);
    }
    controller->timerField.stepTimer.setFrequency(labs(newSpeed));
#else
    TimerField_setStepFrequency(&controller->timerField, labs(newSpeed)); // speed changed, update timer
    if(controller->timerField.stepTimerRunning == 0){
        TimerField_stepTimerStart(&controller->timerField);
    }
#endif
    controller->leadMotor->currentSpeed = newSpeed;
    // printf("%ld\n", newSpeed);
    if(newSpeed == controller->leadMotor->targetSpeed){
        if(!controller->reachedTargetCallback) return;
        controller->reachedTargetCallback(newSpeed);
    }
}

// ROTATE Commands -------------------------------------------------------------------------------

void vRotateControl_rotateAsync(RotateControl *_controller, float speedFactor, uint8_t N, ...)
{
    MotorControlBase *controller = &_controller->controller;
    va_list mlist;

    va_start(mlist, N);
    vvController_attachStepper(controller, N, mlist);
    va_end(mlist);

    doRotate(_controller, N, speedFactor);
}


void RotateControl_rotateAsync(RotateControl *_controller, float speedFactor, uint8_t N, Stepper* *steppers)
{
    MotorControlBase *controller = &_controller->controller;

    Controller_attachStepper(controller, N, steppers);

    doRotate(_controller, N, speedFactor);
}

void RotateControl_rotateAsync2(RotateControl *_controller, int selete, 
                                float speedFactor, uint8_t N, Stepper* *steppers){
    CHECK((selete == LINE_ACCELERATOR) || (selete == SIN_ACCELERATOR));
    _controller->acc = selete;
    RotateControl_rotateAsync(_controller, speedFactor, N, steppers);
}

void RotateControl_overrideSpeed(RotateControl *controller, int32_t newSpeed)
{
    // isStopping = false;
    controller->stop = 0;
    float factor = 1.0f;
    LinRotAccelerator *accelerator = &controller->accelerator;
    if(controller->acc == LINE_ACCELERATOR){
        factor = (float)newSpeed / (float)accelerator->v_tgt_orig;
        controller->controller.leadMotor->targetSpeed = RotAccelerator_overrideSpeed(accelerator, factor);
    }else if(controller->acc == SIN_ACCELERATOR){
        factor = (float)newSpeed / (float)accelerator->v_tgt_orig;
        controller->controller.leadMotor->targetSpeed = SinRotAccelerator_overrideSpeed(
                                    &controller->sinRotAccelerator, 
                                    factor, 
                                    controller->controller.leadMotor->current);
    }else{
        return;
    }
}


void RotateControl_overrideAcceleration(RotateControl *controller, float factor)
{
    LinRotAccelerator *accelerator = &controller->accelerator;
    if(controller->acc == LINE_ACCELERATOR){
        RotAccelerator_overrideAcceleration(accelerator, factor);
    }
}


void RotateControl_stopAsync(RotateControl *_controller)
{
    MotorControlBase *controller = &_controller->controller;
    LinRotAccelerator *accelerator = &_controller->accelerator;
    
    if(_controller->stop) return;
    _controller->stop = 1;
    if(_controller->acc == LINE_ACCELERATOR){
        RotAccelerator_initiateStopping(accelerator, controller->leadMotor->current);
    }else if(_controller->acc == SIN_ACCELERATOR){
        SinRotAccelerator_initiateStopping(&_controller->sinRotAccelerator, 
                                            controller->leadMotor->current);
    }else{
        return;
    }

    
}

void RotateControl_stop(RotateControl *_controller)
{
    RotateControl_stopAsync(_controller);
    while (_controller->stop)
    {
        delay(1);
    }
}

