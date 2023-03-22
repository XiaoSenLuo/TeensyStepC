

#include "StepControlBase.h"
#include "port/timer/wch/TimerField.h"



static void doMove(StepControl *_controller, int N, float speedOverride){
    MotorControlBase *controller = &_controller->controller;

    LinStepAccelerator* accelerator = &_controller->accelerator;

    //Calculate Bresenham parameters -------------------------------------
    sort_element(controller->motorList, controller->motorList + N, Stepper_cmpDelta); // The motor which does most steps leads the movement, move to top of list

    controller->leadMotor = controller->motorList[0];

    for (int i = 1; i < N; i++)
    {
        controller->motorList[i]->B = 2 * controller->motorList[i]->A - controller->leadMotor->A;
    }

    // Calculate acceleration parameters --------------------------------
    uint32_t pullInSpeed = controller->leadMotor->vPullIn;
    uint32_t pullOutSpeed = controller->leadMotor->vPullOut;

    uint32_t acceleration = (*(find_min_element(controller->motorList, controller->motorList + N, Stepper_cmpAcc)))->a; // use the lowest acceleration for the move

    uint32_t targetSpeed = labs((*(find_min_element(controller->motorList, controller->motorList + N, Stepper_cmpVmin)))->vMax) * speedOverride; // use the lowest max frequency for the move, scale by relSpeed
    
    // printf("doMove targetSpeed: %ldHz, leadMotor->A = %ld\n", targetSpeed, controller->leadMotor->A);
    if((controller->leadMotor->A == 0) || (targetSpeed == 0)) return;
    
    // MOTOR_TARGET speed----
#ifdef MATH_TYPE
    {
        _iq x = 0, relDist = 0;
        _iq leadSpeed = 0;
        leadSpeed = _IQ(labs(controller->leadMotor->vMax));
        for(int i = 0; i < N; i++){
            relDist = _IQdiv(_IQ(controller->motorList[i]->A), 
                             _IQdiv(_IQmpy(_IQ(controller->leadMotor->A), 
                             leadSpeed), 
                             _IQ(labs(controller->motorList[i]->vMax))));
            if(relDist > x) x = relDist;
        }
        targetSpeed = _IQint(_IQdiv(leadSpeed, x));
    }

#else
    float x = 0;
    float leadSpeed = labs(controller->leadMotor->vMax);
    for (int i = 0; i < N; i++)
    {
        float relDist = controller->motorList[i]->A / (float)controller->leadMotor->A * leadSpeed / labs(controller->motorList[i]->vMax);
        if (relDist > x) x = relDist;
        // Serial.printf("%d %f\n", i, relDist);
    }
    targetSpeed = leadSpeed / x;
    controller->leadMotor->targetSpeed = targetSpeed;
#endif
    //Serial.printf("\n%d\n",targetSpeed);

    // Start move--------------------------
    
    int32_t freq = Accelerator_prepareMovement(accelerator, controller->leadMotor->current, controller->leadMotor->target, targetSpeed, pullInSpeed, pullOutSpeed, acceleration);
    
    if(freq < 0){
        if(!controller->errorCallback) return;
        controller->errorCallback(controller->leadMotor, err_initialize_move);
        return;
    }
    // printf("doMove: %ldHz\n", freq);
    controller->lastPulse = 0;
#if defined (HAL_TIMER) && HAL_TIMER
    controller->timerField.stepTimer.setFrequency(freq);
    controller->timerField.stepTimer.setStatus(1);
    controller->timerField.pulseTimer.setStatus(1);
    controller->timerField.accTimer.setStatus(1);
#else
    
    TimerField_begin(&controller->timerField);
    TimerField_setStepFrequency(&controller->timerField, freq);
    TimerField_stepTimerStart(&controller->timerField);
    TimerField_accTimerStart(&controller->timerField);
#endif
}


void StepControl_init(StepControl *controller, const StepControl_Init_TypeDef *config){
    // TODO
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
    controller->controller.mode = MOTOR_TARGET;
}


void StepControl_moveAsync(StepControl *_controller, float speedOverride, uint8_t N, Stepper* *steppers){
    MotorControlBase *controller = &_controller->controller;
    
    Controller_attachStepper(controller, N, steppers);
    doMove(_controller, N, speedOverride);
}

void vStepControl_moveAsync(StepControl *_controller, float speedOverride, uint8_t N, ...){
    MotorControlBase *controller = &_controller->controller;
    va_list mlist;
#if(0)
    Stepper* stepperArr[MAXMOTORS] = { NULL };
    va_start(mlist, N);
    for(int i = 0; i < N; i++){
        stepperArr[i] = va_arg(mlist, Stepper *);
    }
    va_end(mlist);
    Controller_attachStepper(controller, N, stepperArr);
#else
    va_start(mlist, N);
    vvController_attachStepper(controller, N, mlist);
    va_end(mlist);
#endif
    doMove(_controller, N, speedOverride);
}

// non blocking stop command
void StepControl_stopAsync(StepControl *_controller){
    MotorControlBase *controller = &_controller->controller;
    LinStepAccelerator *accelerator = &_controller->accelerator;

    if(Controller_isRunning(controller)){
        uint32_t newTarget = Accelerator_initiateStopping(accelerator, controller->leadMotor->current);

        controller->leadMotor->target = controller->leadMotor->current + controller->leadMotor->dir * newTarget;

        if(controller->leadMotor->target == controller->leadMotor->current){
#if defined (HAL_TIMER) && HAL_TIMER
            controller->timerField.stepTimer.setStatus(0);
            controller->timerField.accTimer.setStatus(0);
            controller->timerField.pulseTimer.setStatus(0);
            controller->lastPulse = 0;
#else
            TimerField_end(&controller->timerField);
            controller->lastPulse = 0;
#endif
        }
    }
    
}


void StepControl_move(StepControl *_controller, float speedOverride, uint8_t N, Stepper* *steppers){
    MotorControlBase *controller = &_controller->controller;

    StepControl_moveAsync(_controller, speedOverride, N, steppers);

    while(Controller_isRunning(controller)){
        delay(1);
    }
}

void vStepControl_move(StepControl *_controller, float speedOverride, uint8_t N, ...){
    MotorControlBase *controller = &_controller->controller;
    va_list mlist;
    Stepper* stepperArr[MAXMOTORS] = {NULL };

    va_start(mlist, N);
    for(int i = 0; i < N; i++){
        stepperArr[i] = va_arg(mlist, Stepper *);
    }
    va_end(mlist);

    StepControl_moveAsync(_controller, speedOverride, N, stepperArr);

    while(Controller_isRunning(controller)){
        delay(1);
    }
}

// blocking stop command
void StepControl_stop(StepControl *_controller){
    MotorControlBase *controller = &_controller->controller;

    StepControl_stopAsync(_controller);
    while(Controller_isRunning(controller)){
        delay(1);
    }
}


void FUN_IN_RAM StepControl_accTimerISR(StepControl *_controller){

    if(_controller->controller.leadMotor == NULL) return;
    
    MotorControlBase *controller = &_controller->controller;
    LinStepAccelerator *accelerator = &_controller->accelerator;

    int32_t speed = 0, current = 0;
#if defined (HAL_TIMER) && HAL_TIMER
    if(controller->lastPulse){
        controller->timerField.accTimer.setStatus(0);
        return;
    }else{
        noInterrupts();
        current = controller->leadMotor->current;
        interrupts();
        speed = Accelerator_updateSpeed(accelerator, current);
        // printf("%ld\n", speed);
#if(0)
        controller->timerField.stepTimer.setFrequency(speed);
#else
        noInterrupts();
        controller->stepFrequency = speed;
        controller->updateStepTimer = 1;
        interrupts();
#endif

    }
#else
    if(controller->lastPulse){
        TimerField_accTimerStop(&controller->timerField);
        return;
    }else{
        speed = Accelerator_updateSpeed(accelerator, controller->leadMotor->current);
        // printf("%ld\n", speed);
        TimerField_setStepFrequency(&controller->timerField, speed);
    }
#endif
}

