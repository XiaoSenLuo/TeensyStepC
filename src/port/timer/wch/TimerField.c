
#include "TimerField.h"


// static int instances = 0;

// static TIM_Unit timer_mapping[] = TIMER_UNIT();

#define TIM_CLK_PSC      36000000
#define STEP_TIM_CLK     2000000
#define ACC_TIM_CLK      1000000

// TIM_Unit TimerField_getTimer(void){
//     // TODO
//     return (instances >= (sizeof(timer_mapping) / sizeof(TIM_Unit))) ? timer_mapping[0] : timer_mapping[instances++];
// }



#if defined (HAL_TIMER) && HAL_TIMER
#else
void TimerField_init(TimerField *_timerfield, const TimerField_InitTypeDef *config){
    // TODO

}
#endif


#if defined (HAL_TIMER) && HAL_TIMER
#else
bool TimerField_begin(TimerField* timerfield){
    // TODO

}

void FUN_IN_RAM TimerField_end(TimerField* timerfield){
    // TODO

}
#endif

#if defined (HAL_TIMER) && HAL_TIMER

#else


void TimerField_stepTimerStart(TimerField* timerfield){
    // TODO

}

void FUN_IN_RAM TimerField_stepTimerStop(TimerField* timerfield){
    // TODO

}
bool TimerField_stepTimerIsAllocated(const TimerField* timerfield){
    // TODO

}

bool TimerField_stepTimerIsRunning(const TimerField* timerfield){
    return timerfield->stepTimerRunning;
}

int32_t TimerField_getStepFrequency(const TimerField* timerfield){
    // TODO
}

void FUN_IN_RAM TimerField_setStepFrequency(TimerField* timerfield, uint32_t f){   // Hz
    // TODO

}

bool TimerField_accTimerIsRunning(const TimerField* timerfield){
    return timerfield->accTimerRunning;
}

void FUN_IN_RAM TimerField_setPulseWidth(TimerField* timerfield, uint32_t delay){  // us
    // TODO

}

void FUN_IN_RAM TimerField_triggerDelay(TimerField* timerfield){
    // TODO

}

void FUN_IN_RAM TimerField_pulseTimerStop(TimerField* timerfield){

}

bool TimerField_pulseTimerIsRunning(const TimerField* timerfield)
{
#if(ALLOCAT_PULSE_TIMER)

    return timerfield->pulseTimerRunning;
#else

    return timerfield->pulseTimerRunning;
#endif

}

void FUN_IN_RAM TimerField_timerEndAfterPulse(TimerField *_timerfield){
    // TODO
    _timerfield->lastPulse = 1;
}

#endif
