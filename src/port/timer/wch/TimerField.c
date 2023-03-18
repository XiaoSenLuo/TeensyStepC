
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

#define SETUP_TIM(tim, param)  do{ \
    TIM_DeInit(tim); \
    TIM_TimeBaseInit(tim, param); \
    TIM_InternalClockConfig(tim); \
    TIM_ARRPreloadConfig(tim, ENABLE); \
    TIM_Cmd(tim, DISABLE ); \
}while(0)

#define START_TIM(tim)  do{ \
    TIM_ARRPreloadConfig(tim, ENABLE); \
    TIM_ClearFlag(tim, TIM_FLAG_Update); \
    TIM_ITConfig(tim, TIM_IT_Update, ENABLE); \
    TIM_Cmd(tim, ENABLE); \
}while(0)

#define STOP_TIM(tim)  do{ \
    TIM_Cmd(tim, DISABLE); \
    TIM_ClearFlag(tim, TIM_FLAG_Update); \
    TIM_ITConfig(tim, TIM_IT_Update, DISABLE); \
}while(0)


#if defined (HAL_TIMER) && HAL_TIMER
#else
void TimerField_init(TimerField *_timerfield, const TimerField_InitTypeDef *config){
    // TODO
#if defined (HAL_TIMER) && HAL_TIMER
    _timerfield->stepTimer.Initializes = config->stepTimerInit.initialize;
    _timerfield->stepTimer.start = config->stepTimerInit.start;
    _timerfield->stepTimer.stop = config->stepTimerInit.stop;
    _timerfield->stepTimer.setFrequency = config->stepTimerInit.setFrequency;
    _timerfield->stepTimer.frequency = config->stepTimerInit.frequency;
    _timerfield->stepTimer.setPeriod = config->stepTimerInit.setPeriod;
    _timerfield->stepTimer.period = config->stepTimerInit.period;
    _timerfield->stepTimer.lastPulse = 0;
    _timerfield->stepTimer.running = 0;

    _timerfield->pulseTimer.Initializes = config->pulseTimerInit.initialize;
    _timerfield->pulseTimer.start = config->pulseTimerInit.start;
    _timerfield->pulseTimer.stop = config->pulseTimerInit.stop;
    _timerfield->pulseTimer.setFrequency = config->pulseTimerInit.setFrequency;
    _timerfield->pulseTimer.frequency = config->pulseTimerInit.frequency;
    _timerfield->pulseTimer.setPeriod = config->pulseTimerInit.setPeriod;
    _timerfield->pulseTimer.period = config->pulseTimerInit.period;
    _timerfield->pulseTimer.lastPulse = 0;
    _timerfield->pulseTimer.running = 0;

    _timerfield->accTimer.Initializes = config->accTimerInit.initialize;
    _timerfield->accTimer.start = config->accTimerInit.start;
    _timerfield->accTimer.stop = config->accTimerInit.stop;
    _timerfield->accTimer.setFrequency = config->accTimerInit.setFrequency;
    _timerfield->accTimer.frequency = config->accTimerInit.frequency;
    _timerfield->accTimer.setPeriod = config->accTimerInit.setPeriod;
    _timerfield->accTimer.period = config->accTimerInit.period;
    _timerfield->accTimer.lastPulse = 0;
    _timerfield->accTimer.running = 0;
#else
    if(config){
        _timerfield->accTimer = config->accTimer;
        _timerfield->stepTimer = config->stepTimer;
        _timerfield->pulseTimer = config->pulseTimer;
    }else{
        // _timerfield->stepTimer = TimerField_getTimer();
        // _timerfield->accTimer = TimerField_getTimer();
        // _timerfield->pulseTimer = TimerField_getTimer();
        return;
    }

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };
    TIM_OCInitTypeDef TIM_OCInitStructure={ 0 };
    RCC_ClocksTypeDef RCC_Clocks = { 0 };
    
    uint32_t tim_clk = 144000000, presc = 1;
#if(ALLOCAT_PULSE_TIMER)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
#endif
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    RCC_GetClocksFreq(&RCC_Clocks);
    tim_clk = RCC_Clocks.PCLK1_Frequency;
    if(!(RCC->CFGR0 & RCC_PPRE1_DIV1)){
        tim_clk <<= 1;
    }

    presc = (tim_clk / STEP_TIM_CLK) - 1;

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = (uint16_t)presc;

#if(ALLOCAT_PULSE_TIMER)
    TIM_TimeBaseInitStructure.TIM_Period = STEP_TIM_CLK / 1000000;
    TIM_TimeBaseInitStructure.TIM_Prescaler = presc;
    SETUP_TIM(TIM2, &TIM_TimeBaseInitStructure);
#endif
    /// 用于输出 Step 脉冲
    TIM_TimeBaseInitStructure.TIM_Period = STEP_TIM_CLK / 100000;
    SETUP_TIM(_timerfield->stepTimer, &TIM_TimeBaseInitStructure);

#if(ALLOCAT_PULSE_TIMER == 0)
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = STEP_TIM_CLK / 200000;   /// 默认宽度
    TIM_OC1Init( _timerfield->pulseTimer, &TIM_OCInitStructure );
    TIM_OC1PreloadConfig(_timerfield->pulseTimer, TIM_OCPreload_Enable);
#endif

    /// 用于设置加速间隔
    TIM_TimeBaseInitStructure.TIM_Prescaler = (tim_clk / ACC_TIM_CLK) - 1;
    TIM_TimeBaseInitStructure.TIM_Period = ACC_TIM_CLK / 200;   /// 默认间隔 200Hz
    SETUP_TIM(_timerfield->accTimer, &TIM_TimeBaseInitStructure);

    _timerfield->stepTimerRunning = 0;
    _timerfield->pulseTimerRunning = 0;
    _timerfield->accTimerRunning = 0;
    _timerfield->lastPulse = 0;

#endif
}
#endif


#if defined (HAL_TIMER) && HAL_TIMER
#else
bool TimerField_begin(TimerField* timerfield){
    // TODO
#if defined (HAL_TIMER) && HAL_TIMER

#else
#if(ALLOCAT_PULSE_TIMER == 0)
    TIM_ClearFlag(timerfield->pulseTimer, TIM_FLAG_CC1);
    TIM_ITConfig(timerfield->pulseTimer, TIM_IT_CC1, ENABLE);
    TIM_ARRPreloadConfig(timerfield->stepTimer, DISABLE);
    // TIM_ARRPreloadConfig(TIM3, DISABLE);
#endif
    TIM_SetCounter(timerfield->stepTimer, 0);
    TIM_SetCounter(timerfield->accTimer, 0);
    timerfield->lastPulse = 0;
    return true;
#endif
}

void FUN_IN_RAM TimerField_end(TimerField* timerfield){
    // TODO
#if defined (HAL_TIMER) && HAL_TIMER
    timerfield->stepTimer.stop();
    timerfield->stepTimer.running = 0;
    timerfield->stepTimer.lastPulse = 0;

    timerfield->pulseTimer.stop();
    timerfield->pulseTimer.running = 0;
    timerfield->pulseTimer.lastPulse = 0;

    timerfield->accTimer.stop();
    timerfield->accTimer.running = 0;
    timerfield->accTimer.lastPulse = 0;

#else
    TimerField_stepTimerStop(timerfield);
    TimerField_accTimerStop(timerfield);
    TimerField_pulseTimerStop(timerfield);
    timerfield->lastPulse = 0;
#endif
}
#endif

#if defined (HAL_TIMER) && HAL_TIMER

#else


void TimerField_stepTimerStart(TimerField* timerfield){
    // TODO
    START_TIM(timerfield->stepTimer);
    timerfield->stepTimerRunning = 1;
}

void FUN_IN_RAM TimerField_stepTimerStop(TimerField* timerfield){
    // TODO
    STOP_TIM(timerfield->stepTimer);
    timerfield->stepTimerRunning = 0;
}
bool TimerField_stepTimerIsAllocated(const TimerField* timerfield){
    // TODO
    (void)timerfield;
    return true;
}

bool TimerField_stepTimerIsRunning(const TimerField* timerfield){
    return timerfield->stepTimerRunning;
}

int32_t TimerField_getStepFrequency(const TimerField* timerfield){
    // TODO
    
    return STEP_TIM_CLK / ((timerfield->stepTimer)->ATRLR);
}

void FUN_IN_RAM TimerField_setStepFrequency(TimerField* timerfield, uint32_t f){   // Hz
    // TODO
    uint32_t period = 0;
    static uint16_t ccr = 0;
    static uint32_t minfreq = 0, maxfreq = 0;
#if(1)
    if(ccr == 0){
        ccr = TIM_GetCapture1(timerfield->pulseTimer);
        minfreq = STEP_TIM_CLK / (UINT16_MAX - ccr - 1) + 1;
        maxfreq = STEP_TIM_CLK / (ccr + 1) - 1;

        // printf("%ldHz < f < %ldHz\n", minfreq, maxfreq);
    }
    
    if(f == 0){
        TimerField_stepTimerStop(timerfield);
        return;
    }else if(f < minfreq){
        f = minfreq;
    }else if(f > maxfreq){
        f = maxfreq;
    }
    period = (STEP_TIM_CLK / f);
    // TIM_Cmd(timerfield->stepTimer, DISABLE);
    // TIM_SetCounter(timerfield->stepTimer, 0);
    TIM_SetAutoreload(timerfield->stepTimer, (uint16_t)period);
    // TIM_ARRPreloadConfig(TIM4, ENABLE);
    // TIM_Cmd(timerfield->stepTimer, ENABLE);
#else
    RCC_ClocksTypeDef RCC_Clocks = { 0 };
    uint32_t tim_clk = 0, presc = 0;
    RCC_GetClocksFreq(&RCC_Clocks);
    tim_clk = RCC_Clocks.PCLK1_Frequency;
    if(!(RCC->CFGR0 & RCC_PPRE1_DIV1)){
        tim_clk <<= 1;
    }
    tim_clk /= 4;
    period = tim_clk / f;

    presc = (period >> 16);

    period = period / presc;
    
    TIM_Cmd(TIM4, DISABLE);
    TIM_PrescalerConfig(TIM4, presc - 1, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM4, period);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
#endif
}

void TimerField_accTimerStart(TimerField* timerfield){
    // TODO
    START_TIM(timerfield->accTimer);
    timerfield->accTimerRunning = 1;
}

void FUN_IN_RAM TimerField_accTimerStop(TimerField* timerfield){
    // TODO
    STOP_TIM(timerfield->accTimer);
    timerfield->accTimerRunning = 0;
}

void FUN_IN_RAM TimerField_setAccUpdatePeriod(TimerField* timerfield, uint32_t _period){  // us
    // TODO
    uint16_t period = 0;
    if(_period == 0){
        TimerField_accTimerStop(timerfield);
        return;
    }else if(_period < 1){
        _period = 1;
        period = 1;
    }else if(_period > UINT16_MAX){
        _period = UINT16_MAX;
        period = UINT16_MAX;
    }else{
        period = (uint16_t)_period;
    }
    TIM_SetAutoreload(timerfield->accTimer, period);
}

bool TimerField_accTimerIsRunning(const TimerField* timerfield){
    return timerfield->accTimerRunning;
}

void FUN_IN_RAM TimerField_setPulseWidth(TimerField* timerfield, uint32_t delay){  // us
    // TODO

#if(ALLOCAT_PULSE_TIMER)

    uint32_t timer_clk = 0, period = 0;
    if(delay == 0){
        TimerField_pulseTimerStop(timerfield);
        return;
    }
    timer_clk = STEP_TIM_CLK;
    period = (timer_clk / 1000000) * delay;

    TIM_SetAutoreload(TIM2, period);
#else
    uint16_t ccr = 0;
    if(delay < 1){
        delay = 1;
        ccr = (STEP_TIM_CLK / 1000000);
    }else if(delay > (((UINT16_MAX - 2) / (STEP_TIM_CLK / 1000000)) - 1)){
        delay = (((UINT16_MAX - 2) / (STEP_TIM_CLK / 1000000)) - 1);
    }
    
    ccr = (STEP_TIM_CLK / 1000000) * delay;

    TIM_SetCompare1(timerfield->pulseTimer, ccr);
    TIM_OC1PreloadConfig(timerfield->pulseTimer, TIM_OCPreload_Enable);
#endif

}

void FUN_IN_RAM TimerField_triggerDelay(TimerField* timerfield){
    // TODO
#if(ALLOCAT_PULSE_TIMER)
    
    START_TIM(TIM2);

#else

    TIM_ClearFlag(timerfield->pulseTimer, TIM_FLAG_CC1);
    TIM_ITConfig(timerfield->pulseTimer, TIM_IT_CC1, ENABLE);
#endif
    timerfield->pulseTimerRunning = 1;
}

void FUN_IN_RAM TimerField_pulseTimerStop(TimerField* timerfield){
#if(ALLOCAT_PULSE_TIMER)
    
    STOP_TIM(TIM2);

#else
     (void)timerfield;
     TIM_ITConfig(timerfield->pulseTimer, TIM_IT_CC1, DISABLE);
     TIM_ClearFlag(timerfield->pulseTimer, TIM_FLAG_CC1);

     TimerField_stepTimerStop(timerfield);
#endif
    timerfield->pulseTimerRunning = 0;
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
