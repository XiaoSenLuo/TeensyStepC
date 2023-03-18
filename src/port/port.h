
#ifndef _TEENSYSTEP_PORT_H__
#define _TEENSYSTEP_PORT_H__

#include <stdint.h>
#include <stdbool.h>

#ifndef min
#define min(a, b)       ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b)       ((a) > (b) ? (a) : (b))
#endif

#ifndef HIGH
#define HIGH             1
#endif
#ifndef LOW
#define LOW              0
#endif

#define FUN_IN_RAM
#define ALLOCAT_PULSE_TIMER       0

#ifdef MATH_TYPE
#include "IQmath_RV32.h"
#endif


#include "ch32v20x_gpio.h"
#include "ch32v20x_tim.h"
#include "core_riscv.h"


#ifndef delay
#define delay(ms)       Delay_Ms(ms)
#endif

#ifndef noInterrupts
#define noInterrupts()             __disable_irq()
#endif

#ifndef interrupts
#define interrupts()               __enable_irq()
#endif

/// TODO
#define HAL_TIMER                 1


typedef struct {
    uint16_t pin;
    uint32_t port;
}gpio_pin_t;


static inline void FUN_IN_RAM digitalPinOutputMode(gpio_pin_t pin){
    // TODO
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = pin.pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init((GPIO_TypeDef *)(pin.port), &GPIO_InitStructure);
}

static inline void FUN_IN_RAM digitalWritePin(gpio_pin_t pin, uint8_t level){
    // TODO
    if(level){
        ((GPIO_TypeDef *)(pin.port))->BSHR = pin.pin;
    }else{
        ((GPIO_TypeDef *)(pin.port))->BCR = pin.pin;
    }
}


// #ifndef TIMER_UNIT
// #define TIMER_UNIT()          {TIM2, TIM3, TIM4,TIM5}
// #endif


typedef TIM_TypeDef* TIM_Unit; /// 定义定时器单元

#if defined (HAL_TIMER) && HAL_TIMER
struct HardTimer_Type {
    // volatile struct {
    //     // uint32_t running : 1;
    //     // uint32_t lastPulse : 1;
    //     // uint32_t period : 16;
    // };
    // int32_t next_fequency;
    // void *timer;
    void (*const initialize)(void);
    // void (*prepare)(void);
    void (*const setStatus)(uint32_t);
    // void (*start)(void);
    // void (*stop)(void);
    bool (*const isRunning)(void);
    void (*const setFrequency)(uint32_t);
    uint32_t (*const frequency)(void);
    void (*const setPeriod)(uint32_t);
    uint32_t (*const period)(void);
};
#endif

typedef struct TimerField_Def{
#if defined (HAL_TIMER) && HAL_TIMER
    struct HardTimer_Type stepTimer;
    struct HardTimer_Type pulseTimer;
    struct HardTimer_Type accTimer;
#else
    struct {
        uint32_t stepTimerRunning : 1;
        uint32_t accTimerRunning : 1;
        uint32_t pulseTimerRunning : 1;
        uint32_t lastPulse : 1;
    };

    TIM_Unit stepTimer;
    TIM_Unit accTimer;
    TIM_Unit pulseTimer;
#endif
}TimerField;

typedef struct {
#if defined (HAL_TIMER) && HAL_TIMER
    struct HardTimer_Type *stepTimer;
    struct HardTimer_Type *pulseTimer;
    struct HardTimer_Type *accTimer;
#else
    TIM_Unit stepTimer;
    TIM_Unit accTimer;
    TIM_Unit pulseTimer;
#endif
}TimerField_InitTypeDef;

#if defined (HAL_TIMER) && HAL_TIMER

#else

void TimerField_init(TimerField *_timerfield, const TimerField_InitTypeDef *config);

bool TimerField_begin(TimerField* timerfield);
void TimerField_end(TimerField* timerfield);



void TimerField_stepTimerStart(TimerField* timerfield);   /// 需要强制更新
void TimerField_stepTimerStop(TimerField* timerfield);
bool TimerField_stepTimerIsRunning(const TimerField* timerfield);
bool TimerField_stepTimerIsAllocated(const TimerField* timerfield);
int32_t TimerField_getStepFrequency(const TimerField* timerfield);
void FUN_IN_RAM TimerField_setStepFrequency(TimerField* timerfield, uint32_t f);

void TimerField_accTimerStart(TimerField* timerfield);
void TimerField_accTimerStop(TimerField* timerfield);
void FUN_IN_RAM TimerField_setAccUpdatePeriod(TimerField* timerfield, uint32_t period);
bool TimerField_accTimerIsRunning(const TimerField* timerfield);

void FUN_IN_RAM TimerField_setPulseWidth(TimerField* timerfield, uint32_t delay);
void FUN_IN_RAM TimerField_triggerDelay(TimerField* timerfield);
void TimerField_pulseTimerStop(TimerField* timerfield);
bool TimerField_pulseTimerIsRunning(const TimerField* timerfield);

void FUN_IN_RAM TimerField_timerEndAfterPulse(TimerField *_timerfield);

#endif

#endif
