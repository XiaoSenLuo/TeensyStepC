

#include "LinStepAccelerator.h"

typedef struct {
    int32_t s_0;
    int32_t ds;
    uint32_t vs;
    uint32_t ve;
    uint32_t vt;
    int64_t vs_sqr;
    int64_t ve_sqr;
    int64_t vt_sqr;
    uint32_t two_a;
    int32_t accEnd;
    int32_t decStart;
}LinStepAcceleratorParam;

static uint32_t uint_sqrt_bv(uint32_t x){
    int v_bit = 15;
    uint32_t n = 0; // 结果
    uint32_t b = 0x8000; // 对应的位
    uint32_t temp = 0;

    if(x <= 1) return x;

    while(b)
    {
        temp = ((n << 1) + b) << (v_bit--);
        if(x >= temp)
        {
            n += b;
            x -= temp;
        }
        b >>= 1;
    }
    return n;
}

static int32_t int_sqrt_bv(int32_t x){
    int v_bit = 15;
    int32_t n = 0; // 结果
    int32_t b = 0x8000; // 对应的位
    int32_t temp = 0;
    if(x <= 1) return x;
    while(b)
    {
        temp = ((n << 1) + b) << (v_bit--);
        if(x >= temp)
        {
            n += b;
            x -= temp;
        }
        b >>= 1;
    }
    return n;
}

static int64_t int64_sqrt_bv(int64_t x){
    int v_bit = 31;
    int64_t n = 0; // 结果
    int64_t b = 0x80000000; // 对应的位
    int64_t temp = 0;
    if(x < 1) return x;
    while(b)
    {
        temp = ((n << 1) + b) << (v_bit--);
        if(x >= temp)
        {
            n += b;
            x -= temp;
        }
        b >>= 1;
    }
    return n;
}

static inline float sqrtf_0x5f3759df(float x){
    long i = 0;
    float f = 0.0f;
    float xhalf = 0.5f * x;
    i = 0x5f3759df - ((*(long *)&x) >> 1);
    f = *(float *)&i;
    // f *= 1.5f - xhalf * f * f;
    // f *= 1.5f - xhalf * f * f;
    return xhalf * f + 0.5f / f;
}


int32_t Accelerator_prepareMovement(LinStepAccelerator *_accelerator, int32_t currentPos, int32_t targetPos, uint32_t targetSpeed, uint32_t pullInSpeed, uint32_t pullOutSpeed, uint32_t a){
    
    LinStepAcceleratorParam *accelerator = (LinStepAcceleratorParam*)_accelerator->param;

    accelerator->vt = targetSpeed;
    accelerator->vs = pullInSpeed;  // v_start
    accelerator->ve = pullOutSpeed; // v_end
    accelerator->two_a = 2 * a;

    accelerator->s_0 = currentPos;
    accelerator->ds = labs(targetPos - currentPos);

    accelerator->vs_sqr = (int64_t)accelerator->vs * accelerator->vs;
    accelerator->ve_sqr = (int64_t)accelerator->ve * accelerator->ve;
    accelerator->vt_sqr = (int64_t)accelerator->vt * accelerator->vt;

    int32_t sm = (int32_t)((accelerator->ve_sqr - accelerator->vs_sqr) / accelerator->two_a + accelerator->ds) / 2; // position where acc and dec curves meet

    // Serial.printf("ve: %d\n", ve);
    // Serial.printf("vs: %d\n", vs);
    // Serial.printf("ds: %d\n", ds);
    // Serial.printf("sm: %i\n", sm);
    // printf("doMove sm=%ld, ds=%ld\n", sm, accelerator->ds);
    if ((sm >= 0) && (sm <= accelerator->ds)) // we can directly reach the MOTOR_TARGET with the given values vor v0, ve and a
    {
        int32_t sa = (accelerator->vt_sqr - accelerator->vs_sqr) / accelerator->two_a; // required distance to reach MOTOR_TARGET speed
        if (sa < sm)                              // MOTOR_TARGET speed can be reached
        {
            accelerator->accEnd = sa;
            accelerator->decStart = sm + (sm - sa);
            //Serial.printf("reachable accEnd: %i decStart:%i\n", accEnd, decStart);
        }
        else
        {
            accelerator->accEnd = accelerator->decStart = sm;
            //Serial.printf("limit accEnd: %i decStart:%i\n", accEnd, decStart);
        }
        // printf("accEnd=%ld, decstart=%ld, sa=%ld\n", accelerator->accEnd, accelerator->decStart, sa);
    }
    else
    {
        // hack, call some error callback instead
        // printf("Accelerator_prepareMovement(): %ld\n", sm);
        // while (1)
        // {
        //     // digitalToggle(LED_BUILTIN);
        //     // delay(25);
        // }
        // printf("currentPos=%ld, targetPos=%ld, targetSpeed=%ld, pullInSpeed=%ld, pullOutSpeed=%ld, a=%ld\n", 
                // currentPos, targetPos, targetSpeed, pullInSpeed, pullOutSpeed, a);
        // return -1; 
    }
    return accelerator->vs;
}

int32_t FUN_IN_RAM Accelerator_updateSpeed(LinStepAccelerator *_accelerator, int32_t currentPosition){

    LinStepAcceleratorParam *accelerator = (LinStepAcceleratorParam*)_accelerator->param;
    int32_t s = labs(accelerator->s_0 - currentPosition);

    // acceleration phase -------------------------------------
    if (s < accelerator->accEnd)
    {
        return (int32_t)sqrtf_0x5f3759df((float)accelerator->two_a * s + accelerator->vs_sqr);
        // return (int32_t)int64_sqrt_bv(accelerator->two_a * s + accelerator->vs_sqr);
    }

    // constant speed phase ------------------------------------
    if (s < accelerator->decStart)
    {
        return accelerator->vt;
    }

    //deceleration phase --------------------------------------
    if (s < accelerator->ds)
    {
        //  return sqrtf(two_a * ((stepsDone < ds - 1) ? ds - stepsDone - 2 : 0) + vs_sqr);
        // return (int32_t)int64_sqrt_bv(accelerator->ve_sqr + (accelerator->ds - s - 1) * accelerator->two_a);
        return sqrtf_0x5f3759df((float)(accelerator->ds - s - 1) * accelerator->two_a + accelerator->ve_sqr);
    }

    //we are done, make sure to return 0 to stop the step timer
    return 0;

}

uint32_t Accelerator_initiateStopping(LinStepAccelerator *_accelerator, int32_t currentPosition){

    LinStepAcceleratorParam *accelerator = (LinStepAcceleratorParam*)_accelerator->param;
   int32_t stepsDone = labs(accelerator->s_0 - currentPosition);

    if (stepsDone < accelerator->accEnd) // still accelerating
    {
        accelerator->accEnd = accelerator->decStart = 0; // start deceleration
        accelerator->ds = 2 * stepsDone;    // we need the same way to decelerate as we traveled so far
        return stepsDone;      // return steps to go
    }
    else if (stepsDone < accelerator->decStart) // constant speed phase
    {
        accelerator->decStart = 0;            // start deceleration
        accelerator->ds = stepsDone + accelerator->accEnd; // normal deceleration distance
        return accelerator->accEnd;           // return steps to go
    }
    else // already decelerating
    {
        return accelerator->ds - stepsDone; // return steps to go
    }
}

void Accelerator_overrideSpeed(LinStepAccelerator *_accelerator, float fac, int32_t currentPosition){
    (void)_accelerator;
    (void)fac;
    (void)currentPosition;
}

