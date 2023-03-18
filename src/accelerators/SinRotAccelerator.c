
#include "SinRotAccelerator.h"

// typedef struct {
//     int32_t dir;
//     int32_t a;
//     int32_t two_a;
//     int32_t vstp;
//     int32_t v_tgt;
//     int32_t v_tgt_orig;
//     int32_t vstp_tgt;
//     int32_t v_min;
//     int32_t v_min_sqr;
//     int32_t s_0;
// }SinRotAcceleratorParam;

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
    int32_t si = 1;
    int32_t n = 0; // 结果
    int32_t b = 0x8000; // 对应的位
    int32_t temp = 0;
    if((x == 1) || (x == 0)) return x;
    if(x < 0){
        x *= -1;
        si = -1;
    }
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
    return si * n;
}

static int64_t int64_sqrt_bv(int64_t x){
    int v_bit = 31;
    int64_t si = 1;
    int64_t n = 0; // 结果
    int64_t b = 0x80000000; // 对应的位
    int64_t temp = 0;
    if((x == 1) || (x == 0)) return x;
    if(x < 0){
        x *= -1;
        si = -1;
    }
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
    return n * si;
}

static inline float sqrtf_0x5f3759df(float x){
    long i = 0;
    long *l = NULL;
    float *f = NULL;
    float xhalf = 0.5f * x;
    l = (long *)&x;
    i = 0x5f3759df - ((*l) >> 1);
    f = (float *)&i;
    // *f *= 1.5f - xhalf * (*f) * (*f);
    // *f *= 1.5f - xhalf * (*f) * (*f);
    return xhalf * (*f) + 0.5f / (*f);
}


static inline float signed_sqrt(int32_t x) // signed square root
{
    return x > 0 ? sqrtf_0x5f3759df(x) : -sqrtf_0x5f3759df(-x);
}

// Implementation =====================================================================================================

int32_t SinRotAccelerator_prepareRotation(SinRotAccelerator *accelerator, int32_t currentPosition, int32_t targetSpeed, uint32_t acceleration, float speedFactor)
{
    // SinRotAcceleratorParam *accelerator = (SinRotAcceleratorParam *)_accelerator->param;

    accelerator->v_tgt_orig = targetSpeed;
    accelerator->a = acceleration;
    accelerator->two_a = 2 * accelerator->a;
    accelerator->v_min_sqr = accelerator->a;
    accelerator->v_min = sqrtf_0x5f3759df(accelerator->v_min_sqr);
    accelerator->vstp = 0;
    SinRotAccelerator_overrideSpeed(accelerator, speedFactor, currentPosition);

    //Serial.printf("%vtgt:%i vstp_tgt:%i  \n", v_tgt, vstp_tgt);
    return accelerator->v_min;
}

int32_t FUN_IN_RAM SinRotAccelerator_updateSpeed(SinRotAccelerator *accelerator, int32_t curPos)
{
    // SinRotAcceleratorParam *accelerator = (SinRotAcceleratorParam *)_accelerator->param;

    if (accelerator->vstp == accelerator->vstp_tgt) // already at target, keep spinning with target frequency
    {
        return accelerator->v_tgt;
    }

    accelerator->vstp += labs(curPos - accelerator->s_0) * accelerator->dir;
    accelerator->vstp = accelerator->dir == 1 ? min(accelerator->vstp_tgt, accelerator->vstp) : max(accelerator->vstp_tgt, accelerator->vstp); // clamp vstp to target

    //Serial.printf("dir: %i, vstp_tgt:%i, vstp:%i, deltaS:%i\n", dir, vstp_tgt, vstp, deltaS);
    accelerator->s_0 = curPos;
    return signed_sqrt(accelerator->two_a * accelerator->vstp + accelerator->v_min_sqr);
}

int32_t SinRotAccelerator_overrideSpeed(SinRotAccelerator *accelerator, float fac, int32_t curPos)
{
    // SinRotAcceleratorParam *accelerator = (SinRotAcceleratorParam *)_accelerator->param;

    noInterrupts();
    accelerator->s_0 = curPos;
    accelerator->v_tgt = lroundf(accelerator->v_tgt_orig * fac);
    accelerator->vstp_tgt = ((float)accelerator->v_tgt * accelerator->v_tgt) / accelerator->two_a * (accelerator->v_tgt > 0 ? 1.0f : -1.0f);
    accelerator->dir = accelerator->vstp_tgt > accelerator->vstp ? 1 : -1;
    interrupts();
    return accelerator->v_tgt;
}

int32_t SinRotAccelerator_initiateStopping(SinRotAccelerator *_accelerator, int32_t curPos)
{
    SinRotAccelerator_overrideSpeed(_accelerator, 0.0f, curPos);
    return 0;
}