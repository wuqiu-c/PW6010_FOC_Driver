#ifndef __ENCODER_PLL_H__
#define __ENCODER_PLL_H__

#include <stdbool.h>

#define         _2PI                            6.283185307197f             // 2Π常数
#define         _PI                             3.141592653589f             // Π常数
#define         ENCODER_PLL_RAD_S_TO_RPM        (60.0f / _2PI)              // rad/s转rpm系数

typedef struct
{
    float dt;
    float kp;
    float ki;
    float angle;
    float omega;
    float phase_error;
    bool initialized;
}Encoder_Pll_t;

void Encoder_Pll_Init(Encoder_Pll_t* pll,float dt,float natural_hz,float damping);
void Encoder_Pll_Reset(Encoder_Pll_t* pll,float angle,float speed_rpm);
float Encoder_Pll_Update(Encoder_Pll_t* pll,float angle_meas,bool sample_valid);
#endif

