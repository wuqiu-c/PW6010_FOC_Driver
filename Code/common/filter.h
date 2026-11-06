#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>
#include <stdlib.h>

/* 低通滤波结构体*/
typedef struct
{
    float alpha;        // 滤波系数，越大滤波效果越强
    float last_data;    // 上次数据
    float output;       // 滤波输出
}Filter_Lowpass_t;

/* 卡尔曼滤波结构体*/ 
typedef struct
{
    float z;  // 最优估计值
    float p;  // 误差协方差
    float r;  // 测量噪声
    float q;  // 过程噪声
} Filter_Kalman_t;

float Filter_Median_Apply(float* source_buffer,uint8_t* pindex,uint8_t buffer_size,float new_data);
void Filter_Lowpass_Init(Filter_Lowpass_t* temp,float alpha);
float Filter_Lowpass_Updata(Filter_Lowpass_t* temp,float new_data);
void Filter_Kalman_Init(Filter_Kalman_t *kf, float r, float q);
float Filter_Kalman_Update(Filter_Kalman_t *kf, float input);
#endif

