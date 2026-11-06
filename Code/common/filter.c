#include "filter.h"
#include <string.h>

/**
 * @brief: 滑动窗口中值滤波
 * @param {float*} source_buffer 原数据缓冲区
 * @param {uint8_t*} pindex 原数据缓冲区索引指针
 * @param {uint8_t} buffer_size 数据缓冲区大小(个数)
 * @param {float} new_data 新数据
 * @return {*} 排序后的中值
 * @notes: 
 * @example: 
 */
float Filter_Median_Apply(float* source_buffer,uint8_t* pindex,uint8_t buffer_size,float new_data)
{
    /* 1.更新数据到环形队列*/
    source_buffer[*pindex] = new_data;
    (*pindex) ++;
    if((*pindex) >= buffer_size)
    {
        (*pindex) = 0;
    }

    /* 2.复制一份用来排序，不打乱原数据队列*/
    float sorted_buffer[buffer_size];
    memcpy(sorted_buffer, source_buffer, buffer_size*sizeof(sorted_buffer[0]));

    /* 3.进行冒泡排序*/
    for(uint8_t i = 0; i < buffer_size - 1; i ++)
    {
        for(uint8_t j = 0; j < buffer_size - 1; j++)
        {
            if(sorted_buffer[j + 1] < sorted_buffer[j])
            {
                float temp = sorted_buffer[j];
                sorted_buffer[j] = sorted_buffer[j + 1];
                sorted_buffer[j + 1] = temp;
            }
        }
    }

    /* 4.返回中间值*/
    return sorted_buffer[buffer_size / 2];
}

/**
 * @brief: 一阶低通滤波初始化
 * @param {Filter_Lowpass_t*} temp 一阶低通滤波结构体指针
 * @param {float} alpha 滤波系数,越大滤波效果越强
 * @return {*}
 * @notes: 
 * @example: 
 */
void Filter_Lowpass_Init(Filter_Lowpass_t* temp,float alpha)
{
    temp->alpha = alpha;
    temp->last_data = 0;
    temp->output = 0;
}

/**
 * @brief: 一阶低通滤波更新
 * @param {Filter_Lowpass_t*} temp 一阶低通滤波结构体指针
 * @param {float} new_data 传感器新数据
 * @return {*} 滤波输出数据
 * @notes: 
 * @example: 
 */
float Filter_Lowpass_Updata(Filter_Lowpass_t* temp,float new_data)
{
    temp->output = (1 - temp->alpha)*new_data + temp->alpha*temp->last_data;
    temp->last_data = temp->output;
    return temp->output;
}

/**
 * @brief: 卡尔曼滤波初始化
 * @param {kalman_filter_t} *kf 卡尔曼滤波结构体指针
 * @param {float} r 
 * @param {float} q 
 * @return {*} 无
 * @notes: 
 * @example: 
 */
void Filter_Kalman_Init(Filter_Kalman_t *kf, float r, float q)
{
    kf->z = 0;
    kf->p = 1;
    kf->r = r;
    kf->q = q;
}


float Filter_Kalman_Update(Filter_Kalman_t *kf, float input)
{
    kf->p = kf->p + kf->q;
    float g = kf->p / (kf->p + kf->r);
    kf->z = kf->z + g * (input - kf->z);
    kf->p = (1 - g) * kf->p;
    return kf->z;
}

/*
先固定 q，调 r：

    r越大 → 越不信任采样值 → 滤波越平滑，响应越慢；
    r越小 → 越信任采样值 → 滤波越贴近原始值，降噪效果差；
    电流采样推荐r=0.01 0.5，转速采样推荐r=0.1 1.0；

再微调 q：

    q越大 → 越不信任系统预测 → 滤波响应越快，平滑性下降；
    q越小 → 越信任系统预测 → 滤波越平滑，响应越慢；
    电机控制推荐q=0.001 0.1（q通常远小于r）；

要平滑 → 增大r、减小q；
要响应 → 减小r、增大q；

*/