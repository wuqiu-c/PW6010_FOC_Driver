#ifndef CODE_PID_H_
#define CODE_PID_H_

// #include "zf_common_headfile.h"
#include <stdint.h> 
#include <math.h>

#define FC_PID_ARRAY_SIZE   4

#define         _2PI                            6.283185307197f                         // 2Π常数
#define         _PI                             3.141592653589f                         // Π常数


// PID 控制器句柄结构体
typedef struct
{
    //---------------------------------------------------------------------------------------------------------------
    // 参数设置 (Initialization Parameters) - 初始化时设定，运行时通常不变
    //---------------------------------------------------------------------------------------------------------------
    float Kp;           // 比例系数 (Proportional Gain)
                        // 响应当前误差，值越大响应越快，但易震荡

    float Ki;           // 积分系数 (Integral Gain)
                        // 消除稳态误差，值越大消除越快，但易导致超调

    float Kd;           // 微分系数 (Derivative Gain)
                        // 预判误差趋势，产生阻尼作用，抑制震荡

    float MaxOut;       // 输出限幅 (Output Limit) [绝对值]
                        // 限制最终计算结果的最大值 (如 PWM 满占空比 10000)

    float IntegralLimit;// 积分限幅 (Integral Limit) [绝对值]
                        // 仅位置式有效：限制 I_Term 的最大值，防止积分饱和 (Anti-windup)
                        // 建议设为 MaxOut 的 30%~60%

    float DeadZone;     // 死区阈值 (Dead Zone) [绝对值]
                        // 当 |Error| < DeadZone 时，认为误差为0
                        // 用于消除传感器底噪或机械间隙引起的微小抖动

    //---------------------------------------------------------------------------------------------------------------
    // 运行时状态 (Runtime State) - 计算过程中自动更新，用户无需手动修改
    //---------------------------------------------------------------------------------------------------------------
    float Target;       // 目标值 (Set Point)
    float Measure;      // 测量值 (Feedback / Process Value)
    float LastMeasure;  // 上次测量值 - 仅用于微分先行
    float Error;        // 当前误差 (e[k])
    float LastError;    // 上一次误差 (e[k-1])
    float PrevError;    // 上上次误差 (e[k-2]) - 仅增量式PID使用

    //--- 调试用过程量 ---
    float P_Term;       // 比例项计算结果 (P_out)
    float I_Term;       // 积分项累积结果 (I_out)
    float D_Term;       // 微分项计算结果 (D_out)

    float Output;       // 最终计算出的输出值 (Control Output)
} PID_t;

// 前馈控制器句柄结构体
typedef struct
{
    float Kff;          // 前馈系数
    float MaxOut;       // 输出限幅，<= 0 表示不限幅
    float LastInput;    // 上一次输入值
    float Output;       // 前馈输出值
    uint8_t FirstRun;     // 首次计算标志
} PID_FeedForward_t;

//===================================================================================================================
// 全局变量声明 仅举例 用户根据实际需要调整
//===================================================================================================================
// 在 .c 文件中定义，此处 extern 供外部调用


/**
 * @description: 
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @param {float} kp:比例系数
 * @param {float} ki:积分系数
 * @param {float} kd:微分系数
 * @param {float} max_out:最大输出限幅 (绝对值)
 * @param {float} int_limit:积分项限幅 (绝对值，仅位置式有效)
 * @return {*}
 * @note: 
 * @example: 
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float int_limit);

/**
 * @description: PID复位
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @return {*}
 * @note: 仅清空 误差(Error)、积分(I_Term)、输出(Output) 等运行时变量
 *        保留 Kp,Ki,Kd 等配置参数
 *        建议在电机启动前、摔倒保护触发后调用
 * @example: 
 */

void PID_Reset(PID_t *pid);


/**
 * @description: pid结构体批量复位
 * @param {PID_t**} Pid_Struct_P_Array：包含pid的所有结构体指针的数组
 * @return {*}
 * @note: 
 * @example:仅清空 误差(Error)、积分(I_Term)、输出(Output) 等运行时变量
 *          保留 Kp,Ki,Kd 等配置参数
 *          建议在电机启动前、摔倒保护触发后调用 
 */
void PID_All_Reset(PID_t **Pid_Struct_P_Array);

/**
 * @description: 前馈控制器初始化
 * @parameter {PID_FeedForward_t} *ff:前馈控制器句柄指针
 * @parameter {float} kff:前馈系数
 * @parameter {float} max_out:输出限幅，<= 0 表示不限幅
 * @return {*}
 * @notes:
 * @example: PID_FeedForward_Init(&ff_pos_x, 0.2f, 30.0f);
 */
void PID_FeedForward_Init(PID_FeedForward_t *ff, float kff, float max_out);

/**
 * @description: 前馈控制器复位
 * @parameter {PID_FeedForward_t} *ff:前馈控制器句柄指针
 * @return {*}
 * @notes:
 * @example: PID_FeedForward_Reset(&ff_pos_x);
 */
void PID_FeedForward_Reset(PID_FeedForward_t *ff);

/**
 * @description: 前馈控制器计算
 * @parameter {PID_FeedForward_t} *ff:前馈控制器句柄指针
 * @parameter {float} input:输入值
 * @parameter {float} dt:控制周期，单位秒
 * @return {float}:前馈输出值
 * @notes: 1、输出 = Kff * (input - LastInput) / dt。
 *         2、首次计算返回 0，避免上电冲击。
 *         3、如需低通滤波，由调用层使用 Filter 模块处理。
 * @example: ff_out = PID_Calc_FeedForward(&ff_pos_x, pos_error_x, 0.04f);
 */
float PID_Calc_FeedForward(PID_FeedForward_t *ff, float input, float dt);

/**
 * @description: 位置式PID计算
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @param {float} target:目标值
 * @param {float} measure:当前测量值
 * @return {float*}:计算出的控制量 (如 PWM、角度)
 * @note: float pwm = PID_Calc_Positional(&pid_turn, 0, current_yaw);
 * @example: 公式: Out = Kp*e + Ki*Sum(e) + Kd*(e - last_e)
 *           适用于: 舵机控制、直立环平衡控制、角度闭环
 */
float PID_Calc_Positional(PID_t *pid, float target, float measure,float dt);


/**
 * @description: 增量式 PID 计算
 * @param {PID_t} *pid：PID 结构体句柄指针
 * @param {float} target：目标值
 * @param {float} measure：当前测量值
 * @return {float*}:计算后的累积输出值 (Total PWM)
 * @note: float pwm = PID_Calc_Incremental(&pid_speed, 100, encoder_speed);
 * @example: 公式: Delta = Kp*(e - last_e) + Ki*e + Kd*(e - 2*last_e + prev_e)
 *           适用于: 带有积分特性的执行机构，如电机速度闭环
 *           注意: 本函数内部已完成增量累加，返回的是最终的 Output 值
 */
float PID_Calc_Incremental(PID_t *pid, float target, float measure);


/**
 * @description: yaw角度环专用 PID 计算 (带最短路径处理)
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @param {float} target:目标角度 (-180 ~ 180)
 * @param {float} measure:当前角度 (-180 ~ 180)
 * @return {float*}:计算出的自转速度 (Omega)
 * @note: 专门解决 ±180 度跳变问题，确保走最近的路线转弯
 * @example: 
 */
float PID_Calc_Heading(PID_t *pid, float target, float measure);


#endif /* CODE_PID_H_ */
