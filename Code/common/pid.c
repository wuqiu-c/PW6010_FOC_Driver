#include "pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义,PID结构体
//-------------------------------------------------------------------------------------------------------------------


//===================================================================================================================
// 内部函数实现
//===================================================================================================================

/**
 * @description: 
 * @param {float} amt：输入数据
 * @param {float} low：下限
 * @param {float} high：上限
 * @return {float*}:限幅后的数据
 * @note: 内部调用
 * @example: 
 */
static float constrain_float(float amt, float low, float high)
{
    if (amt < low) return low;
    else if (amt > high) return high;
    else return amt;
}

//===================================================================================================================
// 外部接口实现
//===================================================================================================================

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
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float int_limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->MaxOut = max_out;
    pid->IntegralLimit = int_limit;
    pid->DeadZone = 0.0f;       // 死区默认为0，如需设置死区可在外部手动修改结构体

    PID_Reset(pid);             // 清空历史状态，防止参数突变导致暴冲
}


/**
 * @description: PID复位
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @return {*}
 * @note: 仅清空 误差(Error)、积分(I_Term)、输出(Output) 等运行时变量
 *        保留 Kp,Ki,Kd 等配置参数
 *        建议在电机启动前、摔倒保护触发后调用
 * @example: 
 */
void PID_Reset(PID_t *pid)
{
    // pid->Target = 0;
    pid->Measure = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->P_Term = 0;
    pid->I_Term = 0;
    pid->D_Term = 0;
    pid->Output = 0;
}

/**
 * @description: pid结构体批量复位
 * @param {PID_t**} Pid_Struct_P_Array：包含pid的所有结构体指针的数组
 * @param {uint8} size：数组大小
 * @return {*}
 * @note: 
 * @example:仅清空 误差(Error)、积分(I_Term)、输出(Output) 等运行时变量
 *          保留 Kp,Ki,Kd 等配置参数
 *          建议在电机启动前、摔倒保护触发后调用 
 */
void PID_All_Reset(PID_t** Pid_Struct_P_Array)
{
    for(uint8_t i = 0;i < FC_PID_ARRAY_SIZE;i++)
    {
        Pid_Struct_P_Array[i]->Target = 0;      //目标值清零
        Pid_Struct_P_Array[i]->Measure = 0;     //当前值清零
        Pid_Struct_P_Array[i]->Error = 0;       //当前误差清零
        Pid_Struct_P_Array[i]->LastError = 0;   //上次误差清零
        Pid_Struct_P_Array[i]->PrevError = 0;   //上上次误差清零
        Pid_Struct_P_Array[i]->P_Term = 0;      //P项清零
        Pid_Struct_P_Array[i]->I_Term = 0;      //I项清零
        Pid_Struct_P_Array[i]->D_Term = 0;      //D项清零
        Pid_Struct_P_Array[i]->Output = 0;      //输出值清零
    }
}

/**
 * @description: 前馈控制器初始化
 * @parameter {PID_FeedForward_t} *ff:前馈控制器句柄指针
 * @parameter {float} kff:前馈系数
 * @parameter {float} max_out:输出限幅，<= 0 表示不限幅
 * @return {*}
 * @notes:
 * @example: PID_FeedForward_Init(&ff_pos_x, 0.2f, 30.0f);
 */
void PID_FeedForward_Init(PID_FeedForward_t *ff, float kff, float max_out)
{
    ff->Kff = kff;
    ff->MaxOut = max_out;

    PID_FeedForward_Reset(ff);
}

/**
 * @description: 前馈控制器复位
 * @parameter {PID_FeedForward_t} *ff:前馈控制器句柄指针
 * @return {*}
 * @notes:
 * @example: PID_FeedForward_Reset(&ff_pos_x);
 */
void PID_FeedForward_Reset(PID_FeedForward_t *ff)
{
    ff->LastInput = 0.0f;
    ff->Output = 0.0f;
    ff->FirstRun = 1;
}

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
float PID_Calc_FeedForward(PID_FeedForward_t *ff, float input, float dt)
{
    if (dt <= 0.0f)
    {
        return ff->Output;
    }

    if (ff->FirstRun)
    {
        ff->LastInput = input;
        ff->Output = 0.0f;
        ff->FirstRun = 0;
        return ff->Output;
    }

    ff->Output = ff->Kff * (input - ff->LastInput) / dt;

    if (ff->MaxOut > 0.0f)
    {
        ff->Output = constrain_float(ff->Output, -ff->MaxOut, ff->MaxOut);
    }

    ff->LastInput = input;

    return ff->Output;
}

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
float PID_Calc_Positional(PID_t *pid, float target, float measure,float dt)
{
    pid->Target = target;
    pid->Measure = measure;
    pid->Error = pid->Target - pid->Measure;

    // 0. 死区处理 (可选)
    if (fabs(pid->Error) < pid->DeadZone)
    {
        pid->Error = 0;
    }

    // 1. 比例项 (P)
    pid->P_Term = pid->Kp * pid->Error;

    // 2. 积分项 (I) - 带抗饱和处理 (Anti-windup)
    pid->I_Term += pid->Ki * dt * pid->Error;
    // 关键：对积分结果进行单独限幅，防止长时间偏差导致积分过大
    pid->I_Term = constrain_float(pid->I_Term, -pid->IntegralLimit, pid->IntegralLimit);

    // 3. 微分项 (D) ———— 采用微分先行
    pid->D_Term = - pid->Kd * (pid->Measure - pid->LastMeasure);

    // 4. 计算总输出
    pid->Output = pid->P_Term + pid->I_Term + pid->D_Term;

    // 5. 总输出限幅
    pid->Output = constrain_float(pid->Output, -pid->MaxOut, pid->MaxOut);

    // 6. 更新历史误差和测量值
    pid->LastError = pid->Error;
    pid->LastMeasure = pid->Measure;

    return pid->Output;
}

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
float PID_Calc_Incremental(PID_t *pid, float target, float measure)
{
    pid->Target = target;
    pid->Measure = measure;
    pid->Error = pid->Target - pid->Measure;
    
    // 0. 死区处理
    if (fabs(pid->Error) < pid->DeadZone)
    {
        pid->Error = 0;
    }

    // 1. 比例项 (P) -> 对应位置式的微分作用
    pid->P_Term = pid->Kp * (pid->Error - pid->LastError);

    // 2. 积分项 (I) -> 对应位置式的比例作用
    // 增量式的 Ki 实际上作用于当前误差
    pid->I_Term = pid->Ki * pid->Error;

    // 3. 微分项 (D)
    pid->D_Term = pid->Kd * (pid->Error - 2 * pid->LastError + pid->PrevError);

    // 4. 计算本次增量 (Delta Output)
    // 注意：增量式本身不累积误差，因此通常不需要积分限幅
    float delta_output = pid->P_Term + pid->I_Term + pid->D_Term;

    // 5. 增量限幅 (可选)
    // 如果需要限制占空比变化率(即限制加速度)，可以在这里对 delta_output 进行 constrain_float
    delta_output = constrain_float(delta_output, -600.0f, 600.0f);        // 每帧最多变化600

    // 6. 更新最终输出值 (累加增量)
    pid->Output += delta_output;
    pid->Output = constrain_float(pid->Output, -pid->MaxOut, pid->MaxOut);

    // 7. 更新历史误差和测量值
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
    pid->LastMeasure = pid->Measure;

    // 返回最终累加后的输出值
    return pid->Output;
}

/**
 * @description: yaw角度环专用 PID 计算 (带最短路径处理)
 * @param {PID_t} *pid:PID 结构体句柄指针
 * @param {float} target:目标角度 (-180 ~ 180)
 * @param {float} measure:当前角度 (-180 ~ 180)
 * @return {float*}:计算出的自转速度 (Omega)
 * @note: 专门解决 ±180 度跳变问题，确保走最近的路线转弯
 * @example: 
 */
float PID_Calc_Heading(PID_t *pid, float target, float measure)
{
    pid->Target = target;
    pid->Measure = measure;

    // 1. 计算原始误差
    pid->Error = pid->Target - pid->Measure;

    // 2. 最短路径处理 (解决 179 到 -179 的问题)
    if (pid->Error > 180.0f)
    {
        pid->Error -= 360.0f;
    }
    else if (pid->Error < -180.0f)
    {
        pid->Error += 360.0f;
    }

    // 3. 死区处理 (可选)
    if (fabs(pid->Error) < pid->DeadZone)
    {
        pid->Error = 0;
    }

    // 4. 比例项 (P)
    pid->P_Term = pid->Kp * pid->Error;

    // 5. 积分项 (I)
    pid->I_Term += pid->Ki * pid->Error;
    pid->I_Term = constrain_float(pid->I_Term, -pid->IntegralLimit, pid->IntegralLimit);

    // 6. 微分项 (D)
    pid->D_Term = pid->Kd * (pid->Error - pid->LastError);

    // 7. 计算总输出
    pid->Output = pid->P_Term + pid->I_Term + pid->D_Term;

    // 8. 总输出限幅
    pid->Output = constrain_float(pid->Output, -pid->MaxOut, pid->MaxOut);

    // 9. 更新历史误差
    pid->LastError = pid->Error;

    return pid->Output;
}
