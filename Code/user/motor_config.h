#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/*============================== 数学常数 ===============================*/
#define         _2PI                                    6.283185307197f                                  // 2Π常数
#define         _PI                                     3.141592653589f                                  // Π常数
#define         RTOD                                    57.2957795f                                      // 弧度转角度系数 (180 / PI)
#define         DTOR                                    0.01745329f                                      // 角度转弧度系数 (PI / 180)

/*============================== FOC运行参数 ===============================*/
#define         POSITION_FULL_TURN_DEG                  360.0f                                           // 单圈位置指令的一圈角度，单位度
#define         FOC_HZ                                  20000.0f                                         // FOC执行频率
#define         FOC_DT                                  ((float)(1.0f / FOC_HZ))                         // FOC执行周期
#define         MOTOR_SPEED_MAX_RPM                     1000.0f                                          // 速度指令和位置环输出的转速幅值上限，单位rpm
#define         POS_CLC_DIV                             20U                                              // 每20次速度环更新执行一次位置环
#define         POS_CLC_HZ                              (ENCODER_SPEED_UPDATE_HZ / POS_CLC_DIV)          // 位置环更新频率，单位Hz
#define         POS_CLC_DT                              (1.0f / POS_CLC_HZ)                              // 位置环周期，单位s，当前为5ms

/* ===============================================CAN通信参数======================================*/
#define         MOTOR_CAN_NODE_ID                       1U                                               // 本机节点ID，范围1至255
#define         MOTOR_CAN_TIMEOUT_MS                    500U                                             // 运行请求有效时，CAN控制指令超时停机，单位ms
#define         MOTOR_CAN_FEEDBACK_PERIOD_MS            10U                                              // 自定义状态帧回传周期，单位ms
#define         MOTOR_CAN_RAD_S_TO_RPM                  (60.0f / _2PI)                                   // CAN速度rad/s转控制环rpm
#define         MOTOR_CAN_RPM_TO_RAD_S                  (_2PI / 60.0f)                                   // 控制环rpm转CAN速度rad/s

/*============================== 硬件相关参数 ===============================*/
#define         UDC                                     19.3f                                            // 初始母线电压，运行时使用ADC实测值
#define         MOTOR_PWM_ARR                           4249U                                            // PWM定时器AAR自动重装值
#define         POLE_PAIRS                              14.0f                                            // 14极对

/*============================== 编码器参数 ===============================*/
#define         ENCODER_LINE                            16384                                            // 编码器分辨率，2^14计数/圈
#define         ENCODER_DIR                             ENCODER_DIR_REVERSAL                             // 编码器方向
#define         VD_ALIGN                                0.7f                                             // 编码器校准Vq对齐电压

#define         ENCODER_SPEED_UPDATE_DIV                5.0f                                             // 每5次ADC转换完成回调采集一次速度
#define         ENCODER_SPEED_UPDATE_HZ                 (FOC_HZ / ENCODER_SPEED_UPDATE_DIV)              // 编码器速度更新更新频率
#define         ENCODER_SPEED_UPDATE_DT                 (1.0f / ENCODER_SPEED_UPDATE_HZ)                 // 固定测速周期，单位s

#define         ENCODER_PLL_NATURAL_HZ                  80.0f                                            // PLL自然频率，单位Hz
#define         ENCODER_PLL_DAMPING                     1.0f                                             // PLL阻尼比
#define         ENCODER_ANGLE_PLL_NATURAL_HZ            20.0f                                            // 换相机械角PLL自然频率，单位Hz
#define         ENCODER_ANGLE_PLL_DAMPING               1.0f                                             // 换相机械角PLL阻尼比

/*============================== ADC参数 ===============================*/
#define         I_PHASE_OFFEST_DEFALUT                  2048.0f                                          // 相线电流ADC默认采样偏置
#define         K_ADC_TO_CURRENT                        (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE / CURRENT_GAIN / CURRENT_SHUNT_OHM) // INA240A2、1mΩ采样电阻，单位A/码
#define         ADC_CALIBRATION_COUNT                   1024.0f                                          // ADC零漂校准次数
#define         PHASE_FILTER_WINDOWS                    3                                                // 相电流滑动窗口中值滤波窗口数

/* ===============================================ADC采样与调度参数======================================*/
#define         ADC_REFERENCE_VOLTAGE                   3.3f                                             // ADC参考电压标称值，单位V
#define         ADC_FULL_SCALE                          4096.0f                                          // 12位ADC量化分母
#define         CURRENT_GAIN                            50.0f                                            // 运放固定增益，单位V/V
#define         CURRENT_SHUNT_OHM                       0.001f                                           // 相电流采样电阻，单位Ω
#define         VBUS_UPPER_OHM                          (100000.0f + 5100.0f)                            // R48与R40串联，单位Ω
#define         VBUS_LOWER_OHM                          10000.0f                                         // R41下拉电阻，单位Ω
#define         VBUS_DIVIDER_RATIO                      ((VBUS_UPPER_OHM + VBUS_LOWER_OHM) / VBUS_LOWER_OHM) // 母线与ADC电压比11.51
#define         VBUS_MIN_VALID_V                        1.0f                                             // 防止零母线电压参与SVPWM除法，单位V
#define         ADC_SLOW_AVERAGE_COUNT                  64U                                              // 主循环累计64份新转换结果后发布
#define         ADC_TEMPERATURE_PERIOD_MS               10U                                              // NTC换算周期，单位ms
#define         NTC_PULLUP_OHM                          10000.0f                                         // R51上拉电阻，单位Ω
#define         NTC_R25_OHM                             10000.0f                                         // NCP18XH103F03RB的25℃阻值，单位Ω
#define         NTC_B25_50_K                            3380.0f                                          // B25/50近似值，宽温区需实测校准
#define         NTC_T25_K                               298.15f                                          // 25℃绝对温度，单位K
#define         BUS_SHUNT_OHM                           0.001f                                           // R35母线回流采样电阻，单位Ω
#define         BUS_R38_OHM                             2000.0f                                          // LMV321反相端至PGND，单位Ω
#define         BUS_R42_OHM                             2000.0f                                          // LMV321同相端至采样电阻，单位Ω
#define         BUS_R44_OHM                             10000.0f                                         // 偏置分压上臂，单位Ω
#define         BUS_R45_OHM                             10000.0f                                         // 偏置分压下臂，单位Ω
#define         BUS_R46_OHM                             100000.0f                                        // 同相端至偏置分压节点，单位Ω
#define         BUS_R49_OHM                             100000.0f                                        // 输出至反相端反馈电阻，单位Ω
#define         BUS_BIAS_SOURCE_OHM                     (BUS_R44_OHM * BUS_R45_OHM / (BUS_R44_OHM + BUS_R45_OHM)) // 偏置源等效内阻，单位Ω
#define         BUS_CURRENT_GAIN                        ((1.0f + BUS_R49_OHM / BUS_R38_OHM) * (BUS_R46_OHM + BUS_BIAS_SOURCE_OHM) / (BUS_R42_OHM + BUS_R46_OHM + BUS_BIAS_SOURCE_OHM)) // 计入偏置分压负载的电流增益
#define         K_ADC_TO_BUS_CURRENT                    (ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE / BUS_CURRENT_GAIN / BUS_SHUNT_OHM) // 母线电流系数，单位A/码
#define         PWM_SAMPLE_COMPARE                      (MOTOR_PWM_ARR - 170U)                           // 上数距峰值1μs时触发注入采样
#define         PWM_MAX_COMPARE                         (PWM_SAMPLE_COMPARE - 510U)                      // 最晚开关边沿至采样保留3μs
#define         PWM_MIN_COMPARE                         170U                                             // 最小脉宽约1μs，单位定时器计数
#define         PWM_DEADTIME_TICKS                      68U                                              // TIM1附加死区约400ns，需结合FD6288实测
#define         ENCODER_ALIGN_TIME_MS                   1000U                                            // 沿用源工程1秒对齐时间
#define         VOFA_SEND_PERIOD_MS                     10U                                              // 115200波特率下100Hz发送18通道
#endif
