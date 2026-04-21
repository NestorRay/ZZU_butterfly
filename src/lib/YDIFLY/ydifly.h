/**
 * @file     : ydifly.h
 * @brief    : （一体板开源代码）
 *             YDIFLY蝴蝶扑翼机开源代码，其中功能包括实现蝴蝶的基本遥控飞行，翅膀竖立功能等等，且可通过修改宏定义轻松修改代码参数，无须看懂代码。
 *             一体板具有体积小质量轻，只有一个接收机重量的优势，集所有功能于一身，将主控板重量做到极致。
 * @author   : 一点创绘
 * @date     : 2025-9-14
 * @version  : v1.3
 * 
 * @license  : GPL 3.0 License
 * @changelog:
 * - v1.4 (2025-9-16): 加入上电时电机进入初始位置。
 * - v1.3 (2025-9-14): 初步正式发布开源代码，具有控制蝴蝶飞行，电池电量反馈，连接ELRS遥控等基础功能。
 */
#ifndef __YDIFLY_H

#define __YDIFLY_H

#include <Arduino.h>

/******************** 基本参数 ******************* */
#define YDIFLY_SERVO_LM_L_PIN                  10      // 引脚设置
#define YDIFLY_SERVO_LM_R_PIN                  1       // 引脚设置
#define YDIFLY_SERVO_LJ_L_PIN                  2       // 引脚设置
#define YDIFLY_SERVO_LJ_R_PIN                  3       // 引脚设置

#define YDIFLY_REMOTE_LX                    3       // 左X轴摇杆
#define YDIFLY_REMOTE_LY                    2       // 左Y轴摇杆
#define YDIFLY_REMOTE_RX                    0       // 右X轴摇杆
#define YDIFLY_REMOTE_RY                    1       // 右Y轴摇杆
#define YDIFLY_REMOTE_SWA                   4       // SWA拨杆
#define YDIFLY_REMOTE_SWB                   5       // SWB拨杆
#define YDIFLY_REMOTE_SWC                   6       // SWC拨杆
#define YDIFLY_REMOTE_SWD                   7       // SWD拨杆

#define YDIFLY_REMOTE_JOY_MID               992     // 遥控摇杆的中间值

/******************** 灵眸舵机参数设置 ******************* */
#define YDIFLY_SERVO_LM_ANGLE_L_INIT           90
#define YDIFLY_SERVO_LM_ANGLE_R_INIT           105
#define YDIFLY_SERVO_LM_ANGLE_L_MAX            140
#define YDIFLY_SERVO_LM_ANGLE_L_MIN            35
#define YDIFLY_SERVO_LM_ANGLE_R_MAX            160
#define YDIFLY_SERVO_LM_ANGLE_R_MIN            50

/******************** 蓝箭舵机参数设置 ******************* */
#define YDIFLY_SERVO_LJ_ANGLE_L_INIT           90
#define YDIFLY_SERVO_LJ_ANGLE_R_INIT           105
#define YDIFLY_SERVO_LJ_ANGLE_L_MAX            140
#define YDIFLY_SERVO_LJ_ANGLE_L_MIN            35
#define YDIFLY_SERVO_LJ_ANGLE_R_MAX            160
#define YDIFLY_SERVO_LJ_ANGLE_R_MIN            50

/******************** 舵机方向设置 ******************* */
#define YDIFLY_SERVO_L_DIR                  0       // 左舵机摆动方向，0表示正向，1表示反向
#define YDIFLY_SERVO_R_DIR                  1       // 右舵机摆动方向，0表示正向，1表示反向

/******************** 遥控控制系数设置 ******************* */
#define YDIFLY_FACTOR_FREQ                  0.05f
#define YDIFLY_FACTOR_YAW                   0.02f
#define YDIFLY_FACTOR_PITCH                 0.025f  //俯仰系数
#define YDIFLY_FACTOR_AMP                   0.05f
#define YDIFLY_FACTOR_OFFSET                0.05f
// 如果 YDIFLY_FACTOR_FILTER 很大（接近 1）：算法更倾向于相信新数据，反应非常灵敏，
// 但滤除噪声的能力较弱。
// 如果 $\alpha$ 很小（接近 0）：算法更倾向于相信旧数据，对突发变化的反应迟钝，
// 但能把遥控信号中的“抖动”（高频噪声）滤得非常干净。
#define YDIFLY_FACTOR_FILTER                0.2f

/******************** 翅膀扑翼周期设置 ******************* */
#define YDIFLY_CYCLE_MIN                    200
#define YDIFLY_CYCLE_MAX                    500

/******************** 翅膀扑翼幅度设置 ******************* */
#define YDIFLY_AMP0                         35      // 扑翼幅度为 ±35°
#define YDIFLY_AMP1                         45      // 扑翼幅度为 ±45°
#define YDIFLY_AMP2                         55      // 扑翼幅度为 ±55°

/******************** 翅膀上拍下拍速度差 ******************* */
// 取正值，会减小正弦波在单减区间的时间流速，导致下扑的时候速度变慢
// 同时会增加在单增区间流速，导致上扑的时候速度变快
// 量纲为时间，改变的是相对一个cycle的时间尺度
#define YDIFLY_SPEED_DIFF                   0       // 速度差需要在 -YDIFLY_CONTROL_CYCLE~YDIFLY_CONTROL_CYCLE 之间

/******************** 任务控制周期参数 ******************* */
#define YDIFLY_CONTROL_CYCLE                25      // 舵机的控制周期，ms
#define YDIFLY_DEBUG_CYCLE                  100     // debug的控制周期，ms


typedef enum
{
    SERVO_LM_L,    // 灵眸左翅膀舵机
    SERVO_LM_R,    // 灵眸右翅膀舵机
    SERVO_LJ_L,    // 蓝箭左翅膀舵机
    SERVO_LJ_R,    // 蓝箭右翅膀舵机
}ydifly_servo_name_e;


typedef struct
{
    uint32_t *raw;                  // 遥控原始数据
    float amp;                      // 扑翼幅值
    float freq;                     // 扑翼频率
    float offset;                   // 舵机中间值偏移
    float yaw;                      // 偏航角度控制
    float pitch;                    // 俯仰角度控制
    uint8_t swa;                    // SWA 信号，0和2
    uint8_t swb;                    // SWB 信号，0、1、2
    uint8_t swc;                    // SWC 信号，0、1、2
    uint8_t swd;                    // SWD 信号，0、1、2
}ydifly_remote_cmd_t;


typedef struct
{
    uint8_t init_flag;              // 初始化标志位，0表示还没初始化，1则表示已经完成初始化
    ydifly_remote_cmd_t remote;     // 遥控相关参数
    ydifly_remote_cmd_t remote_last;// 上一次遥控参数
}ydifly_control_t;



void YDIFlyControl( unsigned long now_time_ms );

static void YDIFlyServoSinControl( ydifly_servo_name_e servo_l, ydifly_servo_name_e servo_r, float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff, float *time_now );
static void YDIFlyRemoteDecode( ydifly_remote_cmd_t* remote );
static void YDIFlyInit( void );
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set );


#endif //__YDIFLY_H
