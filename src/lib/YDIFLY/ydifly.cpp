/**
 * @file     : ydifly.cpp
 * @brief    : （一体板开源代码）
 *             YDIFLY蝴蝶扑翼机开源代码，其中功能包括实现蝴蝶的基本遥控飞行，翅膀竖立功能等等，且可通过修改宏定义轻松修改代码参数，无须看懂代码。
 *             一体板具有体积小质量轻，只有一个接收机重量的优势，集所有功能于一身，将主控板重量做到极致。
 * @author   : 一点创绘
 * @date     : 2026-1-26
 * @version  : v2.1
 * 
 * @license  : GPL 3.0 License
 * @changelog:
 * - v2.1 (2026-1-26): 立翅加入可调速功能。
 * - v2.0 (2026-1-24): 优化遥控的控制逻辑，控制更加灵活。
 * - v1.4 (2025-9-16): 加入上电时电机进入初始位置。
 * - v1.3 (2025-9-14): 初步正式发布开源代码，具有控制蝴蝶飞行，电池电量反馈，连接ELRS遥控等基础功能。
 */
#include "CRSF.h"
#include "common.h"
#include "ydifly.h"

/******************** 基本参数 ******************* */
#define YDIFLY_SERVO_L_PIN                  10      // 引脚设置
#define YDIFLY_SERVO_R_PIN                  1       // 引脚设置

#define YDIFLY_REMOTE_LX                    3       // 左X轴摇杆
#define YDIFLY_REMOTE_LY                    2       // 左Y轴摇杆
#define YDIFLY_REMOTE_RX                    0       // 右X轴摇杆
#define YDIFLY_REMOTE_RY                    1       // 右Y轴摇杆
#define YDIFLY_REMOTE_SWA                   4       // SWA拨杆
#define YDIFLY_REMOTE_SWB                   5       // SWB拨杆
#define YDIFLY_REMOTE_SWC                   6       // SWC拨杆
#define YDIFLY_REMOTE_SWD                   7       // SWD拨杆

#define YDIFLY_REMOTE_JOY_MID               992     // 遥控摇杆的中间值

/******************** 舵机参数设置 ******************* */
#define YDIFLY_SERVO_ANGLE_L_INIT           90
#define YDIFLY_SERVO_ANGLE_R_INIT           105
#define YDIFLY_SERVO_ANGLE_L_MAX            140
#define YDIFLY_SERVO_ANGLE_L_MIN            35
#define YDIFLY_SERVO_ANGLE_R_MAX            160
#define YDIFLY_SERVO_ANGLE_R_MIN            50

/******************** 舵机方向设置 ******************* */
#define YDIFLY_SERVO_L_DIR                  0       // 左舵机摆动方向，0表示正向，1表示反向
#define YDIFLY_SERVO_R_DIR                  1       // 右舵机摆动方向，0表示正向，1表示反向

/******************** 遥控控制系数设置 ******************* */
#define YDIFLY_YAW_REMOTE_ANGLE_MAX         20.0f   // 正负20°
#define YDIFLY_FACTOR_FREQ                  0.05f   // 频率系数
#define YDIFLY_FACTOR_AMP                   0.05f
#define YDIFLY_FACTOR_OFFSET                0.02f

#define YDIFLY_FACTOR_FILTER                0.2f

/******************** 翅膀扑翼周期设置 ******************* */
#define YDIFLY_CYCLE_MIN                    235
#define YDIFLY_CYCLE_MAX                    500

/******************** 翅膀扑翼幅度设置 ******************* */
#define YDIFLY_AMP0                         35      // 扑翼幅度为 ±30°
#define YDIFLY_AMP1                         45      // 扑翼幅度为 ±40°
#define YDIFLY_AMP2                         55      // 扑翼幅度为 ±50°

/******************** 立翅速度设置 ******************* */
#define YDIFLY_WING_STAND_SPEED             0.5f    // 数字越小速度越慢

/******************** 翅膀上拍下拍速度差 ******************* */
#define YDIFLY_SPEED_DIFF                   8       // 速度差需要在 -YDIFLY_CONTROL_CYCLE~YDIFLY_CONTROL_CYCLE 之间

/******************** 任务控制周期参数 ******************* */
#define YDIFLY_CONTROL_CYCLE                25      // 舵机的控制周期，ms
#define YDIFLY_DEBUG_CYCLE                  100     // debug的控制周期，ms


typedef enum
{
    SERVO_L,    // 左翅膀舵机
    SERVO_R,    // 右翅膀舵机
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

/* 全局变量 */
ydifly_control_t ydifly;
extern connectionState_e connectionState;
float time_now = 0;
float time_init = 0;

/* 函数声明 */
static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff );
static void YDIFlyRemoteDecode( ydifly_remote_cmd_t* remote );
static void YDIFlyInit( void );
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set );
static void YDIFlyServoAngleSpeedControl( ydifly_servo_name_e servo_name, float angle_set, float speed );

static void YDIFlyInit( void )
{
    ydifly.remote.raw = CRSF::ChannelData;

    YDIFlyRemoteDecode( &ydifly.remote );
    YDIFlyRemoteDecode( &ydifly.remote_last );

    YDIFlyServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_INIT );  // 上电时，电机运行在初始位置
    YDIFlyServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_INIT );  // 上电时，电机运行在初始位置
}

/******************* 主控制函数 *******************/
void YDIFlyControl( unsigned long now_time_ms )
{
    static unsigned long last_time_ms_control = now_time_ms;    // 上一次时间
    static unsigned long last_time_ms_debug = now_time_ms;      // 上一次时间

    static unsigned long last_time_ms = now_time_ms;            // 上一次时间
    float control_T;

    /*输入保护*/
    if( last_time_ms > now_time_ms )    // 判断是否时间溢出，即超出49.7天
    {
        last_time_ms = now_time_ms;             // 时间溢出重新计数
        last_time_ms_control = now_time_ms;     // 时间溢出重新计数
        last_time_ms_debug = now_time_ms;       // 时间溢出重新计数
    }

    if( ydifly.init_flag == 0 )     // 如果还没有初始化
    {
        ydifly.init_flag = 1;       // 设置flag，表示完成初始化
        YDIFlyInit();
    }
    else if( ydifly.init_flag == 1 )// 已完成初始化，判断遥控是否连接
    {
        if( connectionState == connected )      ydifly.init_flag = 2;   // 遥控已连接
    }
    else    // 已经完成初始化，启动任务
    {
        /* 舵机周期控制 */
        if( now_time_ms - last_time_ms_control >= YDIFLY_CONTROL_CYCLE )
        {
            float angle_l_add, angle_l_mid, angle_r_add, angle_r_mid;
            float temp = 0;

            last_time_ms_control += YDIFLY_CONTROL_CYCLE;     // 时间补全

            /* 获取遥控解算数据 */
            YDIFlyRemoteDecode( &ydifly.remote );

            /* 翅膀扑翼幅度解算 */
            if     ( ydifly.remote.swb == 0 )        ydifly.remote.amp = YDIFLY_AMP0;   // 不同档位幅值不同
            else if( ydifly.remote.swb == 1 )        ydifly.remote.amp = YDIFLY_AMP1;   // 不同档位幅值不同
            else                                     ydifly.remote.amp = YDIFLY_AMP2;   // 不同档位幅值不同
            
            /* 通过SWD拨杆，设置翅膀第一次是向上扑还是向下扑 */
            if     ( ydifly.remote.swd == 2 )        time_init = 3.141592653f;  // SWD上拨，扑翼初始方向反向
            else                                     time_init = 0;             // SWD下拨和中档，扑翼初始方向正向

            /* 一阶滤波 */
            ydifly.remote.yaw   = YDIFLY_FACTOR_FILTER*ydifly.remote.yaw    + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.yaw;
            ydifly.remote.pitch = YDIFLY_FACTOR_FILTER*ydifly.remote.pitch  + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.pitch;
            ydifly.remote.freq  = YDIFLY_FACTOR_FILTER*ydifly.remote.freq   + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.freq;
            ydifly.remote.amp   = YDIFLY_FACTOR_FILTER*ydifly.remote.amp    + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.amp;
            ydifly.remote.offset= YDIFLY_FACTOR_FILTER*ydifly.remote.offset + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.offset;

            ydifly.remote_last.yaw   = ydifly.remote.yaw;
            ydifly.remote_last.pitch = ydifly.remote.pitch;
            ydifly.remote_last.freq  = ydifly.remote.freq;
            ydifly.remote_last.amp   = ydifly.remote.amp;
            ydifly.remote_last.offset= ydifly.remote.offset;

            /* 舵机角度控制 */
            if( ydifly.remote.pitch > 0 )
            {
                angle_l_mid = (YDIFLY_SERVO_ANGLE_L_INIT-YDIFLY_SERVO_ANGLE_L_MIN-ydifly.remote.amp)*ydifly.remote.pitch/1500;
                angle_r_mid = (YDIFLY_SERVO_ANGLE_R_INIT-YDIFLY_SERVO_ANGLE_R_MIN-ydifly.remote.amp)*ydifly.remote.pitch/1500;
                temp = (angle_l_mid>angle_r_mid) ? angle_r_mid : angle_l_mid;
            }
            else
            {
                angle_l_mid = (YDIFLY_SERVO_ANGLE_L_MAX-YDIFLY_SERVO_ANGLE_L_INIT-ydifly.remote.amp)*ydifly.remote.pitch/1500;
                angle_r_mid = (YDIFLY_SERVO_ANGLE_R_MAX-YDIFLY_SERVO_ANGLE_R_INIT-ydifly.remote.amp)*ydifly.remote.pitch/1500;
                temp = (angle_l_mid>angle_r_mid) ? angle_l_mid : angle_r_mid;
            }

            /* 舵机扑翼中位值计算 */
            angle_l_mid = YDIFLY_SERVO_ANGLE_L_INIT - temp + ydifly.remote.offset*YDIFLY_FACTOR_OFFSET;
            angle_r_mid = YDIFLY_SERVO_ANGLE_R_INIT - temp - ydifly.remote.offset*YDIFLY_FACTOR_OFFSET;

            if( ydifly.remote.freq > 10 )    // 如果有给油门，则进入起飞程序
            {
                float angle_rate = 1, angle_rate_min = 1;


                temp = (ydifly.remote.yaw>0) ? (ydifly.remote.yaw*YDIFLY_YAW_REMOTE_ANGLE_MAX/1500) : (ydifly.remote.yaw*YDIFLY_YAW_REMOTE_ANGLE_MAX/1500);
                angle_l_add = - temp + ydifly.remote.amp;
                angle_r_add = + temp + ydifly.remote.amp;


                /* 限幅 */
                angle_l_add = (angle_l_add>ydifly.remote.amp) ? ydifly.remote.amp : angle_l_add;
                angle_r_add = (angle_r_add>ydifly.remote.amp) ? ydifly.remote.amp : angle_r_add;

                /* 限幅 */
                if( (angle_l_mid+angle_l_add) > YDIFLY_SERVO_ANGLE_L_MAX )                
                {
                    angle_rate = (YDIFLY_SERVO_ANGLE_L_MAX-angle_l_mid)/angle_l_add;
                    angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
                }
                if( (angle_l_mid-angle_l_add) < YDIFLY_SERVO_ANGLE_L_MIN )
                {
                    angle_rate = (angle_l_mid-YDIFLY_SERVO_ANGLE_L_MIN)/angle_l_add;
                    angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
                }
                if( (angle_r_mid+angle_r_add) > YDIFLY_SERVO_ANGLE_R_MAX )
                {
                    angle_rate = (YDIFLY_SERVO_ANGLE_R_MAX-angle_r_mid)/angle_r_add;
                    angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
                }
                if( (angle_r_mid-angle_r_add) < YDIFLY_SERVO_ANGLE_R_MIN )
                {
                    angle_rate = (angle_r_mid-YDIFLY_SERVO_ANGLE_R_MIN)/angle_r_add;
                    angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
                }
                angle_l_add *= angle_rate_min;
                angle_r_add *= angle_rate_min;

                /* 舵机控制正弦周期 */
                control_T = YDIFLY_CYCLE_MAX + ydifly.remote.freq*(YDIFLY_CYCLE_MIN - YDIFLY_CYCLE_MAX)/1500;

                /* 舵机角度控制 */

                YDIFlyServoSinControl( (angle_l_mid+angle_l_add), (angle_l_mid-angle_l_add), (angle_r_mid+angle_r_add), (angle_r_mid-angle_r_add), control_T, YDIFLY_SPEED_DIFF );
                // YDIFlyServoSinControl( 120, 60, 120, 60, 1000, YDIFLY_SPEED_DIFF );

            }
            else if( ydifly.remote.swc == 2 )     // 如果拨动开关，翅膀摆动。拨杆到上档位
            {
                YDIFlyServoAngleSpeedControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MIN, YDIFLY_WING_STAND_SPEED );
                YDIFlyServoAngleSpeedControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MIN, YDIFLY_WING_STAND_SPEED );
                time_now = 0;
            }
            else if( ydifly.remote.swc == 1 )     // 舵机处于初始位置。拨杆到中档位
            {
                YDIFlyServoAngleSpeedControl( SERVO_L, angle_l_mid, YDIFLY_WING_STAND_SPEED );
                YDIFlyServoAngleSpeedControl( SERVO_R, angle_r_mid, YDIFLY_WING_STAND_SPEED );
                time_now = 0;
            }
            else    // 如果拨动开关，翅膀摆动。拨杆到下档位
            {
                YDIFlyServoAngleSpeedControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MAX, YDIFLY_WING_STAND_SPEED );
                YDIFlyServoAngleSpeedControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MAX, YDIFLY_WING_STAND_SPEED );
                time_now = 0;
            }
        }

        /*调试周期控制*/
        if( now_time_ms - last_time_ms_debug >= YDIFLY_DEBUG_CYCLE )
        {
            last_time_ms_debug += YDIFLY_DEBUG_CYCLE;       // 时间补全
            
            // Serial.printf("\r\n%f,%f,%f,%f,%f\r\n", angle_l_max, angle_l_min, angle_r_max, angle_r_min, control_T);
            // Serial.printf("\r\n%d,%d,%d,%d\r\n", ydifly.remote.raw[YDIFLY_REMOTE_SWA], ydifly.remote.raw[YDIFLY_REMOTE_SWB], ydifly.remote.raw[YDIFLY_REMOTE_SWC], ydifly.remote.raw[YDIFLY_REMOTE_SWD]);
            // Serial.printf("\r\n%d,%d,%d,%d\r\n", ydifly.remote.swa, ydifly.remote.swb, ydifly.remote.swc, ydifly.remote.swd);
        }
    }

    last_time_ms = now_time_ms;
}

static void YDIFlyRemoteDecode( ydifly_remote_cmd_t* remote )
{
    /* 将遥控的数据映射在 0~1500 之间 */
    remote->freq = constrain( ydifly.remote.raw[YDIFLY_REMOTE_LY], 300, 1800 ) - 300;

    /* 将遥控的数据映射在 -700~800 之间 */
    remote->yaw   = (float)ydifly.remote.raw[YDIFLY_REMOTE_LX] - YDIFLY_REMOTE_JOY_MID;
    remote->pitch = (float)ydifly.remote.raw[YDIFLY_REMOTE_RY] - YDIFLY_REMOTE_JOY_MID;
    remote->offset= (float)ydifly.remote.raw[YDIFLY_REMOTE_RX] - YDIFLY_REMOTE_JOY_MID;

    /* 解算 SWA\SWB\SWC\SWD 信号 */
    if( ydifly.remote.raw[YDIFLY_REMOTE_SWA] < 300 )            remote->swa = 0;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWA] > 1500 )      remote->swa = 2;
    if( ydifly.remote.raw[YDIFLY_REMOTE_SWB] < 300 )            remote->swb = 0;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWB] > 1500 )      remote->swb = 2;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWB] > 800 )       remote->swb = 1;
    if( ydifly.remote.raw[YDIFLY_REMOTE_SWC] < 300 )            remote->swc = 0;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWC] > 1500 )      remote->swc = 2;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWC] > 800 )       remote->swc = 1;
    if( ydifly.remote.raw[YDIFLY_REMOTE_SWD] < 300 )            remote->swd = 0;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWD] > 1500 )      remote->swd = 2;
    else if( ydifly.remote.raw[YDIFLY_REMOTE_SWD] > 800 )       remote->swd = 1;
}

static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff )
{
    float angle_set = 0;

    /* 输入参数保护 */
    if( speed_diff > YDIFLY_CONTROL_CYCLE )         speed_diff = YDIFLY_CONTROL_CYCLE-1;
    else if( speed_diff < -YDIFLY_CONTROL_CYCLE )   speed_diff =-YDIFLY_CONTROL_CYCLE+1;


    angle_set = ((l_angle_max-l_angle_min)/2)*sin( time_now*6.283185307179586/T + time_init ) + (l_angle_max+l_angle_min)/2;

    YDIFlyServoAngleControl( SERVO_L, angle_set );

    angle_set = ((r_angle_max-r_angle_min)/2)*sin( time_now*6.283185307179586/T + time_init ) + (r_angle_max+r_angle_min)/2;
    YDIFlyServoAngleControl( SERVO_R, angle_set );

    time_now += YDIFLY_CONTROL_CYCLE;
    if( (time_now > T*0.25f) && (time_now < T*0.75f) )  time_now -= speed_diff;
    else                                                time_now += speed_diff;

    if( time_now > T )
    {
        time_now -= T;
    }    
}

extern void startWaveform8266(uint8_t pin, uint32_t timeHighUS, uint32_t timeLowUS);

//传入角度参数，转换为PWM信号输出给舵机
static void YDIFlyServoAngleControl( ydifly_servo_name_e servo_name, float angle_set )
{
    /*舵机控制角度和PWM占空比换算*/
    float time_hight_us, time_hight_us_r;
    time_hight_us   = 500.0f + 2000.0f*angle_set/180.0f;    // 角度和PWM高电平时间换算
    time_hight_us_r =2500.0f - 2000.0f*angle_set/180.0f;    // 角度和PWM高电平时间换算，反向

    /*对应角度的舵机控制*/
    switch (servo_name)
    {
    case SERVO_L:
        #if YDIFLY_SERVO_L_DIR
        startWaveform8266(YDIFLY_SERVO_L_PIN, time_hight_us_r, 20000-time_hight_us_r);
        #else
        startWaveform8266(YDIFLY_SERVO_L_PIN, time_hight_us, 20000-time_hight_us);
        #endif
        break;
    case SERVO_R:
        #if YDIFLY_SERVO_R_DIR
        startWaveform8266(YDIFLY_SERVO_R_PIN, time_hight_us_r, 20000-time_hight_us_r);
        #else
        startWaveform8266(YDIFLY_SERVO_R_PIN, time_hight_us, 20000-time_hight_us);
        #endif
        break;
    }
}

static void YDIFlyServoAngleSpeedControl( ydifly_servo_name_e servo_name, float angle_set, float speed )
{
  static float angleL_now = YDIFLY_SERVO_ANGLE_L_INIT, angleR_now = YDIFLY_SERVO_ANGLE_R_INIT;

  if( angle_set < 0 )       angle_set = 0;
  else if( angle_set > 180) angle_set = 180;
  if( speed < 0 )           speed = 0;
  else if( speed > 100)     speed = 100;

  if( servo_name == SERVO_L )
  {
    if( fabs(angleL_now-angle_set) <= speed )
    {
      angleL_now = angle_set;
    }
    else if( (angleL_now-angle_set) > speed )
    {
      angleL_now -= speed;
    }
    else if( (angleL_now-angle_set) <-speed )
    {
      angleL_now += speed;
    }
    YDIFlyServoAngleControl(SERVO_L, angleL_now);
  }
  else if( servo_name == SERVO_R )
  {
    if( fabs(angleR_now-angle_set) <= speed )
    {
      angleR_now = angle_set;
    }
    else if( (angleR_now-angle_set) > speed )
    {
      angleR_now -= speed;
    }
    else if( (angleR_now-angle_set) <-speed )
    {
      angleR_now += speed;
    }
    YDIFlyServoAngleControl(SERVO_R, angleR_now);
  }
  else
  {
    return ;
  }
}
