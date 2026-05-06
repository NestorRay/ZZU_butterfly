/**
 * @file     : ydifly.cpp
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
#include "ydifly.h"
#include "CRSF.h"
#include "common.h"

extern void startWaveform8266(uint8_t pin, uint32_t timeHighUS, uint32_t timeLowUS);
extern void stopWaveform8266(uint8_t pin);

ydifly_control_t ydifly;
extern connectionState_e connectionState;

static void YDIFlyInit( void )
{
    ydifly.remote.raw = CRSF::ChannelData;

    YDIFlyRemoteDecode( &ydifly.remote );
    YDIFlyRemoteDecode( &ydifly.remote_last );

    YDIFlyServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_INIT );  // 上电时，电机运行在初始位置
    YDIFlyServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_INIT );  // 上电时，电机运行在初始位置
}

void YDIFlyControl( unsigned long now_time_ms )
{
    static unsigned long last_time_ms_control = now_time_ms;    // 上一次时间
    static unsigned long last_time_ms_debug = now_time_ms;      // 上一次时间
    static unsigned long last_time_ms = now_time_ms;            // 上一次时间，仅用于时间溢出检测
    static bool wasFlapping = false;                             // 标记上一次是否为扑翼模式，用于静态模式同步启动舵机
    float angle_l_max, angle_l_min, angle_r_max, angle_r_min, control_T;
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
            last_time_ms_control += YDIFLY_CONTROL_CYCLE;     // 时间补全

            /* 获取遥控解算数据 */
            YDIFlyRemoteDecode( &ydifly.remote );

            if( ydifly.remote.freq > 10 )    // 如果有给油门，则进入起飞程序
            {
                /* 翅膀扑翼幅度解算 */
                if     ( ydifly.remote.swb == 0 )        ydifly.remote.amp = YDIFLY_AMP0;   // 不同档位幅值不同
                else if( ydifly.remote.swb == 1 )        ydifly.remote.amp = YDIFLY_AMP1;   // 不同档位幅值不同
                else                                     ydifly.remote.amp = YDIFLY_AMP2;   // 不同档位幅值不同

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

                /* 舵机范围控制 */
                angle_l_max = YDIFLY_SERVO_ANGLE_L_INIT - ydifly.remote.yaw*YDIFLY_FACTOR_YAW - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH - ydifly.remote.offset*YDIFLY_FACTOR_OFFSET + ydifly.remote.amp;
                angle_l_min = YDIFLY_SERVO_ANGLE_L_INIT + ydifly.remote.yaw*YDIFLY_FACTOR_YAW - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH - ydifly.remote.offset*YDIFLY_FACTOR_OFFSET - ydifly.remote.amp;
                angle_r_max = YDIFLY_SERVO_ANGLE_L_INIT + ydifly.remote.yaw*YDIFLY_FACTOR_YAW - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH + ydifly.remote.offset*YDIFLY_FACTOR_OFFSET + ydifly.remote.amp;
                angle_r_min = YDIFLY_SERVO_ANGLE_L_INIT - ydifly.remote.yaw*YDIFLY_FACTOR_YAW - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH + ydifly.remote.offset*YDIFLY_FACTOR_OFFSET - ydifly.remote.amp;

                /* 限幅 */
                if( angle_l_max > YDIFLY_SERVO_ANGLE_L_MAX )                angle_l_max = YDIFLY_SERVO_ANGLE_L_MAX;
                if( angle_l_min < YDIFLY_SERVO_ANGLE_L_MIN )                angle_l_min = YDIFLY_SERVO_ANGLE_L_MIN;
                if( angle_r_max > YDIFLY_SERVO_ANGLE_R_MAX )                angle_r_max = YDIFLY_SERVO_ANGLE_R_MAX;
                if( angle_r_min < YDIFLY_SERVO_ANGLE_R_MIN )                angle_r_min = YDIFLY_SERVO_ANGLE_R_MIN;

                /* 舵机控制正弦周期 */
                control_T = YDIFLY_CYCLE_MAX + ydifly.remote.freq*(YDIFLY_CYCLE_MIN - YDIFLY_CYCLE_MAX)/1500;

                /* 舵机角度控制 */
                YDIFlyServoSinControl( angle_l_max, angle_l_min, angle_r_max, angle_r_min, control_T, YDIFLY_SPEED_DIFF );
                wasFlapping = true;
            }
            else
            {
                /* 从扑翼模式切换到静态模式时，先停止两侧波形再同步启动，避免异步上升沿导致舵机不同步 */
                if( wasFlapping )
                {
                    stopWaveform8266(YDIFLY_SERVO_L_PIN);
                    stopWaveform8266(YDIFLY_SERVO_R_PIN);
                    wasFlapping = false;
                }
                if( ydifly.remote.swc == 2 )     // 如果拨动开关，翅膀摆动。拨杆到上档位
                {
                    YDIFlyServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MIN );
                    YDIFlyServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MIN );
                }
                else if( ydifly.remote.swc == 1 )     // 舵机处于初始位置。拨杆到中档位
                {
                    YDIFlyServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_INIT - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH - ydifly.remote.offset*YDIFLY_FACTOR_OFFSET );
                    YDIFlyServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_INIT - ydifly.remote.pitch*YDIFLY_FACTOR_PITCH + ydifly.remote.offset*YDIFLY_FACTOR_OFFSET );
                }
                else    // 如果拨动开关，翅膀摆动。拨杆到下档位
                {
                    YDIFlyServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MAX );
                    YDIFlyServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MAX );
                }
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
    static float time_now = 0;

    /* 输入参数保护 */
    if( speed_diff > YDIFLY_CONTROL_CYCLE )         speed_diff = YDIFLY_CONTROL_CYCLE-1;
    else if( speed_diff < -YDIFLY_CONTROL_CYCLE )   speed_diff =-YDIFLY_CONTROL_CYCLE+1;
    // (l_angle_max-l_angle_min)/2 将标准正弦函数的幅值放大到所需的扑翼震动角度
    // (l_angle_max+l_angle_min)/2 将标准正弦函数的中性点水平平移
    // sin( time_now*6.283185307179586/T )生成目前时间下的[-1,1]的系数
    angle_set = ((l_angle_max-l_angle_min)/2)*sin( time_now*6.283185307179586/T ) + (l_angle_max+l_angle_min)/2;
    YDIFlyServoAngleControl( SERVO_L, angle_set );

    angle_set = ((r_angle_max-r_angle_min)/2)*sin( time_now*6.283185307179586/T ) + (r_angle_max+r_angle_min)/2;
    YDIFlyServoAngleControl( SERVO_R, angle_set );

    time_now += YDIFLY_CONTROL_CYCLE;
    if( (time_now > T*0.25f) && (time_now < T*0.75f) )  time_now -= speed_diff;
    else                                                time_now += speed_diff;

    if( time_now > T )
    {
        time_now -= T;
    }    
}

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
