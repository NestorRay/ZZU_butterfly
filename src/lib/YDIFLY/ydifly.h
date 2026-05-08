/**
 * @file     : ydifly.h
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
#ifndef __YDIFLY_H

#define __YDIFLY_H

#include <Arduino.h>


void YDIFlyControl( unsigned long now_time_ms );


#endif //__YDIFLY_H
