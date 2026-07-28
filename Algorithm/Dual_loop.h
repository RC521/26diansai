#ifndef __DUAL_LOOP_H
#define __DUAL_LOOP_H

#include "pid.h" // 务必包含你之前写的 PI 基类
#include "Pr.h"

/* * 定义双环控制器超级结构体
 * 物理意义：将电压外环和电流内环打包成一个完整的系统
 */
typedef struct {
    PI_Controller_t Voltage_PI;  // 外环PI：老板（根据电压误差，下达电流 KPI）
    PI_Controller_t Current_PI;  // 内环PI：主管（根据电流误差，极速调节 PWM 调制比）
    PR_Controller_t Current_PR;  // 内环PR：主管（根据电流误差，极速调节 PWM 调制比）
} DualLoop_t;

// 函数声明
void DualLoop_Init(DualLoop_t *dual_loop, float sample_time);
float DualLoop_Update(DualLoop_t *dual_loop, float target_voltage, float actual_voltage, float actual_current);

#endif
