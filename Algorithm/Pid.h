#ifndef __PID_H
#define __PID_H

/* PI 控制器结构体定义 */
typedef struct {
    // 1. 核心调节参数 (需要你根据实际硬件去尝试和修改的值)
    float Kp;           // 比例系数：反应速度
    float Ki;           // 积分系数：消除静差
    
    // 2. 历史状态 (算法内部自己维护，外部不需要管)
    float Integral;     // 历史误差的累加和
    
    // 3. 保护参数 (防止占空比或计算结果爆炸)
    float Out_Max;      // 输出上限
    float Out_Min;      // 输出下限
    
    // 4. 最终输出结果
    float Output;       // 计算得出的控制量
} PI_Controller_t;

/* 函数声明 */
void PI_Init(PI_Controller_t *pi, float kp, float ki, float max, float min);
float PI_Update(PI_Controller_t *pi, float target, float actual);

#endif