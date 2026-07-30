/**
 * @file    app_signal.c
 * @brief   通用信号处理模块
 *
 * 提供：
 *   - 一阶低通滤波器 (IIR)
 *   - ADC 偏移量自动校准 (多点平均)
 */

#include "app_signal.h"

/** @brief 圆周率 (单精度) */
#define APP_PI 3.14159265f

/**
 * @brief 初始化低通滤波器
 *
 * 根据一阶 RC 滤波的离散化公式计算 Alpha 系数：
 *   omega_ts = 2 * pi * fc * Ts
 *   alpha    = omega_ts / (1 + omega_ts)
 *
 * 该系数通过双线性变换 (Tustin) 从连续域映射到离散域。
 *
 * @param filter      低通滤波器指针
 * @param sample_time 采样周期 Ts (秒)
 * @param cutoff_hz   截止频率 fc (Hz)
 */
void APP_LowPass_Init(APP_LowPass_t *filter,
                      float sample_time, float cutoff_hz)
{
    float omega_ts;

    omega_ts      = 2.0f * APP_PI * cutoff_hz * sample_time;
    filter->Alpha = omega_ts / (1.0f + omega_ts);
    filter->Value = 0.0f;
    filter->Initialized = 0U;
}

/**
 * @brief 复位/设置低通滤波器输出
 *
 * 直接覆盖滤波器内部状态为指定值。
 * 典型用途：
 *   - 系统启动时用第一个采样值初始化，避免从零爬升
 *   - 模式切换时重置滤波器状态
 *
 * @param filter 低通滤波器指针
 * @param value  目标输出值
 */
void APP_LowPass_Reset(APP_LowPass_t *filter, float value)
{
    filter->Value       = value;
    filter->Initialized = 1U;
}

/**
 * @brief 低通滤波器更新
 *
 * 一阶 IIR 差分方程：
 *   y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *
 * 首次调用 (Initialized == 0) 时，自动用输入值初始化滤波器状态，
 * 避免从 0 到稳态的漫长爬升过程。
 *
 * @param filter 低通滤波器指针
 * @param input  当前输入采样值 x[n]
 * @return 滤波后输出值 y[n]
 */
float APP_LowPass_Update(APP_LowPass_t *filter, float input)
{
    if (filter->Initialized == 0U) {
        /* 首次调用: 直接设置为输入值，跳过暂态响应 */
        APP_LowPass_Reset(filter, input);
    } else {
        /* 一阶 IIR 滤波 */
        filter->Value += filter->Alpha * (input - filter->Value);
    }

    return filter->Value;
}

/**
 * @brief 初始化偏移量校准
 *
 * 设置目标采样点数，清零累加器，标记为未完成状态。
 * 在校准完成前可使用 initial_offset 作为临时偏移量。
 *
 * @param calibration    偏移校准结构体指针
 * @param target_count   目标采样点数 N (校准 = N 个采样值的算术平均)
 * @param initial_offset 初始/默认偏移量
 */
void APP_OffsetCal_Init(APP_OffsetCal_t *calibration,
                        uint32_t target_count, float initial_offset)
{
    calibration->Sum          = 0.0f;
    calibration->Offset       = initial_offset;
    calibration->Count        = 0U;
    calibration->Target_Count = target_count;
    calibration->Ready        = 0U;
}

/**
 * @brief 偏移量校准更新
 *
 * 每次调用累加一个采样值:
 *   - 未完成: 累加 sample 到 Sum, Count++
 *   - 达到 Target_Count: 计算 Offset = Sum / Target_Count, 置 Ready = 1
 *   - 已完成: 不再累加，直接返回 1
 *
 * 校准完成后 Offset 即为 N 个采样点的算术平均值，
 * 可用作 ADC 零漂/偏移补偿值。
 *
 * @param calibration 偏移校准结构体指针
 * @param sample      当前采样值 (原始 ADC 读数)
 * @return 0 = 校准进行中, 1 = 校准已完成
 */
uint8_t APP_OffsetCal_Update(APP_OffsetCal_t *calibration, float sample)
{
    /* 已完成则直接返回，不再累加 */
    if (calibration->Ready != 0U) {
        return 1U;
    }

    /* 累加采样值 */
    calibration->Sum += sample;
    calibration->Count++;

    /* 达到目标点数: 计算平均值作为偏移量 */
    if (calibration->Count >= calibration->Target_Count) {
        calibration->Offset = calibration->Sum /
                              (float)calibration->Target_Count;
        calibration->Ready = 1U;
    }

    return calibration->Ready;
}
