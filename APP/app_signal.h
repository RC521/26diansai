#ifndef APP_SIGNAL_H
#define APP_SIGNAL_H

#include <stdint.h>

/**
 * @brief 一阶低通滤波器结构体
 *
 * 采用一阶 IIR 滤波: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 * 截止频率通过 Alpha 系数控制。
 */
typedef struct {
    float   Value;       /**< 当前滤波输出值 */
    float   Alpha;       /**< 滤波系数 (0~1), 越小滤波越强 */
    uint8_t Initialized; /**< 初始化标志: 0 = 未初始化, 1 = 已初始化 */
} APP_LowPass_t;

/**
 * @brief 偏移量校准结构体
 *
 * 通过累加 N 个采样点求平均值，自动计算零漂/偏移量。
 * 常用于 ADC 采样的零点校准。
 */
typedef struct {
    float    Sum;          /**< 采样累加和 */
    float    Offset;       /**< 最终计算出的偏移量 (平均值) */
    uint32_t Count;        /**< 当前已采样计数 */
    uint32_t Target_Count; /**< 目标采样点数 (达到后校准完成) */
    uint8_t  Ready;        /**< 校准完成标志: 0 = 进行中, 1 = 已完成 */
} APP_OffsetCal_t;

/**
 * @brief 初始化低通滤波器
 *
 * 根据采样时间和截止频率计算滤波系数 Alpha，
 * 并标记为未初始化状态。
 *
 * @param filter      低通滤波器指针
 * @param sample_time 采样周期 (秒)
 * @param cutoff_hz   截止频率 (Hz)
 */
void APP_LowPass_Init(APP_LowPass_t *filter,
                      float sample_time, float cutoff_hz);

/**
 * @brief 复位/初始化低通滤波器输出值
 *
 * 将滤波器输出强制设置为指定值，用于快速建立稳态。
 *
 * @param filter 低通滤波器指针
 * @param value  初始输出值
 */
void APP_LowPass_Reset(APP_LowPass_t *filter, float value);

/**
 * @brief 低通滤波器更新
 *
 * 输入新采样值，返回滤波后的输出。
 * 首次调用时自动用输入值初始化，避免阶跃响应。
 *
 * @param filter 低通滤波器指针
 * @param input  当前采样输入值
 * @return 滤波后的输出值
 */
float APP_LowPass_Update(APP_LowPass_t *filter, float input);

/**
 * @brief 初始化偏移量校准
 *
 * @param calibration   偏移校准结构体指针
 * @param target_count  目标采样点数 (达到后自动完成)
 * @param initial_offset 初始偏移量 (在校准完成前使用)
 */
void APP_OffsetCal_Init(APP_OffsetCal_t *calibration,
                        uint32_t target_count, float initial_offset);

/**
 * @brief 偏移量校准更新
 *
 * 每次调用累加一个采样值，达到目标点数后自动计算平均值作为偏移量。
 * 校准完成后不再累加，直接返回完成状态。
 *
 * @param calibration 偏移校准结构体指针
 * @param sample      当前采样值 (已减去偏移的原始 ADC 值)
 * @return 0 = 校准进行中, 1 = 校准已完成
 */
uint8_t APP_OffsetCal_Update(APP_OffsetCal_t *calibration, float sample);

#endif
