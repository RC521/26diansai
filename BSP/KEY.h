#ifndef __KEY_H_
#define __KEY_H_

#include "main.h"
#include <stdint.h>

/*
 * Change key pins here.
 * KEY_ACTIVE_LEVEL is GPIO_PIN_RESET for a pull-up key, GPIO_PIN_SET for a pull-down key.
 */
#define KEY_ACTIVE_LEVEL       GPIO_PIN_RESET     // 有效电平：低电平有效
#define KEY_GPIO_PULL          GPIO_NOPULL        // 引脚上下拉：无上下拉
#define KEY_DEBOUNCE_TICKS     2U                 // 消抖时间：0（可修改）

typedef enum
{
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_5,
    KEY_ID_6
} KEY_ID_t;

#define KEY_CONFIG_LIST                                      \
    {KEY_ID_1, GPIOD, GPIO_PIN_3, KEY_ACTIVE_LEVEL},        \
    {KEY_ID_2, GPIOC, GPIO_PIN_12,  KEY_ACTIVE_LEVEL},        \
    {KEY_ID_3, GPIOD, GPIO_PIN_2,  KEY_ACTIVE_LEVEL},        \
    {KEY_ID_4, GPIOD, GPIO_PIN_4,  KEY_ACTIVE_LEVEL},        \
    {KEY_ID_5, GPIOD, GPIO_PIN_1,  KEY_ACTIVE_LEVEL},        \
    {KEY_ID_6, GPIOD, GPIO_PIN_0,  KEY_ACTIVE_LEVEL}

typedef struct
{
    // 1. 硬件配置（固定不变）
    KEY_ID_t id;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_level;

    // 2. 消抖与状态变量（运行中动态变化）
    GPIO_PinState last_raw_level;  // 上一次原始电平
    GPIO_PinState stable_level;    // 稳定后的电平
    uint8_t debounce_count;        // 消抖计数器

    // 3. 按键状态与事件（volatile：防止编译器优化）
    volatile uint8_t pressed;        // 是否正在按下
    volatile uint8_t press_event;    // 按下事件标志
    volatile uint8_t release_event;   // 释放事件标志
} KEY_t;

void KEY_Init(void);
void KEY_IRQHandler(void);

uint8_t KEY_IsPressed(KEY_ID_t id);            // 查询是否正在按下
uint8_t KEY_GetPressEvent(KEY_ID_t id);        // 获取按下事件（获取后事件标志会被清除）
uint8_t KEY_GetReleaseEvent(KEY_ID_t id);      // 获取释放事件（获取后事件标志会被清除）
void KEY_USE(void);

#endif
