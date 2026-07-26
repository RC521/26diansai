#include "bsp.h"

static KEY_t key_list[] =
{
    KEY_CONFIG_LIST 
};

#define KEY_NUM (sizeof(key_list) / sizeof(key_list[0]))



static KEY_t *KEY_Find(KEY_ID_t id)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        if (key_list[i].id == id)
        {
            return &key_list[i];
        }
    }

    return 0;
}

void KEY_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {

        GPIO_PinState level = HAL_GPIO_ReadPin(key_list[i].port, key_list[i].pin);
        key_list[i].last_raw_level = level;
        key_list[i].stable_level = level;
        key_list[i].debounce_count = 0;
        key_list[i].pressed = (level == key_list[i].active_level);
        key_list[i].press_event = 0;
        key_list[i].release_event = 0;
    }
}

void KEY_IRQHandler(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        // 1. 读取当前原始电平
        GPIO_PinState raw_level = HAL_GPIO_ReadPin(key_list[i].port, key_list[i].pin);
        // 2. 电平发生变化 → 重置消抖计数器
        if (raw_level != key_list[i].last_raw_level)
        {
            key_list[i].last_raw_level = raw_level;
            key_list[i].debounce_count = 0;
            continue;
        }
        // 3. 电平没变化 → 消抖计数+1
        if (key_list[i].debounce_count < KEY_DEBOUNCE_TICKS)
        {
            key_list[i].debounce_count++;
            continue;
        }
        // 4. 消抖完成 → 电平稳定，更新状态
        if (raw_level != key_list[i].stable_level)
        {
            key_list[i].stable_level = raw_level;
            key_list[i].pressed = (raw_level == key_list[i].active_level);  //判断按键状态是否和有效电平一致
            // 5. 产生事件标志
            if (key_list[i].pressed)
            {
                key_list[i].press_event = 1; // 按下
            }
            else
            {
                key_list[i].release_event = 1;// 释放
            }
        }
    }
}

uint8_t KEY_IsPressed(KEY_ID_t id)
{
    KEY_t *key = KEY_Find(id);

    if (key == 0)
    {
        return 0;
    }

    return key->pressed;
}

uint8_t KEY_GetPressEvent(KEY_ID_t id)
{
    KEY_t *key = KEY_Find(id);
    uint32_t primask;
    uint8_t event;

    if (key == 0)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    event = key->press_event;
    key->press_event = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return event;
}

uint8_t KEY_GetReleaseEvent(KEY_ID_t id)
{
    KEY_t *key = KEY_Find(id);
    uint32_t primask;
    uint8_t event;

    if (key == 0)
    {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    event = key->release_event;
    key->release_event = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return event;
}



void KEY_USE(void)
{

    if (KEY_GetReleaseEvent(KEY_ID_1))
    {
        
    }

    if (KEY_GetReleaseEvent(KEY_ID_2))
    {
        
    }

    if (KEY_GetReleaseEvent(KEY_ID_3))
    {
    }

    if (KEY_GetReleaseEvent(KEY_ID_4))
    {
       
    }

    if (KEY_GetReleaseEvent(KEY_ID_5))
    {
        
    }

    if (KEY_GetReleaseEvent(KEY_ID_6))
    {
        
    }
}
