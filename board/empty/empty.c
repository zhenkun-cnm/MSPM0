#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"

// 任务句柄
TaskHandle_t StartTask_Handler;
TaskHandle_t LEDTask_Handler;

// 任务函数声明
void start_task(void *pvParameters);
void led_task(void *pvParameters);

int main(void)
{
    // 1. 初始化 TI 默认库外设（GPIO, UART等，此时不要包含用户自写SysTick）
    SYSCFG_DL_init();

    // 2. 创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            // 任务函数
                (const char* )"start_task",          // 任务名称
                (uint16_t       )128,                   // 任务堆栈大小
                (void* )NULL,                  // 传入参数
                (UBaseType_t    )1,                     // 任务优先级
                (TaskHandle_t* )&StartTask_Handler);   // 任务句柄              
    
    // 3. 开启任务调度（FreeRTOS 会在此处自动初始化并启动 SysTick 计数）
    vTaskStartScheduler();          
}

// 开始任务：用来创建其他应用任务，创建完后自行删除
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           // 进入临界区

    // 创建 LED 闪烁任务
    xTaskCreate((TaskFunction_t )led_task,     
                (const char* )"led_task",   
                (uint16_t       )128,          
                (void* )NULL,         
                (UBaseType_t    )2,             // 优先级略高
                (TaskHandle_t* )&LEDTask_Handler); 

    vTaskDelete(StartTask_Handler); // 删除开始任务
    
    taskEXIT_CRITICAL();            // 退出临界区
}

// LED 闪烁应用任务
void led_task(void *pvParameters)
{
    while(1)
    {
        // 调用之前 SysConfig 生成的默认库函数翻转 PB22
        DL_GPIO_togglePins(LED_PORT, LED_PIN_22_PIN);
        
        // 操作系统级别的延时：500个Tick (在1000Hz配置下即为500ms)
        // 延时期间 CPU 会自动去跑其他任务，不再死等
        vTaskDelay(500); 
    }
}