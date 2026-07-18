/*
 * FreeRTOS Kernel V10.4.6 or later
 * 专门针对 TI MSPM0G3507 (ARM Cortex-M0+) 裁剪优化的配置文件
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "ti_msp_dl_config.h" // 引入 TI 库的配置，方便后面共享主频

/*-----------------------------------------------------------
 * 核心架构与时钟配置
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1       // 1: 抢占式调度器；0: 协程式调度器
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0       // M0+内核不支持硬件计算前导零，设为0
#define configUSE_TICKLESS_IDLE                 0       // 1: 开启低功耗Tickless模式

// 关键：将 FreeRTOS 的内核时钟和你在 SysConfig 里配置的 80MHz MCLK 绑定
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 80000000 ) 
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 ) // 系统心跳：1ms一次(1000Hz)

#define configMAX_PRIORITIES                    ( 5 )   // 任务最大优先级（0~4），根据需求可调大
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 64 ) // 空闲任务堆栈大小（字，即256字节）
#define configMAX_TASK_NAME_LEN                 ( 16 )  // 任务名称最大长度

#define configUSE_16_BIT_TICKS                  0       // 0: 使用32位Tick计数器（防溢出）
#define configIDLE_SHOULD_YIELD                 1       // 1: 空闲任务让步给同优先级的用户任务

/*-----------------------------------------------------------
 * 任务间通信与内存管理
 *----------------------------------------------------------*/
#define configUSE_MUTEXES                       1       // 支持互斥量
#define configUSE_RECURSIVE_MUTEXES             1       // 支持递归互斥量
#define configUSE_COUNTING_SEMAPHORES           1       // 支持计数信号量
#define configUSE_QUEUE_SETS                    0       // 队列集（选用）
#define configSUPPORT_DYNAMIC_ALLOCATION        1       // 支持动态内存分配（必须配合 heap_x.c）
#define configSUPPORT_STATIC_ALLOCATION         0       // 关闭静态分配（省事）

// 关键：给 FreeRTOS 堆分配的空间（字节）。MSPM0G3507 有 32KB SRAM，这里给系统分配 12KB
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 18 * 1024 ) ) 

/*-----------------------------------------------------------
 * 钩子函数配置（通常设为0，省去手写空函数的麻烦）
 *----------------------------------------------------------*/
#define configUSE_IDLE_HOOK                     0       // 空闲钩子
#define configUSE_TICK_HOOK                     0       // 时间片钩子
#define configCHECK_FOR_STACK_OVERFLOW          0       // 堆栈溢出检查（调试时可设为1或2）
#define configUSE_MALLOC_FAILED_HOOK            0       // 内存分配失败钩子

/*-----------------------------------------------------------
 * 运行时间和任务状态统计
 *----------------------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS           0       // 统计运行时间
#define configUSE_TRACE_FACILITY                0       // 可视化追踪
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/*-----------------------------------------------------------
 * 软件定时器配置
 *----------------------------------------------------------*/
#define configUSE_TIMERS                        1       // 启用软件定时器
#define configTIMER_TASK_PRIORITY               ( 4 )   // 定时器任务优先级（设为最高）
#define configTIMER_QUEUE_LENGTH                ( 10 )  // 定时器命令队列长度
#define configTIMER_TASK_STACK_DEPTH            ( 96 )

/*-----------------------------------------------------------
 * 可选的 API 函数使能 (1: 使能；0: 禁用)
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerPendFunctionCall          1

#define configENABLE_MPU                        0       // 0: 禁用 MPU；1: 启用 MPU

#endif /* FREERTOS_CONFIG_H */