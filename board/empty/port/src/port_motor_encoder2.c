/**
 * @file    port_motor_encoder2.c
 * @brief   Port 层电机2编码器实现
 * @note    硬件架构: "自动捕获 + DMA 环形搬运 + 软件轮询解码"
 *
 *           TIMG7 (TB6612_ENB) → Edge-Time Capture, CCP0 = PA28 (A相) 上升沿
 *           DMA_CH0 → 源地址 = &GPIOA->DIN31_0 (总线快照), 目标地址 = g_enc2Buffer
 *           触发链: TIMG7 CC0_DN_EVENT → Publisher CH1 → DMA Generic Sub1 Trigger
 *
 *           10ms 轮询解码:
 *           ✅ 每个 DMA 快照条目 = A 相上升沿时刻的 GPIOA 端口快照
 *           ✅ 快照中 bit28 = A 相电平 (=1, 因为是上升沿), bit29 = B 相电平
 *           ✅ B=0 → 正转脉冲, B=1 → 反转脉冲
 *           ✅ 无需软件附加判向, 方向信息已在硬件层面同步锁存
 *
 *           编码器参数: 13 线 × 4 倍频 = 52 脉冲/电机转
 *           减速比 1:30 → 1560 脉冲/输出轴转
 */

#include "port_motor_encoder2.h"
#include "ti_msp_dl_config.h"
#include "port_log.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ================================================================
 *  环形缓冲区定义
 * ================================================================ */

/**
 * @brief DMA 环形缓冲区
 * @note  每条 32-bit, 内容是 GPIOA->DIN31_0 在 A 相上升沿时刻的快照
 */
static uint32_t g_enc2Buffer[ENC2_BUF_SIZE];

/**
 * @brief 软件读指针 (上次轮询结束时的位置)
 * @note  初始化为 0, 每次 Poll 后更新为 DMA 写指针位置
 */
static uint32_t s_readIndex = 0;

/**
 * @brief 首次轮询标志
 */
static bool s_firstPoll = true;

/* ================================================================
 *  获取 DMA 硬件写指针偏移
 * ================================================================ */

uint32_t PORT_MOTOR_ENCODER2_GetDmaWriteOffset(void)
{
    /*
     * DMA 目的地址寄存器保存的是"下一次写入的地址"。
     * 通过计算 (当前目的地址 - 缓冲区基址) / sizeof(条目) 得到已写入条目数。
     *
     * 注意: MSPM0 DMA 目的地址在每次传输后递增 sizeof(destWidth) 字节。
     * destWidth = DL_DMA_WIDTH_WORD = 4 字节。
     */
    uint32_t destAddr = DL_DMA_getDestAddr(DMA, DMA_CH0_CHAN_ID);
    uint32_t baseAddr = (uint32_t)&g_enc2Buffer[0];
    uint32_t offset   = (destAddr - baseAddr) / sizeof(uint32_t);

    /*
     * 如果 offset 超出缓冲区范围 (DMA 已绕回但 SDM 未取模),
     * 说明 DMA 已写满一整圈。此时对缓冲区大小取模。
     */
    if (offset >= ENC2_BUF_SIZE) {
        offset %= ENC2_BUF_SIZE;
    }

    return offset;
}

/* ================================================================
 *  初始化
 * ================================================================ */

void PORT_MOTOR_ENCODER2_Init(void)
{
    /*
     * 1. 清零环形缓冲区
     */
    for (uint32_t i = 0; i < ENC2_BUF_SIZE; i++) {
        g_enc2Buffer[i] = 0;
    }
    s_readIndex  = 0;
    s_firstPoll  = true;

    /*
     * 2. 配置 DMA_CH0 源地址 → GPIOA 输入数据寄存器
     *    GPIOA->DIN31_0 是 32-bit 只读寄存器, 反映 GPIOA 端口 32 个引脚的
     *    实时数字电平 (PA0~PA31)。
     *
     *    在 A 相上升沿触发的 DMA 周期中, 硬件总线读取该寄存器,
     *    获得边沿时刻 PA28 (A相) 和 PA29 (B相) 的同步快照。
     */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)&GPIOA->DIN31_0);

    /*
     * 3. 配置 DMA_CH0 目标地址 → 环形缓冲区基址
     */
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t)&g_enc2Buffer[0]);

    /*
     * 4. 配置 DMA_CH0 传输大小
     *    NORMAL 模式下, 传输 ENC2_BUF_SIZE 次后 DMA 自动停止。
     *    由于缓冲区是 256 条, 10ms 轮询周期内最大脉冲数远小于 256,
     *    每次 Poll 后软件将 DMA 目的地址重置为缓冲区基址,
     *    实现软件控制的环形回绕。
     *
     *    备选方案: 使用 DMA 循环模式 (如需硬件自动回绕)。
     */
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, ENC2_BUF_SIZE);

    /*
     * 5. 启动 DMA 通道 (使能触发响应)
     */
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    /*
     * 6. 启动 TIMG7 计数器
     *    SYSCFG_DL_TB6612_ENB_init() 已将 TIMG7 配置为 Edge-Time Capture 模式,
     *    但 startTimer = DL_TIMER_STOP。此处显式启动计数。
     *
     *    计数器开始运行后, PA28 的首次上升沿将:
     *      1. TIMG7 CC0 锁存 CNT 值到 CC0 寄存器 (Edge-Time)
     *      2. 产生 CC0_DN_EVENT → Publisher CH1 → DMA Generic Sub1 Trigger
     *      3. DMA 搬运 1 个 WORD: GPIOA->DIN31_0 → g_enc2Buffer[n]
     */
    DL_TimerG_startCounter(TB6612_ENB_INST);

    LOG_INFO("[ENC2] DMA-GPIO snapshot encoder initialized\r\n");
}

/* ================================================================
 *  10ms 轮询解码
 * ================================================================ */

void PORT_MOTOR_ENCODER2_Poll(int32_t *deltaPulses)
{
    uint32_t writeOffset = PORT_MOTOR_ENCODER2_GetDmaWriteOffset();
    int32_t  pulseCount  = 0;

    if (s_firstPoll)
    {
        /*
         * 首次轮询: 仅同步读指针到当前写位置, 不输出增量。
         * 避免初始化前 DMA 已写入的旧数据被当成有效脉冲。
         */
        s_readIndex  = writeOffset;
        s_firstPoll  = false;
        *deltaPulses = 0;
        return;
    }

    if (writeOffset == s_readIndex)
    {
        /* 无新数据 */
        *deltaPulses = 0;
        return;
    }

    /*
     * 遍历自上次轮询以来 DMA 新写入的所有快照条目。
     *
     * 每个条目是 GPIOA->DIN31_0 在 A 相上升沿时刻的快照:
     *   - bit28 = A 相电平 (=1, 因为是上升沿, 捕获机制保证了这一点)
     *   - bit29 = B 相电平
     *
     * 方向判定:
     *   - A 相上升沿时 B=0 → 正转 (A 超前 B)
     *   - A 相上升沿时 B=1 → 反转 (B 超前 A)
     *
     * 每条快照 = 1 个有效脉冲边沿 (4 倍频的单个步进)。
     * 一个完整的编码器周期 (A 完整的 1 个 HIGH+1 个 LOW) 产生 4 个快照:
     *   A↑(B=L), A↓(B=H), A↑(B=L), A↓(B=H) → 4 脉冲 = 1 编码器周期
     *   对应 13 线编码器: 4 × 13 = 52 脉冲/电机转
     */
    uint32_t idx = s_readIndex;

    while (idx != writeOffset)
    {
        uint32_t snapshot = g_enc2Buffer[idx];

        /*
         * A 相电平 (bit28): 理论值为 1 (上升沿), 但为健壮性做验证
         * B 相电平 (bit29): 0=LOW → 正转, 1=HIGH → 反转
         */
        if (snapshot & ENC2_BIT_A)
        {
            /* A 相为高, 确认这是一个有效的上升沿快照 */
            if (snapshot & ENC2_BIT_B)
            {
                pulseCount--;   /* B=HIGH → 反转 */
            }
            else
            {
                pulseCount++;   /* B=LOW  → 正转 */
            }
        }
        /*
         * 理论上不应出现 A=0 的情况 (DMA 只在上升沿触发),
         * 但若出现 (如噪声或 DMA 时序边界), 忽略该条目。
         */

        /* 环形缓冲区索引递增 */
        idx++;
        if (idx >= ENC2_BUF_SIZE)
        {
            idx = 0;
        }
    }

    /* 更新软件读指针 */
    s_readIndex = writeOffset;

    /*
     * 重置 DMA 目的地址到缓冲区头部
     *
     * NORMAL 模式下 DMA 写满 ENC2_BUF_SIZE 条后停止。
     * 此处每次 Poll 后都将 DMA 目的地址重置为缓冲区基址,
     * 实现软件控制的环形回绕。
     *
     * 代价: 单次 DMA 调用开销 < 1 µs, 每 10ms 一次, 可忽略。
     *
     * 如果 DMA 的 destAddr 仍在缓冲区范围内 (未溢出), 不重置。
     * 仅在接近缓冲区末尾时重置, 避免干扰当前 DMA 传输。
     */
    if (writeOffset >= (ENC2_BUF_SIZE - 16U))
    {
        DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
            (uint32_t)&g_enc2Buffer[0]);
        DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, ENC2_BUF_SIZE);
        DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
        s_readIndex = 0;
    }

    *deltaPulses = pulseCount;
}