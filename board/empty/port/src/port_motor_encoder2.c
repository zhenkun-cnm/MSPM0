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
 *           编码器参数: DMA 仅捕获 A 相上升沿(1x), App 层乘 4 折算
 *           减速比 1:20, 实测约 985 脉冲/输出轴转
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

/**
 * @brief 诊断计数器 (每 100 次 Poll 打印一次 DMA 状态, 即每 1 秒)
 */
static uint32_t s_diagCounter = 0;

/**
 * @brief 上一次 Poll 的 writeOffset, 用于检测是否有新数据传输
 */
static uint32_t s_lastWriteOffset = 0;

/* ================================================================
 *  获取 DMA 硬件写指针偏移
 * ================================================================ */

uint32_t PORT_MOTOR_ENCODER2_GetDmaWriteOffset(void)
{
    /*
     * DMA 循环模式: DL_DMA_getTransferSize() 返回当前循环剩余传输次数。
     * 已写入条目数 = 总大小 - 剩余次数。
     *
     * 例如: ENC2_BUF_SIZE=256, remaining=200 → 已写入 56 条 → offset=56。
     *
     * 注意: 只有循环模式/Circular 下 getTransferSize 才持续有效。
     * NORMAL 模式下传完 256 次后返回 0, 不代表写入 256 条。
     */
    uint16_t remaining = DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
    uint32_t offset    = (ENC2_BUF_SIZE - remaining) % ENC2_BUF_SIZE;

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
     * 4. DMA_CH0 循环模式 (DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE)
     *    硬件自动回绕到缓冲区头部, 无需 CPU 干预。
     *    TransferSize 由 SYSCFG_DL_DMA_CH0_init() 设置 (=256)。
     */

    /*
     * 5. 手动将 DMA 的 FSUB_1 (Generic Subscriber 1) 订阅到
     *    Event Fabric 的 Publisher Channel 1。
     *    TIMG7 通过 setPublisherChanID 发布事件到通道 1,
     *    DMA 通过 setSubscriberChanID 订阅通道 1,
     *    两者通过 Event Fabric 内部总线直连。
     *
     *    没有这一行: TIMG7 捕获事件产生但 DMA 收不到 (DMA-rem=256)。
     *    加上这一行: PA28 上升沿 → TIMG7 → DMA → 自动搬运 GPIO 快照。
     */
    DL_DMA_setSubscriberChanID(DMA, DL_DMA_SUBSCRIBER_INDEX_1, 1);

    /*
     * 6. 启动 DMA 通道 (使能触发响应)
     */
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    /*
     * 7. 启动 TIMG7 计数器
     *    SYSCFG_DL_TB6612_ENB_init() 已将 TIMG7 配置为 Edge-Time Capture 模式,
     *    但 startTimer = DL_TIMER_STOP。此处显式启动计数。
     *
     *    计数器开始运行后, PA28 的首次上升沿将:
     *      1. TIMG7 CC0 锁存 CNT 值到 CC0 寄存器 (Edge-Time)
     *      2. 产生 CC0_DN_EVENT → Publisher CH1 → DMA Generic Sub1 Trigger
     *      3. DMA 搬运 1 个 WORD: GPIOA->DIN31_0 → g_enc2Buffer[n]
     */
    DL_TimerG_startCounter(TB6612_ENB_INST);

    /*
     * 诊断: 验证 TIMG7 计数器是否在跑
     * 延迟 1ms 后读两次 CTR, 差值应 > 0
     */
    uint16_t cnt1 = (uint16_t)(TIMG7->COUNTERREGS.CTR);
    DL_Common_delayCycles(80000);  /* 1ms @ 80MHz */
    uint16_t cnt2 = (uint16_t)(TIMG7->COUNTERREGS.CTR);
    uint16_t cntDiff = cnt2 - cnt1;

    /* LOG_INFO("[ENC2] CNT1=%u CNT2=%u diff=%u %s\r\n",
        cnt1, cnt2, cntDiff,
        (cntDiff > 0) ? "RUNNING" : "STOPPED"); */

    /*
     * 诊断: 打印 DMA 传输大小初始值
     */
    uint16_t dmaRem = DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
    /* LOG_INFO("[ENC2] DMA remaining=%u src=0x%08lX dest=0x%08lX\r\n",
        dmaRem,
        (uint32_t)&GPIOA->DIN31_0,
        (uint32_t)&g_enc2Buffer[0]);

    LOG_INFO("[ENC2] DMA-GPIO snapshot encoder initialized\r\n"); */
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

        /*
         * 每 1 秒 (100 次 Poll) 打印一次诊断信息,
         * 方便确认 DMA 是否在编码器转动时收到触发。
         */
        s_diagCounter++;
        if (s_diagCounter >= 100U)
        {
            s_diagCounter = 0;

            /* 读取 DMA 剩余传输次数 */
            uint16_t dmaRem = DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
            uint32_t offset = (ENC2_BUF_SIZE - dmaRem) % ENC2_BUF_SIZE;

            /*
             * 读取 TIMG7 中断标志位, 查看 CC0_DN_EVENT 是否触发过
             * RIS (Raw Interrupt Status) 不需要中断使能即可读取
             */
            uint32_t timg7IIDX = TIMG7->CPU_INT.IIDX;
            uint32_t timg7RIS  = TIMG7->CPU_INT.RIS;
            uint16_t timg7CC0  = (uint16_t)DL_TimerG_getCaptureCompareValue(
                                     TB6612_ENB_INST, DL_TIMER_CC_0_INDEX);
            uint16_t timg7CTR  = (uint16_t)(TIMG7->COUNTERREGS.CTR);

            /* LOG_INFO("[ENC2 DIAG] DMA-rem=%u offset=%u rdIdx=%u"
                     " CC0=%u CTR=%u"
                     " IIDX=0x%02lX RIS=0x%04lX"
                     " %s\r\n",
                dmaRem, offset, s_readIndex,
                timg7CC0, timg7CTR,
                timg7IIDX, timg7RIS,
                (writeOffset != s_lastWriteOffset) ? "DATA!" : "no-data"); */

            s_lastWriteOffset = writeOffset;
        }

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
     * 每条快照 = 1 个 A 相上升沿原始脉冲。
     * App 层再乘以 4, 将 1x 捕获值折算为与 M1 QEI 相近的 4x 标尺。
     * 对应 13 线编码器: 原始 13 捕获/电机转, 折算后 52 脉冲/电机转。
     */
    uint32_t idx = s_readIndex;

    while (idx != writeOffset)
    {
        uint32_t snapshot = g_enc2Buffer[idx];

        /*
         * A 相电平 (bit28): 理论值为 1 (上升沿), 但为健壮性做验证
         * B 相电平 (bit29): 0=LOW → 正转, 1=HIGH → 反转
         */
        /*
         * DMA 仅在 A 相上升沿被触发, 因此每条快照 = 1 个有效脉冲。
         * PA28 (A相) 作为 TIMG7 CCP0 外设功能, GPIOA->DIN31_0 中
         * 对应位可能不反映真实电平, 故不检查 bit28。
         *
         * 方向判定: bit29 (PA29 = B 相) 在 A 相上升沿时刻被 DMA
         * 同步锁存, 可直接用来判断方向。
         *   B=0 (LOW)  → 正转 +1
         *   B=1 (HIGH) → 反转 -1
         */
        if (snapshot & ENC2_BIT_B)
        {
            pulseCount--;   /* B=HIGH → 反转 */
        }
        else
        {
            pulseCount++;   /* B=LOW  → 正转 */
        }

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
     * DMA 循环模式: 硬件自动回绕到 g_enc2Buffer[0], 无需 CPU 干预。
     *
     * 如果没有启用循环模式 (SysConfig DMA extendedMode = Circular),
     * 则 NORMAL 模式下 DMA 传完 256 次后停止。需在 SysConfig 中修改。
     */

    *deltaPulses = pulseCount;
}
