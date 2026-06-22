/**
 * @file    dev_imu.h
 * @brief   IMU Device 层抽象接口
 * @note    四层解耦架构 - Device 层
 *          支持初始化、WHO_AM_I 验证、6 轴原始数据读取
 */

#ifndef DEV_IMU_H
#define DEV_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* 6 轴原始数据：加速度计 + 陀螺仪 */
typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} DevIMU_SensorData_t;

/* IMU 设备接口结构体（OOP 契约） */
typedef struct DevIMU {
    /**
     * @brief 初始化 IMU 硬件
     * @param self  指向自身
     */
    void (*init)(struct DevIMU *self);

    /**
     * @brief 读取 WHO_AM_I 寄存器
     * @param self  指向自身
     * @return uint8_t  寄存器值（预期 0xEA）
     */
    uint8_t (*whoAmI)(struct DevIMU *self);

    /**
     * @brief 读取 6 轴原始数据
     * @param self  指向自身
     * @param data  输出传感器数据指针
     * @return true 读取成功
     */
    bool (*readSensorData)(struct DevIMU *self, DevIMU_SensorData_t *data);
} DevIMU;

/**
 * @brief 获取全局 IMU 设备句柄
 * @return DevIMU* 指向 IMU 设备接口的指针
 */
DevIMU* GetIMU(void);

#endif /* DEV_IMU_H */