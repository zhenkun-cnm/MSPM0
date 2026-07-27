/**
 * @file    app_vehicle_config.h
 * @brief   Shared physical calibration for the current car chassis.
 */
#ifndef APP_VEHICLE_CONFIG_H
#define APP_VEHICLE_CONFIG_H

#define VEHICLE_PI_F                         3.14159265358979323846f

/* Driven-wheel geometry (metres). */
#define VEHICLE_WHEEL_DIAMETER_M             0.056f
#define VEHICLE_WHEEL_TRACK_M                0.2142f
#define VEHICLE_WHEEL_CIRCUMFERENCE_M        (VEHICLE_PI_F * VEHICLE_WHEEL_DIAMETER_M)

/* Integer circumference for the UART speed diagnostic (micrometres). */
#define VEHICLE_WHEEL_CIRCUMFERENCE_UM       175929L

/* 13-line Hall encoder, 4x quadrature decode, 1:28 gearbox. */
#define VEHICLE_ENCODER_LINES_PER_MOTOR_REV  13L
#define VEHICLE_ENCODER_QUADRATURE_MULTIPLIER 4L
#define VEHICLE_GEAR_RATIO                   28L
#define VEHICLE_COUNTS_PER_OUTPUT_REV_NOMINAL \
    (VEHICLE_ENCODER_LINES_PER_MOTOR_REV * VEHICLE_ENCODER_QUADRATURE_MULTIPLIER * \
     VEHICLE_GEAR_RATIO)

/* Initial theoretical values; replace with measured left/right values after calibration. */
#define VEHICLE_LEFT_COUNTS_PER_OUTPUT_REV_CAL   1456L
#define VEHICLE_RIGHT_COUNTS_PER_OUTPUT_REV_CAL  1456L

#endif /* APP_VEHICLE_CONFIG_H */
