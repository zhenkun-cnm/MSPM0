/**
 * @file    imu_ahrs9.c
 * @brief   Lightweight 9-axis Mahony AHRS attitude solver.
 */
#include "imu_ahrs9.h"
#include <math.h>

#define AHRS9_PI_F 3.14159265358979323846f

static float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
static float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;
static Ahrs9Config cfg_ = {1.6f, 0.08f, 0.02f};

static float inv_sqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

static float wrap_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void normalize_quat(void)
{
    float n = inv_sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (n <= 0.0f) {
        q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
        return;
    }
    q0_ *= n; q1_ *= n; q2_ *= n; q3_ *= n;
}

void ahrs9_init(const Ahrs9Config *cfg)
{
    if (cfg) {
        cfg_ = *cfg;
    }
    q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
    ix_ = 0.0f; iy_ = 0.0f; iz_ = 0.0f;
}

void ahrs9_update(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz,
                  float dt)
{
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float gxRad, gyRad, gzRad;
    float accNorm;
    float magNorm;
    bool useAcc;
    bool useMag;

    if (dt <= 0.0f) return;

    accNorm = inv_sqrt(ax * ax + ay * ay + az * az);
    useAcc = (accNorm > 0.0f);
    if (useAcc) {
        float vx, vy, vz;

        ax *= accNorm; ay *= accNorm; az *= accNorm;

        vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
        vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
        vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

        ex += cfg_.kpAcc * (ay * vz - az * vy);
        ey += cfg_.kpAcc * (az * vx - ax * vz);
        ez += cfg_.kpAcc * (ax * vy - ay * vx);
    }

    magNorm = inv_sqrt(mx * mx + my * my + mz * mz);
    useMag = (magNorm > 0.0f);
    if (useMag) {
        float hx, hy, bx, bz;
        float wx, wy, wz;

        mx *= magNorm; my *= magNorm; mz *= magNorm;

        hx = 2.0f * mx * (0.5f - q2_ * q2_ - q3_ * q3_) +
             2.0f * my * (q1_ * q2_ - q0_ * q3_) +
             2.0f * mz * (q1_ * q3_ + q0_ * q2_);
        hy = 2.0f * mx * (q1_ * q2_ + q0_ * q3_) +
             2.0f * my * (0.5f - q1_ * q1_ - q3_ * q3_) +
             2.0f * mz * (q2_ * q3_ - q0_ * q1_);
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * mx * (q1_ * q3_ - q0_ * q2_) +
             2.0f * my * (q2_ * q3_ + q0_ * q1_) +
             2.0f * mz * (0.5f - q1_ * q1_ - q2_ * q2_);

        wx = 2.0f * bx * (0.5f - q2_ * q2_ - q3_ * q3_) +
             2.0f * bz * (q1_ * q3_ - q0_ * q2_);
        wy = 2.0f * bx * (q1_ * q2_ - q0_ * q3_) +
             2.0f * bz * (q0_ * q1_ + q2_ * q3_);
        wz = 2.0f * bx * (q0_ * q2_ + q1_ * q3_) +
             2.0f * bz * (0.5f - q1_ * q1_ - q2_ * q2_);

        ex += cfg_.kpMag * (my * wz - mz * wy);
        ey += cfg_.kpMag * (mz * wx - mx * wz);
        ez += cfg_.kpMag * (mx * wy - my * wx);
    }

    gxRad = gx * (AHRS9_PI_F / 180.0f);
    gyRad = gy * (AHRS9_PI_F / 180.0f);
    gzRad = gz * (AHRS9_PI_F / 180.0f);

    if (cfg_.ki > 0.0f && (useAcc || useMag)) {
        ix_ += cfg_.ki * ex * dt;
        iy_ += cfg_.ki * ey * dt;
        iz_ += cfg_.ki * ez * dt;
        gxRad += ix_; gyRad += iy_; gzRad += iz_;
    }

    gxRad += ex;
    gyRad += ey;
    gzRad += ez;

    {
        float qa = q0_, qb = q1_, qc = q2_;
        float halfDt = 0.5f * dt;

        q0_ += (-qb * gxRad - qc * gyRad - q3_ * gzRad) * halfDt;
        q1_ += ( qa * gxRad + qc * gzRad - q3_ * gyRad) * halfDt;
        q2_ += ( qa * gyRad - qb * gzRad + q3_ * gxRad) * halfDt;
        q3_ += ( qa * gzRad + qb * gyRad - qc * gxRad) * halfDt;
        normalize_quat();
    }
}

void ahrs9_get_euler(float *roll, float *pitch, float *yaw)
{
    float sinr = 2.0f * (q0_ * q1_ + q2_ * q3_);
    float cosr = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
    float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    float siny = 2.0f * (q0_ * q3_ + q1_ * q2_);
    float cosy = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);

    if (roll) *roll = atan2f(sinr, cosr) * (180.0f / AHRS9_PI_F);
    if (pitch) {
        if (fabsf(sinp) >= 1.0f) {
            *pitch = (sinp > 0.0f) ? 90.0f : -90.0f;
        } else {
            *pitch = asinf(sinp) * (180.0f / AHRS9_PI_F);
        }
    }
    if (yaw) *yaw = wrap_180(atan2f(siny, cosy) * (180.0f / AHRS9_PI_F));
}

void ahrs9_reset_yaw(void)
{
    float roll, pitch, yaw;
    float cy, sy;
    float y0, y3;
    float nq0, nq1, nq2, nq3;

    ahrs9_get_euler(&roll, &pitch, &yaw);
    cy = cosf(yaw * (AHRS9_PI_F / 360.0f));
    sy = sinf(yaw * (AHRS9_PI_F / 360.0f));
    y0 = cy;
    y3 = -sy;

    nq0 = y0 * q0_ - y3 * q3_;
    nq1 = y0 * q1_ + y3 * q2_;
    nq2 = y0 * q2_ - y3 * q1_;
    nq3 = y0 * q3_ + y3 * q0_;

    q0_ = nq0; q1_ = nq1; q2_ = nq2; q3_ = nq3;
    normalize_quat();
}
