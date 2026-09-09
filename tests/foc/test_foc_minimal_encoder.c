#include <stdio.h>

#include "foc_encoder.h"

/* 精确可表示的周期：1/2048 s，Q15 定点为 16，速度断言无量化误差 */
#define TEST_PERIOD_S        (1.0f / 2048.0f)
#define TEST_POLE_PAIRS      1U
#define TEST_TIMEOUT         200U

static foc_encoder_t encoder_ready(foc_encoder_params_t *ptParams)
{
    foc_encoder_t tEncoder = {0};

    foc_encoder_DefaultParams(ptParams);
    ptParams->hwInvalidTimeout = TEST_TIMEOUT;
    (void)foc_encoder_Init(&tEncoder, ptParams, TEST_POLE_PAIRS,
                           FOC_SCALAR(TEST_PERIOD_S));
    return tEncoder;
}

static int test_null_arguments(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    if (foc_encoder_Step(NULL, &tSample, &tOutput) != FOC_RESULT_NULL) {
        return 1;
    }
    if (foc_encoder_Step(&tEncoder, NULL, &tOutput) != FOC_RESULT_NULL) {
        return 1;
    }
    return foc_encoder_Step(&tEncoder, &tSample, NULL) != FOC_RESULT_NULL;
}

static int test_invalid_init_params(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = {0};

    foc_encoder_DefaultParams(&tParams);
    if (foc_encoder_Init(&tEncoder, &tParams, 0U,
                         FOC_SCALAR(TEST_PERIOD_S)) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    foc_encoder_DefaultParams(&tParams);
    return foc_encoder_Init(&tEncoder, &tParams, TEST_POLE_PAIRS,
                            FOC_ZERO) !=
           FOC_RESULT_INVALID_ARGUMENT;
}

/* 首样本：角度=原始码/4096 圈，速度=0 */
static int test_first_sample(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.hwRawAngle = 2048U;
    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    if (foc_encoder_Step(&tEncoder, &tSample, &tOutput) != FOC_RESULT_OK) {
        return 1;
    }
    if (!tEncoder.bInitialized || !tEncoder.bValid) {
        return 1;
    }
    if (foc_angle_to_turns(tOutput.tMechanicalAngle) != 0.5f) {
        return 1;
    }
    return tOutput.qMechanicalSpeed != FOC_ZERO;
}

/* 正向：角度码递增 → 速度 > 0 */
static int test_forward_speed(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    tSample.wSequence = 2U;
    tSample.hwRawAngle = 1024U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    if (tOutput.qMechanicalSpeed <= FOC_ZERO) {
        return 1;
    }
    /* 1024 counts / 1 tick @1/2048s = 512 turn/s，低通 alpha=0.25 首拍为 128 */
    {
        foc_scalar_t qExpected = FOC_SCALAR(512.0f * 0.25f);
        foc_scalar_t qDiff = foc_abs(
            foc_sub_sat(tOutput.qMechanicalSpeed, qExpected));
        if (qDiff > FOC_SCALAR(0.5f)) {
            return 1;
        }
    }
    return 0;
}

/* 反向：角度码递减 → 速度 < 0 */
static int test_reverse_speed(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 2048U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    tSample.wSequence = 2U;
    tSample.hwRawAngle = 1024U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    if (tOutput.qMechanicalSpeed >= FOC_ZERO) {
        return 1;
    }
    {
        foc_scalar_t qExpected = FOC_SCALAR(-512.0f * 0.25f);
        foc_scalar_t qDiff = foc_abs(
            foc_sub_sat(tOutput.qMechanicalSpeed, qExpected));
        if (qDiff > FOC_SCALAR(0.5f)) {
            return 1;
        }
    }
    return 0;
}

/* 4095 → 0 回卷：视为 +1 count 正向 */
static int test_wrap_forward(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 4095U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    tSample.wSequence = 2U;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    if (tOutput.qMechanicalSpeed <= FOC_ZERO) {
        return 1;
    }
    /* 1 count / 1 tick @1/2048s = 0.5 turn/s，低通首拍 0.125 */
    {
        foc_scalar_t qExpected = FOC_SCALAR(0.5f * 0.25f);
        foc_scalar_t qDiff = foc_abs(
            foc_sub_sat(tOutput.qMechanicalSpeed, qExpected));
        if (qDiff > FOC_SCALAR(0.01f)) {
            return 1;
        }
    }
    return 0;
}

/* 丢样：序号跳变 > 1 时重设角度基准并清零速度，避免启动速度尖峰 */
static int test_sequence_gap_resets_speed(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    tSample.wSequence = 3U;     /* 跳过 2：视为采样间隙 */
    tSample.hwRawAngle = 256U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    if (tOutput.qMechanicalSpeed != FOC_ZERO) {
        return 1;
    }
    return foc_angle_to_turns(tOutput.tMechanicalAngle) !=
           (256.0f / 4096.0f);
}

/* 磁铁异常：立即置无效并返回错误 */
static int test_magnet_failure(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    tSample.wSequence = 2U;
    tSample.bMagnetOk = false;
    if (foc_encoder_Step(&tEncoder, &tSample, &tOutput) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    return tEncoder.bValid ? 1 : 0;
}

/* 样本超时：距上次样本超过阈值后置无效 */
static int test_sample_timeout(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};
    uint16_t hwIndex;

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    for (hwIndex = 0U; hwIndex < TEST_TIMEOUT + 2U; hwIndex++) {
        (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    }
    return tEncoder.bValid ? 1 : 0;
}

/* 低速禁止外推：0.5 e-turn/s < 1 → 无新样本时角度不变 */
static int test_low_speed_no_extrapolation(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tBase = {0};
    foc_encoder_output_t tNext = {0};
    uint16_t hwIndex;

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);

    tSample.wSequence = 2U;
    tSample.hwRawAngle = 1U;    /* 0.5 turn/s @1/2048s */
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);
    for (hwIndex = 0U; hwIndex < 8U; hwIndex++) {
        (void)foc_encoder_Step(&tEncoder, &tSample, &tNext);
        if (tNext.tMechanicalAngle.wBam32 !=
            tBase.tMechanicalAngle.wBam32) {
            return 1;
        }
    }
    return 0;
}

/* 高速外推：新样本后无新样本，角度按 qSpeed × ticks × period 前移。
   期望值直接用滤波后的实际速度计算，float/fixed 后端均精确一致 */
static int test_high_speed_extrapolation(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tBase = {0};
    foc_encoder_output_t tNext = {0};
    uint32_t wExpectedDelta;
    uint32_t wActualDelta;
    uint16_t hwIndex;

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);

    /* 100 counts/tick @1/2048s → 原始 50 turn/s，低通首拍 12.5 */
    tSample.wSequence = 2U;
    tSample.hwRawAngle = 100U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);
    for (hwIndex = 0U; hwIndex < 16U; hwIndex++) {
        (void)foc_encoder_Step(&tEncoder, &tSample, &tNext);
    }
    {
        float fSpeed = foc_to_float(tEncoder.qMechanicalSpeed);
        wExpectedDelta = (uint32_t)(
            fSpeed * 16.0f * (1.0f / 2048.0f) * 4294967296.0f);
    }
    wActualDelta = tNext.tMechanicalAngle.wBam32 -
                   tBase.tMechanicalAngle.wBam32;
    if (wActualDelta != wExpectedDelta) {
        return 1;
    }
    /* 反向：负速度 → 角度回卷减少 */
    tSample.wSequence = 3U;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);
    tSample.wSequence = 4U;
    tSample.hwRawAngle = 3996U;     /* -100 counts */
    (void)foc_encoder_Step(&tEncoder, &tSample, &tBase);
    for (hwIndex = 0U; hwIndex < 16U; hwIndex++) {
        (void)foc_encoder_Step(&tEncoder, &tSample, &tNext);
    }
    {
        int32_t nDelta = (int32_t)(tNext.tMechanicalAngle.wBam32 -
                                   tBase.tMechanicalAngle.wBam32);
        if (nDelta >= 0) {
            return 1;
        }
    }
    return 0;
}

/* 输出单位：机械 turn/s（256 counts/tick @1/2048s → 128 turn/s 稳态） */
static int test_output_unit_turn_per_second(void)
{
    foc_encoder_params_t tParams;
    foc_encoder_t tEncoder = encoder_ready(&tParams);
    foc_encoder_sample_t tSample = {0};
    foc_encoder_output_t tOutput = {0};
    uint16_t hwIndex;
    foc_scalar_t qExpected = FOC_SCALAR(128.0f);
    foc_scalar_t qDiff;

    tSample.wSequence = 1U;
    tSample.bMagnetOk = true;
    tSample.hwRawAngle = 0U;
    (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);

    /* 每 tick +256 counts，连续 24 拍逼近稳态 128 turn/s
       （低通 alpha=0.25：0.75^24 × 128 ≈ 0.13 < 1 容差） */
    for (hwIndex = 0U; hwIndex < 24U; hwIndex++) {
        tSample.wSequence = (uint32_t)hwIndex + 2U;
        tSample.hwRawAngle = (uint16_t)((hwIndex + 1U) * 256U);
        (void)foc_encoder_Step(&tEncoder, &tSample, &tOutput);
    }
    qDiff = foc_abs(foc_sub_sat(tOutput.qMechanicalSpeed, qExpected));
    if (qDiff > FOC_SCALAR(1.0f)) {
        return 1;
    }
    /* 角度应连续：24 拍 × 256 counts = 6144 counts = 1.5 圈 → 0.5 圈 */
    return foc_angle_to_turns(tOutput.tMechanicalAngle) != 0.5f;
}

int main(void)
{
    int nFailures = 0;

    nFailures += test_null_arguments();
    nFailures += test_invalid_init_params();
    nFailures += test_first_sample();
    nFailures += test_forward_speed();
    nFailures += test_reverse_speed();
    nFailures += test_wrap_forward();
    nFailures += test_sequence_gap_resets_speed();
    nFailures += test_magnet_failure();
    nFailures += test_sample_timeout();
    nFailures += test_low_speed_no_extrapolation();
    nFailures += test_high_speed_extrapolation();
    nFailures += test_output_unit_turn_per_second();
    printf("minimal encoder: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
