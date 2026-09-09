/****************************************************************************
 * @file    foc_core.c
 * @brief   Architecture-independent Clarke and Park transforms
 ************************************************************************** */

#include "foc_core.h"
#include "foc_modulation.h"

#include <stddef.h>

static bool core_mode_is_valid(foc_control_mode_e eMode)
{
    return eMode < FOC_MODE_MAX;
}

static foc_result_t core_validate_inputs(const foc_core_state_t *ptState,
                                         const foc_core_command_t *ptCommand,
                                         const foc_core_input_t *ptInput)
{
    if (ptState == NULL || ptCommand == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCommand->eMode == FOC_MODE_POSITION) {
        return FOC_RESULT_DISABLED;
    }
    if (!ptInput->bAngleValid) {
        return FOC_RESULT_SAFETY;
    }
    return core_mode_is_valid(ptCommand->eMode) ? FOC_RESULT_OK :
                                                  FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t core_update_current(foc_core_state_t *ptState,
                                        const foc_core_command_t *ptCommand)
{
    if (ptCommand->eMode == FOC_MODE_VOLTAGE) {
        ptState->tVoltage = ptCommand->tVoltageReference;
        return FOC_RESULT_OK;
    }
    ptState->tVoltage.qD = foc_pid_Step(&ptState->tIdPi,
                                        ptCommand->tCurrentReference.qD,
                                        ptState->tCurrent.qD);
    ptState->tVoltage.qQ = foc_pid_Step(&ptState->tIqPi,
                                        ptCommand->tCurrentReference.qQ,
                                        ptState->tCurrent.qQ);
    return FOC_RESULT_OK;
}

void foc_core_Reset(foc_core_state_t *ptState)
{
    if (ptState == NULL) {
        return;
    }
    foc_pid_Reset(&ptState->tIdPi);
    foc_pid_Reset(&ptState->tIqPi);
    ptState->tCurrentAlphaBeta = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    ptState->tCurrent = (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptState->tVoltage = (foc_dq_t){FOC_ZERO, FOC_ZERO};
    ptState->tVoltageAlphaBeta = (foc_ab_t){FOC_ZERO, FOC_ZERO};
    ptState->tDuty = (foc_duty_abc_t){FOC_HALF, FOC_HALF, FOC_HALF};
}

foc_result_t foc_core_step(foc_core_state_t *ptState,
                           const foc_core_command_t *ptCommand,
                           const foc_core_input_t *ptInput)
{
    foc_scalar_t qSin = FOC_ZERO;
    foc_scalar_t qCos = FOC_ZERO;
    foc_result_t eResult;

    eResult = core_validate_inputs(ptState, ptCommand, ptInput);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_clarke(ptInput->qIu, ptInput->qIv, ptInput->qIw,
                         &ptState->tCurrentAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    foc_angle_sincos(ptInput->tElectricalAngle, &qSin, &qCos);
    eResult = foc_park_cached(&ptState->tCurrentAlphaBeta, qSin, qCos,
                              &ptState->tCurrent);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = core_update_current(ptState, ptCommand);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    eResult = foc_ipark_cached(&ptState->tVoltage, qSin, qCos,
                               &ptState->tVoltageAlphaBeta);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    return foc_svpwm(&ptState->tVoltageAlphaBeta, &ptState->tDuty);
}

foc_result_t foc_clarke(foc_scalar_t qIu,
                        foc_scalar_t qIv,
                        foc_scalar_t qIw,
                        foc_ab_t *ptAB)
{
    const foc_scalar_t qInvSqrtThree = FOC_SCALAR(0.5773502692f);

    if (ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = qIu;
    ptAB->qBeta = foc_sub_sat(foc_mul_pu(qIv, qInvSqrtThree),
                              foc_mul_pu(qIw, qInvSqrtThree));
    return FOC_RESULT_OK;
}

foc_result_t foc_park_cached(const foc_ab_t *ptAB,
                             foc_scalar_t qSin,
                             foc_scalar_t qCos,
                             foc_dq_t *ptDQ)
{
    if (ptAB == NULL || ptDQ == NULL) {
        return FOC_RESULT_NULL;
    }
    ptDQ->qD = foc_add_sat(foc_mul_pu(ptAB->qAlpha, qCos),
                           foc_mul_pu(ptAB->qBeta, qSin));
    ptDQ->qQ = foc_sub_sat(foc_mul_pu(ptAB->qBeta, qCos),
                           foc_mul_pu(ptAB->qAlpha, qSin));
    return FOC_RESULT_OK;
}

foc_result_t foc_park(const foc_ab_t *ptAB,
                      foc_angle_t tTheta,
                      foc_dq_t *ptDQ)
{
    return foc_park_cached(ptAB, foc_angle_sin(tTheta),
                           foc_angle_cos(tTheta), ptDQ);
}

foc_result_t foc_ipark_cached(const foc_dq_t *ptDQ,
                              foc_scalar_t qSin,
                              foc_scalar_t qCos,
                              foc_ab_t *ptAB)
{
    if (ptDQ == NULL || ptAB == NULL) {
        return FOC_RESULT_NULL;
    }
    ptAB->qAlpha = foc_sub_sat(foc_mul_pu(ptDQ->qD, qCos),
                               foc_mul_pu(ptDQ->qQ, qSin));
    ptAB->qBeta = foc_add_sat(foc_mul_pu(ptDQ->qD, qSin),
                              foc_mul_pu(ptDQ->qQ, qCos));
    return FOC_RESULT_OK;
}

foc_result_t foc_ipark(const foc_dq_t *ptDQ,
                       foc_angle_t tTheta,
                       foc_ab_t *ptAB)
{
    return foc_ipark_cached(ptDQ, foc_angle_sin(tTheta),
                            foc_angle_cos(tTheta), ptAB);
}
