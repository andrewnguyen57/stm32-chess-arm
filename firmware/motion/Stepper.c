/** 
* @file Stepper.c
* @brief Stepper Motor Controller Library
*
* Author: Andrew Nguyen
* Date: July 2026
* Require: TMC5160 Driver Library
*/

#include "TMC5160.h"
#include "Stepper.h"

#include <stdio.h>
#include <math.h>

static Stepper_Status_TypeDef Stepper_Recalc(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->step_angle <= 0.0f) {
        hs->steps_per_deg = 0.0f;
        return STEPPER_BADARG;
    }
    if (hs->htmc.microstep == 0) {return STEPPER_ERR;}

    // Calculate steps per degree = microsteps / step angle
    hs->steps_per_deg = (float)hs->htmc.microstep / hs->step_angle;
    return STEPPER_OK;
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
Stepper_Status_TypeDef Stepper_Init(Stepper_TypeDef *hs, const Stepper_Config_TypeDef *cfg)
{
    if (hs == NULL || cfg == NULL) {return STEPPER_BADARG;}
    if (!isfinite(cfg->step_angle) || cfg->step_angle <= 0.0f) {return STEPPER_BADARG;}

    TMC5160_Config_TypeDef tmc_cfg = {
        .hspi = cfg->hspi,
        .cs = cfg->cs,
        .en = cfg->en,
        .r_sense = cfg->r_sense
    };

    hs->step_angle = cfg->step_angle;

    // TMC init must precede Recalc: Recalc reads htmc.microstep, populated by TMC5160_Init
    if (TMC5160_Init(&hs->htmc, &tmc_cfg) != TMC5160_OK) {return STEPPER_ERR;}
    if (TMC5160_SetCurrent(&hs->htmc, cfg->current_ma) != TMC5160_OK) {return STEPPER_ERR;}
    if (TMC5160_SetMicrostep(&hs->htmc, cfg->microstep) != TMC5160_OK) {return STEPPER_BADARG;}

    return Stepper_Recalc(hs);
}

Stepper_Status_TypeDef Stepper_Enable(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_BADARG;}

    if (TMC5160_SetEN(&hs->htmc, GPIO_PIN_RESET) != TMC5160_OK) {return STEPPER_ERR;}
    return STEPPER_OK;
}

Stepper_Status_TypeDef Stepper_Disable(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_BADARG;}

    if (TMC5160_SetEN(&hs->htmc, GPIO_PIN_SET) != TMC5160_OK) {return STEPPER_ERR;}
    return STEPPER_OK;
}

Stepper_Status_TypeDef Stepper_SetAngle(Stepper_TypeDef *hs, float step_angle)
{
    if (hs == NULL) {return STEPPER_BADARG;}

    if (!isfinite(step_angle) || step_angle <= 0.0f) {return STEPPER_BADARG;}

    hs->step_angle = step_angle;
    return Stepper_Recalc(hs);
}

Stepper_Status_TypeDef Stepper_SetCurrent(Stepper_TypeDef *hs, uint16_t current_ma)
{
    if (hs == NULL) {return STEPPER_BADARG;}

    if (TMC5160_SetCurrent(&hs->htmc, current_ma) != TMC5160_OK) {return STEPPER_ERR;}
    
    return STEPPER_OK;
}

Stepper_Status_TypeDef Stepper_SetMicrostep(Stepper_TypeDef *hs, uint16_t microstep)
{
    if (hs == NULL) {return STEPPER_BADARG;}

    if (TMC5160_SetMicrostep(&hs->htmc, microstep) != TMC5160_OK) {return STEPPER_ERR;}

    return Stepper_Recalc(hs);
}

Stepper_Status_TypeDef Stepper_SetMode(Stepper_TypeDef *hs, Stepper_Mode_TypeDef mode)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (mode < STEPPER_MODE_POSITION || mode > STEPPER_MODE_HOLD) {return STEPPER_BADARG;}

    TMC5160_Status_TypeDef status = TMC5160_SetRampMode(&hs->htmc, (TMC5160_RampMode_TypeDef)mode);

    return (status == TMC5160_OK) ? STEPPER_OK : STEPPER_ERR;
}

// -----------------------------------------------------------------------------
// VELOCITY and ACCELERATION
// -----------------------------------------------------------------------------
Stepper_Status_TypeDef Stepper_SetVelocity(Stepper_TypeDef *hs, float deg_per_sec)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(deg_per_sec) || deg_per_sec <= 0) {return STEPPER_BADARG;}

    float steps_per_sec = deg_per_sec * hs->steps_per_deg;

    // time reference t = 2^24 / fCLK (pg 42)
    // VMAX = a[usteps/s] * 2^24 / fCLK
    float vmax_f = steps_per_sec * (STEPPER_VEL_TIME_BASE/STEPPER_FCLK);

    vmax_f += 0.5f;

    uint32_t vmax = (uint32_t)vmax_f;

    TMC5160_Status_TypeDef status = TMC5160_SetVelocity(&hs->htmc, vmax);

    switch(status) {
        case TMC5160_OK: return STEPPER_OK;
        case TMC5160_CLAMPED: return STEPPER_ERR;
        default: return STEPPER_ERR;
    } 
}

Stepper_Status_TypeDef Stepper_SetAcceleration(Stepper_TypeDef *hs, float deg_per_sec2)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(deg_per_sec2) || deg_per_sec2 <= 0) {return STEPPER_BADARG;}

    float steps_per_sec2 = deg_per_sec2 * hs->steps_per_deg;

    // time reference t = 2^41 / fCLK^2 (pg 42)
    // AMAX = a[usteps/s^2] * 2^41 / fCLK^2
    float amax_f = steps_per_sec2 * (STEPPER_ACCEL_TIME_BASE/(STEPPER_FCLK * STEPPER_FCLK));

    amax_f += 0.5f;

    uint32_t amax = (uint32_t)amax_f;

    TMC5160_Status_TypeDef status = TMC5160_SetAcceleration(&hs->htmc, amax);

    switch(status) {
        case TMC5160_OK: return STEPPER_OK;
        case TMC5160_CLAMPED: return STEPPER_ERR;
        default: return STEPPER_ERR;
    } 
}
Stepper_Status_TypeDef Stepper_SetVelocityPercentage(Stepper_TypeDef *hs, float percent)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(percent) || percent < 0.0f || percent > 100.0f) {return STEPPER_BADARG;}

    // 0-100%
    // Convert percentage value to actual value
    float vmax_f = percent / 100.0f * (float)TMC5160_VMAX_MAX;
    vmax_f += 0.5f;
    uint32_t vmax = (uint32_t)vmax_f;
    
    TMC5160_Status_TypeDef status = TMC5160_SetVelocity(&hs->htmc, vmax);

    switch(status) {
        case TMC5160_OK: return STEPPER_OK;
        case TMC5160_CLAMPED: return STEPPER_ERR;
        default: return STEPPER_ERR;
    }
}

Stepper_Status_TypeDef Stepper_SetAccelerationPercentage(Stepper_TypeDef *hs, float percent)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(percent) || percent < 0.0f || percent > 100.0f) {return STEPPER_BADARG;}

    // 0-100%
    // Convert percentage value to actual value
    float amax_f = percent / 100.0f * (float)TMC5160_AMAX_MAX;
    amax_f += 0.5f;
    uint32_t amax = (uint32_t)amax_f;

    TMC5160_Status_TypeDef status = TMC5160_SetAcceleration(&hs->htmc, amax);

    switch(status) {
        case TMC5160_OK: return STEPPER_OK;
        case TMC5160_CLAMPED: return STEPPER_ERR;
        default: return STEPPER_ERR;
    }
}

// -----------------------------------------------------------------------------
// MOTION
// -----------------------------------------------------------------------------
Stepper_Status_TypeDef Stepper_MoveTo(Stepper_TypeDef *hs, float deg)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(deg)) {return STEPPER_BADARG;}

    // Convert degree to steps
    float steps_f = deg * hs->steps_per_deg;
    if (steps_f >= STEPPER_STEPS_LIMIT || steps_f <= -STEPPER_STEPS_LIMIT) {return STEPPER_BADARG;}

    // Round to nearest to avoid truncation
    if (steps_f >= 0.0f) {steps_f += 0.5f;}
    else {steps_f -= 0.5f;}

    int32_t steps = (int32_t)steps_f;

    if(TMC5160_MoveTo(&hs->htmc, steps) != TMC5160_OK) {return STEPPER_ERR;}

    return STEPPER_OK;
}

Stepper_Status_TypeDef Stepper_MoveBy(Stepper_TypeDef *hs, float deg)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(deg)) {return STEPPER_BADARG;}

    float cur_pos = Stepper_GetPosition(hs);

    return Stepper_MoveTo(hs, cur_pos + deg); 
}

Stepper_Status_TypeDef Stepper_Stop(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}

    TMC5160_Status_TypeDef status = TMC5160_Stop(&hs->htmc);

    switch (status) {
        case TMC5160_OK: return STEPPER_OK;
        case TMC5160_BADARG: return STEPPER_BADARG;
        default: return STEPPER_ERR;
    }
}

Stepper_Status_TypeDef Stepper_SetPosition(Stepper_TypeDef *hs, float deg)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    if (hs->steps_per_deg <= 0.0f) {return STEPPER_ERR;}
    if (!isfinite(deg)) {return STEPPER_BADARG;}
    // Convert position from degrees to steps
    float steps_f = hs->steps_per_deg * deg;

    if (steps_f >= 0.0f) {steps_f += 0.5f;}
    else {steps_f -= 0.5f;}

    if (steps_f >= STEPPER_STEPS_LIMIT || steps_f <= -STEPPER_STEPS_LIMIT) {return STEPPER_BADARG;}

    int32_t steps = (int32_t)steps_f;

    if (TMC5160_SetPosition(&hs->htmc, steps) != TMC5160_OK) {return STEPPER_ERR;}
    
    return STEPPER_OK;
}

Stepper_Status_TypeDef Stepper_WaitStopped(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_BADARG;}
    uint32_t t = HAL_GetTick();
    while (Stepper_IsMoving(hs)) {
        if (HAL_GetTick() - t >= STEPPER_TIMEOUT_MS) {return STEPPER_TIMEOUT;}
        HAL_Delay(STEPPER_POLL_MS);
    }
    return STEPPER_OK;
}

// -----------------------------------------------------------------------------
// STATUS and READBACK
// -----------------------------------------------------------------------------
float Stepper_GetPosition(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return 0.0f;}
    if (hs->steps_per_deg <= 0.0f) {return 0.0f;}

    int32_t step_position = TMC5160_GetPosition(&hs->htmc); 

    // Convert step to degree
    return ((float)step_position / hs->steps_per_deg); 
}

float Stepper_GetVelocity(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return 0.0f;}
    if (hs->steps_per_deg <= 0.0f) {return 0.0f;}

    uint32_t vmax = TMC5160_GetMaxVelocity(&hs->htmc);

    // Inverse of SetVelocity: v[deg/s] = VMAX * fCLK / 2^24 / steps_per_deg
    return (float)vmax * (STEPPER_FCLK / STEPPER_VEL_TIME_BASE) / hs->steps_per_deg;
}

float Stepper_GetAcceleration(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return 0.0f;}
    if (hs->steps_per_deg <= 0.0f) {return 0.0f;}

    uint32_t amax = TMC5160_GetMaxAcceleration(&hs->htmc);

    // Inverse of SetAcceleration: a[deg/s^2] = AMAX * fCLK^2 / 2^41 / steps_per_deg
    return (float)amax * ((STEPPER_FCLK * STEPPER_FCLK) / STEPPER_ACCEL_TIME_BASE) / hs->steps_per_deg;
}

Stepper_Mode_TypeDef Stepper_GetMode(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return STEPPER_MODE_POSITION;}

    TMC5160_RampMode_TypeDef mode = TMC5160_GetRampMode(&hs->htmc);

    switch (mode) {
    case TMC5160_RAMPMODE_POSITION:
    case TMC5160_RAMPMODE_VEL_POS:
    case TMC5160_RAMPMODE_VEL_NEG:
    case TMC5160_RAMPMODE_HOLD:
        return (Stepper_Mode_TypeDef)mode;
        
    default:
        return STEPPER_MODE_POSITION;
    }
}

bool Stepper_IsMoving(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return false;}

    TMC5160_RampStat_TypeDef rampstat = TMC5160_GetRampStat(&hs->htmc);

    return !rampstat.vzero;
}

bool Stepper_IsStalled(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return false;}

    TMC5160_DrvStat_TypeDef drvstat = TMC5160_GetDrvStat(&hs->htmc);

    return drvstat.stallguard;
}

bool Stepper_PositionReached(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return false;}

    TMC5160_RampStat_TypeDef rampstat = TMC5160_GetRampStat(&hs->htmc);

    return rampstat.position_reached;
}

bool Stepper_VelocityReached(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return false;}

    TMC5160_RampStat_TypeDef rampstat = TMC5160_GetRampStat(&hs->htmc);

    return rampstat.velocity_reached;
}

// -----------------------------------------------------------------------------
// DIAGNOSTICS
// -----------------------------------------------------------------------------
Stepper_Diag_TypeDef Stepper_GetDiag(Stepper_TypeDef *hs)
{
    if (hs == NULL) {return (Stepper_Diag_TypeDef){0};}
    Stepper_Diag_TypeDef diag;

    diag.ioin = TMC5160_GetIOIN(&hs->htmc);
    diag.drv_stat = TMC5160_GetDrvStat(&hs->htmc);
    diag.g_stat = TMC5160_GetGStat(&hs->htmc);
    diag.ramp_stat = TMC5160_GetRampStat(&hs->htmc);

    return diag;
}

Stepper_Status_TypeDef Stepper_PrintDiag(Stepper_TypeDef *hs, UART_HandleTypeDef *huart)
{
    if (hs == NULL || huart == NULL) {return STEPPER_BADARG;}

    Stepper_Diag_TypeDef diag = Stepper_GetDiag(hs);

    char buf[64];
    int len;

    // IC version expects 0x30
    len = snprintf(buf, sizeof(buf), "--- Stepper Diag ---\r\nIC version: 0x%02X\r\n", diag.ioin.version);
    HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);

    // Print a line for each flag that is set
    #define DIAG_FLAG(field, label)                                          \
        do {                                                                 \
            if (field) {                                                     \
                len = snprintf(buf, sizeof(buf), "  %s\r\n", label);         \
                HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)len,      \
                                  HAL_MAX_DELAY);                            \
            }                                                                \
        } while (0)

    // GSTAT (pg 33)
    DIAG_FLAG(diag.g_stat.reset,   "GSTAT: IC was reset");
    DIAG_FLAG(diag.g_stat.drv_err, "GSTAT: driver error shutdown");
    DIAG_FLAG(diag.g_stat.uv_cp,   "GSTAT: charge pump undervoltage");

    // DRV_STATUS (pg 56)
    DIAG_FLAG(diag.drv_stat.stst,       "DRV: standstill");
    DIAG_FLAG(diag.drv_stat.olb,        "DRV: open load B");
    DIAG_FLAG(diag.drv_stat.ola,        "DRV: open load A");
    DIAG_FLAG(diag.drv_stat.s2gb,       "DRV: short to GND B");
    DIAG_FLAG(diag.drv_stat.s2ga,       "DRV: short to GND A");
    DIAG_FLAG(diag.drv_stat.otpw,       "DRV: overtemp prewarning");
    DIAG_FLAG(diag.drv_stat.ot,         "DRV: OVERTEMP");
    DIAG_FLAG(diag.drv_stat.stallguard, "DRV: StallGuard");

    #undef DIAG_FLAG
    return STEPPER_OK;
}