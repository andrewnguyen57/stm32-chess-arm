/**
 * @file    TMC5160.c
 * @brief   TMC5160 Stepper Driver Library
 *
 * Author: Andrew Nguyen
 * Date:   June 2026
 * 
 * References:
 * Trinamic TMC5160 Datasheet (see docs)
 * Relevant page numbers are cited throughout the source.
 */

#include "TMC5160.h"
#include "TMC5160_SPI.h"

// --- REGISTER ACCESS ---
void TMC5160_WriteRegister(TMC5160_TypeDef *htmc, uint8_t reg_address, uint32_t data) 
{
    if (htmc == NULL) {return;}
    if (reg_address >= 128) {return;}

    // Cache the register value
    // Because some registers are write-only
    htmc->reg_cache[reg_address] = data;

    TMC5160_SPI_Write(htmc, reg_address, data);
}

uint32_t TMC5160_ReadRegister(TMC5160_TypeDef *htmc, uint8_t reg_address)
{
    if (htmc == NULL) {return 0;}
    if (reg_address >= 128) {return 0;}

    return TMC5160_SPI_Read(htmc, reg_address);
}

TMC5160_Status_TypeDef TMC5160_Init(TMC5160_TypeDef *htmc, const TMC5160_Config_TypeDef *cfg)
{
    if (htmc == NULL || cfg == NULL || cfg->hspi == NULL) {return TMC5160_BADARG;}

    // From user config (fall back to default sense resistor if unset)
    htmc->hspi    = cfg->hspi;
    htmc->cs      = cfg->cs;
    htmc->en      = cfg->en;
    htmc->r_sense = (cfg->r_sense > 0.0f) ? cfg->r_sense : TMC5160_R_SENSE_DEFAULT;

    // Internal state mirrors of the register defaults written below
    htmc->irun      = TMC5160_IRUN_DEFAULT;
    htmc->ihold     = TMC5160_IHOLD_DEFAULT;
    htmc->microstep = TMC5160_MICROSTEP_DEFAULT;
    htmc->vmax      = TMC5160_VMAX_DEFAULT;
    htmc->amax      = TMC5160_AMAX_DEFAULT;

    if (TMC5160_GetIOIN(htmc).version != TMC5160_IC_VERSION) {return TMC5160_ERR;}

    // --- Driver configuration ---
    TMC5160_WriteRegister(htmc, TMC5160_GCONF, (0UL << 4));                          // Normal direction, GCONF.shaft = 0 (pg 32-33)
    TMC5160_WriteRegister(htmc, TMC5160_CHOPCONF, TMC5160_CHOPCONF_DEFAULT);         // pg 51
    TMC5160_WriteRegister(htmc, TMC5160_GLOBAL_SCALER, TMC5160_GLOBALSCALER_DEFAULT);// pg 36
    TMC5160_WriteRegister(htmc, TMC5160_PWMCONF, TMC5160_PWMCONF_DEFAULT);

    // Run/hold current. For Rsense = 0.075 ohm, Imax ~ 3.1 A (pg 38, 74)
    TMC5160_WriteRegister(htmc, TMC5160_IHOLD_IRUN, TMC5160_IRUNIHOLD_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_TPOWERDOWN, TMC5160_TPOWERDOWN_DEFAULT);     // Standstill to powerdown delay (pg 38)
    TMC5160_WriteRegister(htmc, TMC5160_TPWMTHRS, TMC5160_TPWMTHRS_DEFAULT);         // StealthChop/SpreadCycle crossover

    // --- Ramp profile (position mode, pg 40) ---
    TMC5160_WriteRegister(htmc, TMC5160_RAMPMODE, TMC5160_RAMPMODE_POSITION);
    TMC5160_WriteRegister(htmc, TMC5160_VSTART, TMC5160_VSTART_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_A1,     TMC5160_A1_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_V1,     TMC5160_V1_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_AMAX,   TMC5160_AMAX_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_VMAX,   TMC5160_VMAX_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_DMAX,   TMC5160_DMAX_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_D1,     TMC5160_D1_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_VSTOP,  TMC5160_VSTOP_DEFAULT);

    // --- Clear position ---
    TMC5160_WriteRegister(htmc, TMC5160_XACTUAL, TMC5160_POSITION_DEFAULT);
    TMC5160_WriteRegister(htmc, TMC5160_XTARGET, TMC5160_POSITION_DEFAULT);

    // --- Clear GStat (pg 33) --- 
    // Because reset = 1 and uv_cp = 1 stuck at power on
    TMC5160_WriteRegister(htmc, TMC5160_GSTAT, 0x07);
    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_SetCurrent(TMC5160_TypeDef *htmc, uint16_t current_ma)
{
    if (htmc == NULL) {return TMC5160_BADARG;}

    if (htmc->r_sense <= 0.0f) {return TMC5160_BADARG;}

    // Current scale from RMS current (pg 74, VFS = 0.325 V at VSENSE = 0):
    // Formula: CS = 32 * sqrt(2) * I_RMS * (Rsense) / VFS - 1
    float cs = 32 * SQRT_TWO * (current_ma/1000.0f) * (htmc->r_sense) / 0.325f - 1;

    TMC5160_Status_TypeDef status = TMC5160_OK;
    if (cs > 31.0f) {
        cs = 31.0f; // IRUN cannot be greater than 31
        status = TMC5160_CLAMPED;
    }
    else if (cs < 0.0f) {
        cs = 0.0f;
        status = TMC5160_CLAMPED;
    }
    htmc->irun = (uint8_t)(cs + 0.5f);  // Round to nearest
    htmc->ihold = htmc->irun / 2; 

    // IHOLD_IRUN: IHOLD[4:0], IRUN[12:8], IHOLDDELAY[19:16]
    TMC5160_WriteRegister(htmc, TMC5160_IHOLD_IRUN, 
    ((uint32_t)htmc->ihold << 0) |
    ((uint32_t)htmc->irun << 8)  |
    (6UL << 16));  

    return status;
}

TMC5160_Status_TypeDef TMC5160_SetEN(TMC5160_TypeDef *htmc, GPIO_PinState state)
{
    if (htmc == NULL || htmc->en.port == NULL) {return TMC5160_BADARG;}

    HAL_GPIO_WritePin(htmc->en.port, htmc->en.pin, state);
    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_SetRampMode(TMC5160_TypeDef *htmc, TMC5160_RampMode_TypeDef ramp_mode)
{
    if (htmc == NULL) {return TMC5160_BADARG;}

    switch (ramp_mode)
    {
        case TMC5160_RAMPMODE_POSITION:
        case TMC5160_RAMPMODE_VEL_POS:
        case TMC5160_RAMPMODE_VEL_NEG:
        case TMC5160_RAMPMODE_HOLD:
            TMC5160_WriteRegister(htmc, TMC5160_RAMPMODE, ramp_mode);
            break;
        default:
            return TMC5160_BADARG;
    }
    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_SetMicrostep(TMC5160_TypeDef *htmc, uint16_t microstep)
{   
    if (htmc == NULL) {return TMC5160_BADARG;}

    // MRES: CHOPCONF[27:24] (pg 51)
    uint32_t chopconf = htmc->reg_cache[TMC5160_CHOPCONF];
    chopconf &= ~(0xFUL << 24);

    // Set MRES bits
    switch (microstep)
    {
        case 256: chopconf |= (0x00UL << 24); htmc->microstep = 256; break;
        case 128: chopconf |= (0x01UL << 24); htmc->microstep = 128; break;
        case 64: chopconf |= (0x02UL << 24); htmc->microstep = 64; break;
        case 32: chopconf |= (0x03UL << 24); htmc->microstep = 32; break;
        case 16: chopconf |= (0x04UL << 24); htmc->microstep = 16; break;
        case 8: chopconf |= (0x05UL << 24); htmc->microstep = 8; break;
        case 4: chopconf |= (0x06UL << 24); htmc->microstep = 4; break;
        case 2: chopconf |= (0x07UL << 24); htmc->microstep = 2; break;
        case 1: chopconf |= (0x08UL << 24); htmc->microstep = 1; break;
        default: return TMC5160_BADARG;
    }

    TMC5160_WriteRegister(htmc, TMC5160_CHOPCONF, chopconf);
    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_SetVelocity(TMC5160_TypeDef *htmc, uint32_t max_velocity)
{  
    if (htmc == NULL) {return TMC5160_BADARG;}

    TMC5160_Status_TypeDef status = TMC5160_OK;
    if (max_velocity > TMC5160_VMAX_MAX) { // VMAX = 23 bits (pg 40)
        max_velocity = TMC5160_VMAX_MAX;
        status = TMC5160_CLAMPED;
    }

    htmc->vmax = max_velocity;
    TMC5160_WriteRegister(htmc, TMC5160_VMAX, max_velocity);
    return status;
}

TMC5160_Status_TypeDef TMC5160_SetAcceleration(TMC5160_TypeDef *htmc, uint32_t max_acceleration)
{
    if (htmc == NULL) {return TMC5160_BADARG;}

    TMC5160_Status_TypeDef status = TMC5160_OK;
    if (max_acceleration > TMC5160_AMAX_MAX) { // AMAX = 16 bits (pg 40)
        max_acceleration = TMC5160_AMAX_MAX;
        status = TMC5160_CLAMPED;
    }

    htmc->amax = max_acceleration;
    TMC5160_WriteRegister(htmc, TMC5160_AMAX, max_acceleration);
    return status;
}

TMC5160_Status_TypeDef TMC5160_SetPosition(TMC5160_TypeDef *htmc, int32_t position)
{
    if (htmc == NULL) {return TMC5160_BADARG;}

    TMC5160_WriteRegister(htmc, TMC5160_RAMPMODE, TMC5160_RAMPMODE_HOLD);
    TMC5160_WriteRegister(htmc, TMC5160_XTARGET, (uint32_t)position);
    TMC5160_WriteRegister(htmc, TMC5160_XACTUAL, (uint32_t)position);
    TMC5160_WriteRegister(htmc, TMC5160_RAMPMODE, TMC5160_RAMPMODE_POSITION);

    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_MoveTo(TMC5160_TypeDef *htmc, int32_t position)
{
    if (htmc == NULL) {return TMC5160_BADARG;}
    // Force POSITION MODE and move the motor to target position
    TMC5160_WriteRegister(htmc, TMC5160_RAMPMODE, TMC5160_RAMPMODE_POSITION);
    TMC5160_WriteRegister(htmc, TMC5160_XTARGET, position);
    return TMC5160_OK;
}

TMC5160_Status_TypeDef TMC5160_Stop(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return TMC5160_BADARG;}
    int32_t position = (int32_t)TMC5160_ReadRegister(htmc, TMC5160_XACTUAL);

    TMC5160_WriteRegister(htmc, TMC5160_XTARGET, position);
    TMC5160_SetRampMode(htmc, TMC5160_RAMPMODE_POSITION);

    return TMC5160_OK;
}

int32_t TMC5160_GetPosition(TMC5160_TypeDef *htmc)
{    
    if (htmc == NULL) {return 0;}

    return (int32_t)TMC5160_ReadRegister(htmc, TMC5160_XACTUAL);
}

int32_t TMC5160_GetVelocity(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return 0;}

    // VACTUAL is 24-bit signed; Mask and sign-extend
    uint32_t data = TMC5160_ReadRegister(htmc, TMC5160_VACTUAL) & 0x00FFFFFF;
    if (data & 0x00800000UL) {       // bit 23 set -> negative
        data |= 0xFF000000UL;        // sign-extend
    }

    return (int32_t)data;
}

uint32_t TMC5160_GetMaxVelocity(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return 0;}

    return htmc->vmax;
}

uint32_t TMC5160_GetMaxAcceleration(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return 0;}

    return htmc->amax;
}

TMC5160_RampMode_TypeDef TMC5160_GetRampMode(TMC5160_TypeDef *htmc) 
{
    if (htmc == NULL) {return TMC5160_RAMPMODE_POSITION;}

    uint32_t rampmode = TMC5160_ReadRegister(htmc, TMC5160_RAMPMODE);

    return (TMC5160_RampMode_TypeDef)(rampmode & 0x03U);
}

TMC5160_IOIN_TypeDef TMC5160_GetIOIN(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return (TMC5160_IOIN_TypeDef){0};}

    uint32_t ioin = TMC5160_ReadRegister(htmc, TMC5160_IOIN); // (pg 34)

    TMC5160_IOIN_TypeDef data = {
        .version = (ioin >> 24) & 0xFF, // Expect 0x30
        .sd_mode = (ioin >> 6) & 1, // SD MODE
        .drv_enn = (ioin >> 4) & 1 // EN Pin
    };

    return data;
}

TMC5160_DrvStat_TypeDef TMC5160_GetDrvStat(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return (TMC5160_DrvStat_TypeDef){0};}

    uint32_t stat = TMC5160_ReadRegister(htmc, TMC5160_DRV_STATUS); // (pg 56)

    TMC5160_DrvStat_TypeDef data = {
        .stst = (stat >> 31) & 1, // Standstill 
        .olb = (stat >> 30) & 1, // Open Load B
        .ola = (stat >> 29) & 1, // Open Load A
        .s2gb = (stat >> 28) & 1, // Short to GND B
        .s2ga = (stat >> 27) & 1, // Short to GND A
        .otpw = (stat >> 26) & 1, // Overtemperature Prewarning
        .ot = (stat >> 25) & 1, // Overtemperature
        .stallguard = (stat >> 24) & 1 // StallGuard
    };

    return data;
}

TMC5160_GStat_TypeDef TMC5160_GetGStat(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return (TMC5160_GStat_TypeDef){0};}

    uint32_t gstat = TMC5160_ReadRegister(htmc, TMC5160_GSTAT); // (pg 33)

    TMC5160_GStat_TypeDef data = {
        .reset = (gstat >> 0) & 1, // IC has been reset
        .drv_err = (gstat >> 1) & 1, // Driver shutdown
        .uv_cp = (gstat >> 2) & 1 // Undervoltage on charge pump
    };

    return data;
}

TMC5160_RampStat_TypeDef TMC5160_GetRampStat(TMC5160_TypeDef *htmc)
{
    if (htmc == NULL) {return (TMC5160_RampStat_TypeDef){0};}

    uint32_t rampstat = TMC5160_ReadRegister(htmc, TMC5160_RAMPSTAT); // (pg 44)

    TMC5160_RampStat_TypeDef data = {
        .status_sg = (rampstat >> 13) & 1,      // Stallguard flag
        .vzero = (rampstat >> 10) & 1,          // Vactual = 0 flag
        .position_reached = (rampstat >> 9) & 1,
        .velocity_reached = (rampstat >> 8) & 1,
        .status_stop_r = (rampstat >> 1) & 1,   // Reference switch status right
        .status_stop_l = (rampstat >> 0) & 1    // Reference switch status left
    };

    return data;
}