/**
 * @file    TMC5160.h
 * @brief   TMC5160 stepper driver: public API, handle type, and defaults.
 *
 * Author: Andrew Nguyen
 * Date:   June 2026
 */

#ifndef TMC5160_H
#define TMC5160_H

#include "TMC5160_Registers.h"
#include "stm32f4xx_hal.h"

/** @name Handle defaults */
///@{
#define TMC5160_IC_VERSION          0x30U           /**< Expected IOIN.VERSION of a TMC5160 */
#define TMC5160_CHOPCONF_DEFAULT    0x000100C3UL
#define TMC5160_IRUN_DEFAULT        20UL
#define TMC5160_IHOLD_DEFAULT       10UL
#define TMC5160_MICROSTEP_DEFAULT   256UL
#define TMC5160_R_SENSE_DEFAULT     0.075f          /**< Sense resistor in ohms */
///@}

/** @name Register defaults */
///@{
#define TMC5160_GLOBALSCALER_DEFAULT 0x00000000UL   /**< 0 = full scale (256) */
#define TMC5160_IRUNIHOLD_DEFAULT   0x00060A05UL    /**< IHOLD = 5, IRUN = 10, IHOLDDELAY = 6 */
#define TMC5160_TPOWERDOWN_DEFAULT  0x0000000AUL     /**< 10 */
#define TMC5160_TPWMTHRS_DEFAULT    0x000001F4UL     /**< 500 */
#define TMC5160_PWMCONF_DEFAULT     0xC10D0024UL
#define TMC5160_VSTART_DEFAULT      0x00000001UL     /**< 1 */
#define TMC5160_A1_DEFAULT          0x0000003E8UL    /**< 1000 */
#define TMC5160_V1_DEFAULT          0x0000C350UL     /**< 50000 */
#define TMC5160_AMAX_DEFAULT        0x000001F4UL     /**< 500 */
#define TMC5160_VMAX_DEFAULT        0x00030D40UL     /**< 200000 */
#define TMC5160_DMAX_DEFAULT        0x0000002BCUL    /**< 700 */
#define TMC5160_D1_DEFAULT          0x00000578UL     /**< 1400 */
#define TMC5160_VSTOP_DEFAULT       0x0000000AUL     /**< 10 */
#define TMC5160_POSITION_DEFAULT    0x00000000UL     /**< 0 */
///@}

/** @name Register field limits */
///@{
#define TMC5160_VMAX_MAX            0x007FFE00UL     /**< VMAX: 23-bit */
#define TMC5160_AMAX_MAX            0x0000FFFFUL     /**< AMAX: 16-bit */
///@}

#define SQRT_TWO                    1.41421356f      /**< sqrt(2) */

/** @brief GPIO port and pin */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Pin_TypeDef;

/** @brief RAMP MODE */
typedef enum {
    TMC5160_RAMPMODE_POSITION = 0x00,   /**< Move toward XTARGET */
    TMC5160_RAMPMODE_VEL_POS  = 0x01,   /**< Constant velocity, positive direction */
    TMC5160_RAMPMODE_VEL_NEG  = 0x02,   /**< Constant velocity, negative direction */
    TMC5160_RAMPMODE_HOLD     = 0x03    /**< Velocity held, no ramp */
} TMC5160_RampMode_TypeDef;

/** @brief API return status. */
typedef enum {
    TMC5160_OK = 0,     /**< Success */
    TMC5160_BADARG,     /**< NULL handle or invalid argument */
    TMC5160_CLAMPED,     /**< Success, but a value was clamped to its valid range */
    TMC5160_ERR         /**< Communication failure: the chip did not respond as expected */
} TMC5160_Status_TypeDef;

/** @brief Driver handle */
typedef struct {
    SPI_HandleTypeDef *hspi; /**< SPI */
    GPIO_Pin_TypeDef cs;    /**< Chip select */
    GPIO_Pin_TypeDef en;    /**< Enable */

    uint32_t reg_cache[128]; /**< Shadow copy of written registers (512 bytes) */

    float r_sense; /**< Sense resistor in ohm */
    uint8_t irun; /**< Run current */
    uint8_t ihold; /**< Hold current */

    uint32_t vmax; /**< Max velocity */
    uint32_t amax; /**< Max acceleration */

    uint16_t microstep; /**< Microstep resolution */
} TMC5160_TypeDef;

/** @brief User configuration */
typedef struct {
    SPI_HandleTypeDef *hspi; /**< SPI */
    GPIO_Pin_TypeDef cs;    /**< Chip select */
    GPIO_Pin_TypeDef en;    /**< Enable */

    float r_sense;          /**< Sense resistor in ohm */
} TMC5160_Config_TypeDef;

/** @brief IOIN register */
typedef struct {
    uint8_t version;    /**< IC version, expect 0x30 */
    uint8_t sd_mode;    /**< Step/dir mode active */
    uint8_t drv_enn;    /**< Enable pin state */
} TMC5160_IOIN_TypeDef;

/** @brief DRV_STATUS register */
typedef struct {
    uint8_t stst;       /**< Standstill */
    uint8_t olb;        /**< Open load phase B */
    uint8_t ola;        /**< Open load phase A */
    uint8_t s2gb;       /**< Short to GND phase B */
    uint8_t s2ga;       /**< Short to GND phase A */
    uint8_t otpw;       /**< Overtemperature prewarning */
    uint8_t ot;         /**< Overtemperature shutdown */
    uint8_t stallguard; /**< StallGuard status */
} TMC5160_DrvStat_TypeDef;

/** @brief GSTAT register */
typedef struct {
    uint8_t reset;      /**< IC has been reset */
    uint8_t drv_err;    /**< Driver shutdown due to error */
    uint8_t uv_cp;      /**< Undervoltage on charge pump */
} TMC5160_GStat_TypeDef;

/** @brief RAMP_STAT register */
typedef struct {
    uint8_t status_sg;          /**< StallGuard2 flag */
    uint8_t vzero;              /**< Vactual is zero flag */
    uint8_t position_reached;   /**< Position reached flag: 1 means Xtarget = Xactual */
    uint8_t velocity_reached;   /**< Velocity reached flag: 1 means Vactual = Vmax */
    uint8_t status_stop_r;       /**< Reference switch right status */
    uint8_t status_stop_l;       /**< Reference switch left status */
} TMC5160_RampStat_TypeDef;

/**
 * @brief  Write a 32-bit value to a register and update the shadow cache
 * @param  htmc        Driver handle
 * @param  reg_address Register address (0 to 127)
 * @param  data        Value to write
 */
void TMC5160_WriteRegister(TMC5160_TypeDef *htmc, uint8_t reg_address, uint32_t data);

/**
 * @brief  Read a 32-bit value from a register
 * @param  htmc        Driver handle
 * @param  reg_address Register address (0 to 127)
 * @return Register value, or 0 on invalid argument
 */
uint32_t TMC5160_ReadRegister(TMC5160_TypeDef *htmc, uint8_t reg_address);

/**
 * @brief  Initialise a handle and load the default register set
 * @param  htmc Handle to populate
 * @param  cfg  User configuration (SPI, CS/EN pins, sense resistor)
 * @retval TMC5160_OK     Initialised
 * @retval TMC5160_BADARG htmc, cfg, or cfg->hspi was NULL
 */
TMC5160_Status_TypeDef TMC5160_Init(TMC5160_TypeDef *htmc, const TMC5160_Config_TypeDef *cfg);

/**
 * @brief  Drive the enable pin
 * @param  htmc  Driver handle
 * @param  state GPIO_PIN_RESET enables the driver, GPIO_PIN_SET disables the driver
 * @retval TMC5160_OK     Pin written
 * @retval TMC5160_BADARG htmc or the enable port was NULL
 */
TMC5160_Status_TypeDef TMC5160_SetEN(TMC5160_TypeDef *htmc, GPIO_PinState state);

/**
 * @brief  Set the run current; hold current is set to half the run value
 * @param  htmc       Driver handle
 * @param  current_ma Desired RMS run current in milliamps
 * @retval TMC5160_OK      Applied
 * @retval TMC5160_BADARG  htmc was NULL or the sense resistor is not positive
 * @retval TMC5160_CLAMPED Current fell outside the CS range (0 to 31) and was clamped
 */
TMC5160_Status_TypeDef TMC5160_SetCurrent(TMC5160_TypeDef *htmc, uint16_t current_ma);

/**
 * @brief  Select the ramp generator mode
 * @param  htmc      Driver handle
 * @param  ramp_mode One of TMC5160_RampMode_TypeDef
 * @retval TMC5160_OK     Applied
 * @retval TMC5160_BADARG htmc was NULL or ramp_mode was unrecognised
 */
TMC5160_Status_TypeDef TMC5160_SetRampMode(TMC5160_TypeDef *htmc, TMC5160_RampMode_TypeDef ramp_mode);

/**
 * @brief  Set microstep resolution (MRES field of CHOPCONF)
 * @param  htmc      Driver handle
 * @param  microstep Microsteps per full step: 1, 2, 4, 8, 16, 32, 64, 128, or 256
 * @retval TMC5160_OK     Applied
 * @retval TMC5160_BADARG htmc was NULL or microstep was not a supported value
 */
TMC5160_Status_TypeDef TMC5160_SetMicrostep(TMC5160_TypeDef *htmc, uint16_t microstep);

/**
 * @brief  Set the maximum ramp velocity (VMAX)
 * @param  htmc         Driver handle
 * @param  max_velocity Target velocity; clamped to TMC5160_VMAX_MAX
 * @retval TMC5160_OK      Applied as requested
 * @retval TMC5160_BADARG  htmc was NULL
 * @retval TMC5160_CLAMPED Value exceeded the limit and was clamped
 */
TMC5160_Status_TypeDef TMC5160_SetVelocity(TMC5160_TypeDef *htmc, uint32_t max_velocity);

/**
 * @brief  Set the maximum ramp acceleration (AMAX)
 * @param  htmc             Driver handle
 * @param  max_acceleration Target acceleration; clamped to TMC5160_AMAX_MAX
 * @retval TMC5160_OK      Applied as requested
 * @retval TMC5160_BADARG  htmc was NULL
 * @retval TMC5160_CLAMPED Value exceeded the limit and was clamped
 */
TMC5160_Status_TypeDef TMC5160_SetAcceleration(TMC5160_TypeDef *htmc, uint32_t max_acceleration);

/**
 * @brief Set the position of the shaft (XACTUAL)
 * @param  htmc             Driver handle
 * @param  position         New position in microsteps (signed)
 * @retval TMC5160_OK     Position redefined
 * @retval TMC5160_BADARG htmc was NULL
 */
TMC5160_Status_TypeDef TMC5160_SetPosition(TMC5160_TypeDef *htmc, int32_t position);

/**
 * @brief  Switch to position mode and move to an absolute target
 * @param  htmc     Driver handle
 * @param  position Target position in microsteps (signed)
 * @retval TMC5160_OK     Target accepted
 * @retval TMC5160_BADARG htmc was NULL
 */
TMC5160_Status_TypeDef TMC5160_MoveTo(TMC5160_TypeDef *htmc, int32_t position);

/**
 * @brief  Switch to position mode and stop
 * @param  htmc     Driver handle
 * @retval TMC5160_OK     Stop commanded
 * @retval TMC5160_BADARG htmc was NULL
 */
TMC5160_Status_TypeDef TMC5160_Stop(TMC5160_TypeDef *htmc);

/**
 * @brief  Read the actual position (XACTUAL)
 * @param  htmc Driver handle
 * @return Position in microsteps (signed)
 */
int32_t TMC5160_GetPosition(TMC5160_TypeDef *htmc);

/**
 * @brief  Read the actual ramp velocity (VACTUAL)
 * @param  htmc Driver handle
 * @return Velocity (signed)
 */
int32_t TMC5160_GetVelocity(TMC5160_TypeDef *htmc);

/**
 * @brief Read the max ramp velocity (VMAX)
 * @param htmc Driver handle
 * @return Max velocity (unsigned)
 */
uint32_t TMC5160_GetMaxVelocity(TMC5160_TypeDef *htmc);

/**
 * @brief Read the max ramp acceleration (AMAX)
 * @param htmc Driver handle
 * @return Max acceleration (unsigned)
 */
uint32_t TMC5160_GetMaxAcceleration(TMC5160_TypeDef *htmc);

/**
 * @brief Read the current ramp mode
 * @param htmc Driver handle
 * @return ramp mode
 */
TMC5160_RampMode_TypeDef TMC5160_GetRampMode(TMC5160_TypeDef *htmc);

/**
 * @brief  Read and decode the IOIN register (pg 34)
 * @param  htmc Driver handle
 * @return Decoded input states
 */
TMC5160_IOIN_TypeDef TMC5160_GetIOIN(TMC5160_TypeDef *htmc);

/**
 * @brief  Read and decode the DRV_STATUS register (pg 56)
 * @param  htmc Driver handle
 * @return Decoded driver status flags
 */
TMC5160_DrvStat_TypeDef TMC5160_GetDrvStat(TMC5160_TypeDef *htmc);

/**
 * @brief  Read and decode the GSTAT register (pg 33)
 * @param  htmc Driver handle
 * @return Decoded global status flags
 */
TMC5160_GStat_TypeDef TMC5160_GetGStat(TMC5160_TypeDef *htmc);

/**
 * @brief Read and decode the RAMPSTAT register (pg 44)
 * @param htmc Driver handle
 * @return Decoded ramp status flags
 */
TMC5160_RampStat_TypeDef TMC5160_GetRampStat(TMC5160_TypeDef *htmc);

#endif