/**
 * @file    Stepper.h
 * @brief   Stepper motor control layer: degree-based motion on top of the
 *          TMC5160 driver library.
 *
 * Author: Andrew Nguyen
 * Date:   July 2026
 */

#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include "TMC5160.h"

/** @name Constants */
///@{
#define STEPPER_STEP_ANGLE_DEFAULT  1.8f            /**< Full-step angle of a typical NEMA motor, in degrees */
#define STEPPER_STEPS_LIMIT         2147483648.0f   /**< |target| must stay below 2^31 microsteps (XTARGET is int32) */
#define STEPPER_FCLK                12000000.0f     /**< TMC5160 internal clock */
#define STEPPER_VEL_TIME_BASE       16777216.0f     /**< 2^24. VMAX = v[usteps/s] * 2^24 / fCLK (pg 42) */
#define STEPPER_ACCEL_TIME_BASE     2199023255552.0f/**< 2^41. AMAX = a[usteps/s^2] * 2^41 / fCLK^2 (pg 42) */
#define STEPPER_TIMEOUT_MS          10000U          /**< Default deadline for blocking waits, in ms */
#define STEPPER_POLL_MS             5U              /**< Stall poll interval */
///@}

/**
 * @brief Propagate the first non-OK status out of the calling function.
 *        Only usable inside functions returning Stepper_Status_TypeDef.
 */
#define STEPPER_CHECK(expr)                         \
    do {                                            \
        Stepper_Status_TypeDef err = (expr);        \
        if (err != STEPPER_OK) {                    \
            return err;                             \
        }                                           \
} while(0)

/** @brief API return status */
typedef enum {
    STEPPER_OK = 0,     /**< Success */
    STEPPER_BADARG,     /**< NULL handle or invalid argument */
    STEPPER_ERR,         /**< Driver-level failure */
    STEPPER_TIMEOUT     /**< blocking wait expired */
} Stepper_Status_TypeDef;

/**
 * @brief Ramp generator mode. Values map 1:1 onto TMC5160_RampMode_TypeDef.
 */
typedef enum {
    STEPPER_MODE_POSITION = 0,  /**< Move toward the commanded target (in degrees) */
    STEPPER_MODE_VEL_POS,       /**< Spin continuously at VMAX, positive direction */
    STEPPER_MODE_VEL_NEG,       /**< Spin continuously at VMAX, negative direction */
    STEPPER_MODE_HOLD           /**< Hold current velocity */
} Stepper_Mode_TypeDef;

// --- TypeDefs ---

/**
 * @brief User configuration for Stepper_Init.
 *
 * The SPI peripheral and both GPIO pins must be initialised before calling
 * Stepper_Init. Every field is required: current_ma and microstep are applied
 * during init, so a zeroed microstep fails with STEPPER_BADARG.
 */
typedef struct
{
    GPIO_Pin_TypeDef cs;        /**< Chip-select pin (active low) */
    GPIO_Pin_TypeDef en;        /**< Driver enable pin (active low) */

    SPI_HandleTypeDef *hspi;    /**< Initialised SPI handle (mode 3, MSB first) */

    float r_sense;              /**< Sense resistor of the driver board (ohms).*/
    float step_angle;           /**< Motor full-step angle (degrees) */

    uint16_t current_ma;        /**< Motor running current (mA) */
    uint16_t microstep;         /**< Motor microstep resolution: 1, 2, 4, 8, 16, 32, 64, 128, or 256 */
} Stepper_Config_TypeDef;

/**
 * @brief Stepper handle. Treat all fields as private; use the API.
 */
typedef struct {
    TMC5160_TypeDef htmc;       /**< TMC5160 driver handle */

    float step_angle;           /**< Full-step angle (degrees) */
    float steps_per_deg;        /**< Cached microsteps-per-degree conversion */
} Stepper_TypeDef;

/**
 * @brief Aggregated diagnostic snapshot of all four TMC5160 status registers.
 */
typedef struct {
    TMC5160_IOIN_TypeDef ioin;          /**< Pin states and IC version (expect 0x30) */
    TMC5160_DrvStat_TypeDef drv_stat;   /**< Driver faults */
    TMC5160_GStat_TypeDef g_stat;       /**< Global faults */
    TMC5160_RampStat_TypeDef ramp_stat; /**< Motion flags */
} Stepper_Diag_TypeDef;

// --- APIs ---
// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
/**
 * @brief  Initialise the stepper and the TMC5160 driver.
 *
 * Defaults: Mode: POSITION, XACTUAL & XTARGET: 0
 * Applies cfg sense resistor, current and microstep
 *
 * @param  hs  Handle stepper
 * @param  cfg User configuration (see Stepper_Config_TypeDef requirements)
 * @retval STEPPER_OK     Initialised
 * @retval STEPPER_BADARG hs or cfg was NULL, step_angle was not positive/finite,
 *                        or cfg->microstep was not a supported value
 * @retval STEPPER_ERR    TMC5160 initialisation failed, or cfg->current_ma was
 *                        outside the range this board can deliver (the clamped
 *                        value was still written to the driver)
 */
Stepper_Status_TypeDef Stepper_Init(Stepper_TypeDef *hs, const Stepper_Config_TypeDef *cfg);

/**
 * @brief  Power the motor
 * @param  hs Handle stepper
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_Enable(Stepper_TypeDef *hs);

/**
 * @brief  Unpower the motor
 * @param  hs Handle stepper
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_Disable(Stepper_TypeDef *hs);

/**
 * @brief  Change the motor full-step angle and recompute the degree conversion.
 * @param  hs         Handle stepper
 * @param  step_angle Full-step angle in degrees; must be positive and finite
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_SetAngle(Stepper_TypeDef *hs, float step_angle);

/**
 * @brief  Set the motor RMS run current; hold current becomes half the run value.
 * @param  hs         Handle stepper
 * @param  current_ma Desired RMS run current in milliamps
 * @retval STEPPER_OK     Applied as requested
 * @retval STEPPER_BADARG hs was NULL
 * @retval STEPPER_ERR    Request exceeded the driver's current-scale range;
 *                        the clamped value was still written to the driver
 */
Stepper_Status_TypeDef Stepper_SetCurrent(Stepper_TypeDef *hs, uint16_t current_ma);

/**
 * @brief  Change microstep resolution; the degree conversion is recomputed.
 * @warning The TMC5160 does not rescale XACTUAL when MRES changes.
 * @param  hs        Handle stepper
 * @param  microstep 1, 2, 4, 8, 16, 32, 64, 128, or 256
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_SetMicrostep(Stepper_TypeDef *hs, uint16_t microstep);

/**
 * @brief  Select the ramp generator mode.
 * @param  hs   Handle stepper.
 * @param  mode One of Stepper_Mode_TypeDef
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_SetMode(Stepper_TypeDef *hs, Stepper_Mode_TypeDef mode);

// -----------------------------------------------------------------------------
// VELOCITY AND ACCELERATION
// -----------------------------------------------------------------------------
/**
 * @brief  Set the maximum velocity in physical units.
 * @param  hs          Handle stepper
 * @param  deg_per_sec Target velocity in degrees per second; must be > 0 and finite
 * @retval STEPPER_OK     Applied as requested
 * @retval STEPPER_BADARG Invalid argument
 * @retval STEPPER_ERR    Uninitialised handle, or request exceeded VMAX range
 *                        (the clamped value was still applied)
 */
Stepper_Status_TypeDef Stepper_SetVelocity(Stepper_TypeDef *hs, float deg_per_sec);

/**
 * @brief  Set the maximum acceleration in physical units.
 * @param  hs           Handle stepper.
 * @param  deg_per_sec2 Target acceleration in degrees per second squared; > 0, finite
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR (clamp semantics as SetVelocity)
 */
Stepper_Status_TypeDef Stepper_SetAcceleration(Stepper_TypeDef *hs, float deg_per_sec2);

/**
 * @brief  Set the maximum velocity as a percentage of the driver's absolute maximum.
 * @param  hs      Handle stepper.
 * @param  percent 0 to 100
 * @retval STEPPER_OK     Applied as requested
 * @retval STEPPER_BADARG Invalid argument
 * @retval STEPPER_ERR    Uninitialised handle, or the value was clamped by the
 *                        driver (the clamped value was still applied)
 */
Stepper_Status_TypeDef Stepper_SetVelocityPercentage(Stepper_TypeDef *hs, float percent);

/**
 * @brief  Set the maximum acceleration as a percentage of the driver's absolute maximum.
 * @param  hs      Handle stepper.
 * @param  percent 0 to 100
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR (clamp semantics as SetVelocityPercentage)
 */
Stepper_Status_TypeDef Stepper_SetAccelerationPercentage(Stepper_TypeDef *hs, float percent);

// -----------------------------------------------------------------------------
// MOTION
// -----------------------------------------------------------------------------
/**
 * @brief  Move to an absolute shaft angle.
 * @param  hs  Handle stepper.
 * @param  deg Absolute target in degrees (signed, multi-turn allowed)
 * @retval STEPPER_OK     Target accepted
 * @retval STEPPER_BADARG Invalid argument or target outside the int32 microstep range
 * @retval STEPPER_ERR    Uninitialised handle or driver failure
 */
Stepper_Status_TypeDef Stepper_MoveTo(Stepper_TypeDef *hs, float deg);

/**
 * @brief  Move relative to the current position.
 * @param  hs  Handle stepper.
 * @param  deg Offset in degrees (signed)
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR (as Stepper_MoveTo)
 */
Stepper_Status_TypeDef Stepper_MoveBy(Stepper_TypeDef *hs, float deg);

/**
 * @brief  Stop the motor
 * @param  hs  Handle stepper.
 * @note   Stops the motor by setting the current position (XACTUAL)
 *        as the new target position (XTARGET)
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_Stop(Stepper_TypeDef *hs);

/**
 * @brief  Redefine the current shaft angle without moving the motor.
 * @warning Call only at standstill. Redefining the position while the motor is
 *          moving causes an immediate uncontrolled jump.
 * @param  hs  Handle stepper.
 * @param  deg Position in degrees
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_ERR
 */
Stepper_Status_TypeDef Stepper_SetPosition(Stepper_TypeDef *hs, float deg);

/**
 * @brief Wait for the motor to stop from motion.
 * @param hs Handle stepper.
 * @retval STEPPER_OK / STEPPER_BADARG / STEPPER_TIMEOUT
 *
 */
Stepper_Status_TypeDef Stepper_WaitStopped(Stepper_TypeDef *hs);

// -----------------------------------------------------------------------------
// STATUS and READBACK
// -----------------------------------------------------------------------------
/**
 * @brief  Read the current shaft position.
 * @param  hs Handle stepper.
 * @return Position in degrees.
 * @warning Returns 0.0f on NULL or uninitialised handle -- indistinguishable
 *          from a genuine 0 deg reading. Check handle validity separately.
 */
float Stepper_GetPosition(Stepper_TypeDef *hs);

/**
 * @brief  Read the configured maximum velocity.
 * @param  hs Handle
 * @return Velocity in deg/s
 */
float Stepper_GetVelocity(Stepper_TypeDef *hs);

/**
 * @brief  Read the configured maximum acceleration.
 * @param  hs Handle
 * @return Acceleration in deg/s^2
 */
float Stepper_GetAcceleration(Stepper_TypeDef *hs);

/**
 * @brief  Read the mode.
 * @param  hs Handle stepper.
 * @return Current mode
 */
Stepper_Mode_TypeDef Stepper_GetMode(Stepper_TypeDef *hs);

/**
 * @brief  True while motor is moving. Works in all modes.
 * @param  hs Handle stepper.
 * @return Motion state.
 */
bool Stepper_IsMoving(Stepper_TypeDef *hs);

/**
 * @brief  True while motor is stalled
 * @param  hs Handle stepper.
 * @return Stall state.
 */
bool Stepper_IsStalled(Stepper_TypeDef *hs);

/**
 * @brief  True when XACTUAL has reached XTARGET.
 * @param  hs Handle stepper.
 * @return Completion state.
 */
bool Stepper_PositionReached(Stepper_TypeDef *hs);

/**
 * @brief  True when VACTUAL has reached VMAX.
 * @param  hs Handle stepper.
 * @return Completion state.
 */
bool Stepper_VelocityReached(Stepper_TypeDef *hs);

// -----------------------------------------------------------------------------
// DIAGNOSTICS
// -----------------------------------------------------------------------------
/**
 * @brief  Read all four TMC5160 status registers.
 * @param  hs Handle stepper.
 * @return Populated data.
 */
Stepper_Diag_TypeDef Stepper_GetDiag(Stepper_TypeDef *hs);

/**
 * @brief  Dump a diagnostic report over UART.
 * @param  hs    Handle stepper.
 * @param  huart Initialised UART handle
 * @retval STEPPER_OK / STEPPER_BADARG
 */
Stepper_Status_TypeDef Stepper_PrintDiag(Stepper_TypeDef *hs, UART_HandleTypeDef *huart);

#endif