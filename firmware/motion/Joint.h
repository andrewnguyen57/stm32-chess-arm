/**
 * @file    Joint.h
 * @brief   Joint layer: joint-level control on top of the Stepper library.
 *
 * Author: Andrew Nguyen
 * Date:   July 2026
 *
 */

#ifndef JOINT_H
#define JOINT_H

#include <stdbool.h>
#include "Stepper.h"

/** @name Calibration parameters (joint units) */
///@{
#define JOINT_HOME_ANGLE              0.0f        /**< Joint angle stamped when the sensor triggers */
#define JOINT_LIMIT_SEARCH_ANGLE      360.0f      /**< Sweep target for a limit search; must exceed the joint's real travel */
#define JOINT_LIMIT_MARGIN            2.0f        /**< Margin to pull back from the stall point*/
#define JOINT_SETUP_VELOCITY          50.0f       /**< Homing sweep velocity in deg/s */
#define JOINT_SETUP_ACCELERATION      100.0f       /**< Homing ramp acceleration in deg/s^2 */
#define JOINT_SETUP_TIMEOUT_MS        10000U      /**< Abort one sweep after this long */
#define JOINT_SETUP_POLL_MS           5U          /**< Sensor/stall poll interval */
#define JOINT_STALL_DETECT_DELAY_MS   500U        /**< Delay before checking stall during setup */
#define JOINT_STALL_COUNT             3U          /**< Number of stalls read before deciding actual stall    */
///@}

/** @name Default velocity and acceleration*/
///@{
#define JOINT_DEFAULT_VELOCITY        180.0f      /**< Default velocity (deg/s) */
#define JOINT_DEFAULT_ACCELERATION    360.0f      /**< Default acceleration (deg/s²) */
#define JOINT_VEL_ACC_MULTIPLIER      3.0f        /**< Acceleration-to-velocity multiplier: 1/k is the ramp time in seconds */
///@}

/** @brief API return status */
typedef enum {
    JOINT_OK = 0,     /**< Success */
    JOINT_BADARG,     /**< NULL handle or invalid argument */
    JOINT_NOT_HOMED,  /**< Operation requires a successful Joint_Home first */
    JOINT_NOT_FOUND,  /**< Swept the full search range without finding a hard stop */
    JOINT_ERR,        /**< Stepper-level failure, or stalled in both directions */
    JOINT_TIMEOUT,     /**< Sweep finished without finding the sensor or stalling */
    JOINT_OVER_LIMIT
} Joint_Status_TypeDef;

/**
 * @brief User configuration for Joint_Init.
 */
typedef struct
{
    GPIO_Pin_TypeDef sensor;    /**< Home reference sensor input */

    Stepper_Config_TypeDef s_cfg; /**< Stepper motor set up config */

    float gear_ratio;           /**< Motor degrees per joint degree; must be > 0 */

    float joint_len;            /**< Joint length */

} Joint_Config_TypeDef;

/**
 * @brief Joint handle. Treat all fields as private; use the API.
 */
typedef struct {
    Stepper_TypeDef hs;         /**< Stepper driving this joint */

    GPIO_Pin_TypeDef sensor;    /**< Home reference sensor input */

    float joint_len;            /**< Length of joint */

    float gear_ratio;           /**< Motor degrees per joint degree */

    float home;                 /**< Home angle in joint units */
    bool homed;                 /**< True once homing has succeeded */

    float limit_min;            /**< Limit angle (negative) */
    float limit_max;            /**< Limit angle (positive)*/
    bool limited_min;           /**< True once the lower limit has been found */
    bool limited_max;           /**< True once the upper limit has been found */
} Joint_TypeDef;

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
/**
 * @brief  Initialise a joint and the stepper driving it.
 *
 * Applies the default velocity and acceleration scaled by the gear ratio and
 * enables the motor. The joint is left un-homed: absolute angles are
 * meaningless until Joint_Home succeeds.
 *
 * @param  hj  Joint handle to populate
 * @param  cfg User configuration
 * @retval JOINT_OK     Initialised
 * @retval JOINT_BADARG hj or cfg was NULL, or gear_ratio/joint_len was not positive
 * @retval JOINT_ERR    Stepper-level failure
 */
Joint_Status_TypeDef Joint_Init(Joint_TypeDef *hj, const Joint_Config_TypeDef *cfg);
 
/**
 * @brief  Find the home reference and stamp the joint's zero position.
 * @param  hj Joint handle
 * @retval JOINT_OK      Home found and position stamped
 * @retval JOINT_BADARG  hj was NULL
 * @retval JOINT_TIMEOUT A sweep ran to JOINT_SETUP_TIMEOUT_MS without result
 * @retval JOINT_ERR     Stepper-level failure, or stalled in both directions
 */
Joint_Status_TypeDef Joint_Home(Joint_TypeDef *hj);
 
/**
 * @brief  Find the joint's travel limits by driving into the hard stops.
 * @param  hj Joint handle
 * @retval JOINT_OK        At least one limit was found and stored
 * @retval JOINT_BADARG    hj was NULL
 * @retval JOINT_NOT_HOMED This joint was not homed
 * @retval JOINT_NOT_FOUND No limit found
 * @retval JOINT_TIMEOUT   A sweep ran to JOINT_SETUP_TIMEOUT_MS without result
 * @retval JOINT_ERR       Stepper-level failure
 */
Joint_Status_TypeDef Joint_Limit(Joint_TypeDef *hj);

/**
 * @brief  Set the joint speed; acceleration follows automatically.
 * @param  hj    Joint handle
 * @param  speed Target speed in joint deg/s; must be > 0 and finite
 * @retval JOINT_OK / JOINT_BADARG / JOINT_ERR
 */
Joint_Status_TypeDef Joint_SetSpeed(Joint_TypeDef *hj, float speed);

// -----------------------------------------------------------------------------
// MOTION
// -----------------------------------------------------------------------------
/**
 * @brief  Move the joint back to its home position.
 * @note   Returns immediately; the move is executed by the ramp generator.
 * @param  hj Joint handle
 * @retval JOINT_OK        Target accepted
 * @retval JOINT_BADARG    hj was NULL
 * @retval JOINT_NOT_HOMED This joint was not homed
 * @retval JOINT_ERR       Stepper-level failure
 */
Joint_Status_TypeDef Joint_MoveHome(Joint_TypeDef *hj);

/**
 * @brief  Move to an absolute joint angle.
 * @note   Use with Joint_WaitStopped for complete movement
 * @param  hj  Joint handle
 * @param  deg Absolute target in joint degrees
 * @retval JOINT_OK         Target accepted
 * @retval JOINT_BADARG     hj was NULL, deg was not finite
 * @retval JOINT_NOT_HOMED  This joint was not homed
 * @retval JOINT_ERR        Stepper-level failure
 * @retval JOINT_OVER_LIMIT Target out-of-range
 */
Joint_Status_TypeDef Joint_MoveTo(Joint_TypeDef *hj, float deg);

/**
 * @brief  Move relative to the current joint angle.
 * @param  hj  Joint handle
 * @param  deg Offset in joint degrees (signed)
 * @retval JOINT_OK / JOINT_BADARG / JOINT_NOT_HOMED / JOINT_ERR / JOINT_OVER_LIMIT (as Joint_MoveTo)
 */
Joint_Status_TypeDef Joint_MoveBy(Joint_TypeDef *hj, float deg);

/**
 * @brief  Decelerate the joint to a stop.
 * @param  hj Joint handle
 * @retval JOINT_OK / JOINT_BADARG / JOINT_ERR
 */
Joint_Status_TypeDef Joint_Stop(Joint_TypeDef *hj);

/**
 * @brief  Block until the joint reaches standstill.
 * @param  hj Joint handle
 * @retval JOINT_OK      Motor stopped
 * @retval JOINT_BADARG  hj was NULL
 * @retval JOINT_TIMEOUT Still moving when the deadline expired
 * @retval JOINT_ERR     Stepper-level failure
 */
Joint_Status_TypeDef Joint_WaitStopped(Joint_TypeDef *hj);

/**
 * @brief  Move to an absolute joint angle and wait for the move to finish.
 * @param  hj  Joint handle
 * @param  deg Absolute target in joint degrees
 * @retval JOINT_OK         Move completed
 * @retval JOINT_BADARG     hj was NULL or deg was not finite
 * @retval JOINT_NOT_HOMED  This joint was not homed
 * @retval JOINT_OVER_LIMIT Target out-of-range
 * @retval JOINT_TIMEOUT    The move did not complete before the deadline
 * @retval JOINT_ERR        Stepper-level failure
 */
Joint_Status_TypeDef Joint_Move(Joint_TypeDef *hj, float deg);

// -----------------------------------------------------------------------------
// STATUS and READBACK
// -----------------------------------------------------------------------------
/**
 * @brief  Read the current joint angle.
 * @param  hj Joint handle
 * @return Position in joint degrees; 0.0f if hj is NULL.
 */
float Joint_GetPosition(Joint_TypeDef *hj);

/**
 * @brief  Read the configured joint speed.
 * @param  hj Joint handle
 * @return Speed in joint deg/s; 0.0f if hj is NULL.
 */
float Joint_GetSpeed(Joint_TypeDef *hj);

/**
 * @brief  True while the joint is moving.
 * @param  hj Joint handle
 * @return Motion state; false if hj is NULL.
 */
bool Joint_IsMoving(Joint_TypeDef *hj);

/**
 * @brief  True when the joint has reached its commanded target.
 * @param  hj Joint handle
 * @return Completion state; false if hj is NULL.
 */
bool Joint_PositionReached(Joint_TypeDef *hj);


#endif