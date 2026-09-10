/**
 * @file Joint.c
 * @brief Joint Controller
 *
 * Author: Andrew Nguyen
 * Date: July 2026
 * Require: Stepper Motor Controller Library & TMC5160 Driver Library
 */

#include "Joint.h"
#include <math.h>

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
Joint_Status_TypeDef Joint_Init(Joint_TypeDef *hj, const Joint_Config_TypeDef *cfg)
{
    if (hj == NULL || cfg == NULL) {return JOINT_BADARG;}
    if (cfg->gear_ratio <= 0.0f || cfg->joint_len <= 0.0f) {return JOINT_BADARG;}

    hj->home = JOINT_HOME_ANGLE;
    hj->homed = false;

    hj->limit_min = 0.0f;
    hj->limit_max = 0.0f;
    hj->limited_min = false;
    hj->limited_max = false;

    hj->gear_ratio = cfg->gear_ratio;
    hj->joint_len = cfg->joint_len;

    if (Stepper_Init(&hj->hs, &cfg->s_cfg) != STEPPER_OK) {return JOINT_ERR;}
    if (Stepper_SetVelocity(&hj->hs, JOINT_DEFAULT_VELOCITY * hj->gear_ratio) != STEPPER_OK) {return JOINT_ERR;}
    if (Stepper_SetAcceleration(&hj->hs, JOINT_DEFAULT_ACCELERATION * hj->gear_ratio) != STEPPER_OK) {return JOINT_ERR;}
    if (Stepper_Enable(&hj->hs) != STEPPER_OK) {return JOINT_ERR;}

    return JOINT_OK;
}

Joint_Status_TypeDef Joint_Home(Joint_TypeDef *hj)
{
    if (hj == NULL) {return JOINT_BADARG;}

    hj->homed = false;
    hj->limited_min = false;
    hj->limited_max = false;

    // Saved velocity and acceleration for restoration once homing is finished
    float saved_vmax = Stepper_GetVelocity(&hj->hs);
    float saved_amax = Stepper_GetAcceleration(&hj->hs);

    Joint_Status_TypeDef status = JOINT_ERR;

    // Set temporary velocity and acceleration for homing task
    if (Stepper_SetVelocity(&hj->hs, JOINT_SETUP_VELOCITY * hj->gear_ratio) != STEPPER_OK) {goto restore;}
    if (Stepper_SetAcceleration(&hj->hs, JOINT_SETUP_ACCELERATION * hj->gear_ratio) != STEPPER_OK) {goto restore;}

    // Attempt homing in 2 different directions
    Stepper_Mode_TypeDef mode = STEPPER_MODE_VEL_POS;
    for (uint8_t attempt = 0; attempt < 2; attempt++) {

        if (Stepper_SetMode(&hj->hs, mode) != STEPPER_OK) {goto restore;}

        uint32_t start = HAL_GetTick();
        float home_position = 0.0f;
        uint8_t stall_count = 0;
        bool found   = false;
        bool stalled = false;
    
        while (HAL_GetTick() - start <= JOINT_SETUP_TIMEOUT_MS) {
            // Detect home position
            if (HAL_GPIO_ReadPin(hj->sensor.port, hj->sensor.pin) == GPIO_PIN_RESET) {
                home_position = Stepper_GetPosition(&hj->hs);
                found = true;
                break;
            }
            // Stall checks need a delay at motor startup
            // Stall is only detected after 3 detections
            if (HAL_GetTick() - start >= JOINT_STALL_DETECT_DELAY_MS) {
                if (Stepper_IsStalled(&hj->hs)) {if (++stall_count >= JOINT_STALL_COUNT) {stalled = true; break;}}
                else {stall_count = 0;}
            }
            HAL_Delay(JOINT_SETUP_POLL_MS);
        }

        // Stop
        if (Stepper_Stop(&hj->hs) != STEPPER_OK) {goto restore;}
        if (Stepper_WaitStopped(&hj->hs) != STEPPER_OK) {status = JOINT_TIMEOUT; goto restore;}

        if (found) {
            if (Stepper_MoveTo(&hj->hs, home_position) != STEPPER_OK) {goto restore;}
            if (Stepper_WaitStopped(&hj->hs) != STEPPER_OK) {status = JOINT_TIMEOUT; goto restore;}
            if (Stepper_SetPosition(&hj->hs, JOINT_HOME_ANGLE * hj->gear_ratio) != STEPPER_OK) {goto restore;}
            hj->home  = JOINT_HOME_ANGLE;
            hj->homed = true;
            status = JOINT_OK;
            goto restore;
        }

        // First search failed; try the opposite direction
        if (attempt == 0) {mode = STEPPER_MODE_VEL_NEG; continue;}

        // Both directions searched without finding home
        status = stalled ? JOINT_ERR : JOINT_TIMEOUT;
        goto restore;
    }

    status = JOINT_ERR; // stalled in both directions

    restore:
        (void)Stepper_Stop(&hj->hs);
        (void)Stepper_WaitStopped(&hj->hs);
        if (status != JOINT_OK) {(void)Stepper_SetMode(&hj->hs, STEPPER_MODE_HOLD);}
        (void)Stepper_SetVelocity(&hj->hs, saved_vmax);
        (void)Stepper_SetAcceleration(&hj->hs, saved_amax);
        return status;
}

static Joint_Status_TypeDef Joint_SearchLimit(Joint_TypeDef *hj, float dir, float *lim)
{
    if (hj == NULL || lim == NULL) {return JOINT_BADARG;}
    if (!hj->homed) {return JOINT_NOT_HOMED;}
    if (dir != 1.0f && dir != -1.0f) {return JOINT_BADARG;}

    Joint_Status_TypeDef status = JOINT_ERR;

    // Saved velocity and acceleration for restoration once limit is found
    float saved_vmax = Stepper_GetVelocity(&hj->hs);
    float saved_amax = Stepper_GetAcceleration(&hj->hs);

    // Set temporary velocity and acceleration for homing task
    if (Stepper_SetVelocity(&hj->hs, JOINT_SETUP_VELOCITY * hj->gear_ratio) != STEPPER_OK) {goto restore;}
    if (Stepper_SetAcceleration(&hj->hs, JOINT_SETUP_ACCELERATION * hj->gear_ratio) != STEPPER_OK) {goto restore;}

    float search_angle = JOINT_LIMIT_SEARCH_ANGLE * dir;

    if (Stepper_MoveTo(&hj->hs, search_angle * hj->gear_ratio) != STEPPER_OK) {goto restore;}

    uint8_t stall_count = 0;
    uint32_t start = HAL_GetTick();
    bool stalled = false;
    bool arrived = false;

    while (HAL_GetTick() - start <= JOINT_SETUP_TIMEOUT_MS) {
        // Stall checks need a delay at motor startup
        // Stall is only detected after 3 detections
        // Stalled means there is a limit
        if (HAL_GetTick() - start >= JOINT_STALL_DETECT_DELAY_MS) {
            if (Stepper_IsStalled(&hj->hs)) {
                if (++stall_count >= JOINT_STALL_COUNT) {
                    stalled = true;
                    break;
                }
            }
            else {stall_count = 0;}
        }
        // Reaching the search angle means no limit
        if (Stepper_PositionReached(&hj->hs)) {arrived = true; break;}
        HAL_Delay(JOINT_SETUP_POLL_MS);
    }

    // Stop 
    if (Stepper_Stop(&hj->hs) != STEPPER_OK) {goto restore;}
    if (Stepper_WaitStopped(&hj->hs) != STEPPER_OK) {status = JOINT_TIMEOUT; goto restore;}

    if (stalled) {*lim = (Stepper_GetPosition(&hj->hs) / hj->gear_ratio) - (JOINT_LIMIT_MARGIN * dir); status = JOINT_OK;}
    else if (arrived) {status = JOINT_NOT_FOUND;}
    else {status = JOINT_TIMEOUT;}

    // Return home once search is completed normally
    if (Stepper_MoveTo(&hj->hs, hj->home * hj->gear_ratio) != STEPPER_OK) {status = JOINT_ERR; goto restore;}
    if (Stepper_WaitStopped(&hj->hs) != STEPPER_OK) {status = JOINT_TIMEOUT; goto restore;}

restore:
    (void)Stepper_SetVelocity(&hj->hs, saved_vmax);
    (void)Stepper_SetAcceleration(&hj->hs, saved_amax);
    
    return status;
}

Joint_Status_TypeDef Joint_Limit(Joint_TypeDef *hj)
{   
    if (hj == NULL) {return JOINT_BADARG;}
    if (!hj->homed) {return JOINT_NOT_HOMED;}

    hj->limited_min = false;
    hj->limited_max = false;

    Joint_Status_TypeDef status;

    // Search in the positive direction relative to home
    status = Joint_SearchLimit(hj, +1.0f, &hj->limit_max);
    if (status == JOINT_OK) {hj->limited_max = true;}
    else if (status != JOINT_NOT_FOUND) {return status;}

    // Search in the negative direction relative to home
    status = Joint_SearchLimit(hj, -1.0f, &hj->limit_min);
    if (status == JOINT_OK) {hj->limited_min = true;}
    else if (status != JOINT_NOT_FOUND) {return status;}

    return (hj->limited_min || hj->limited_max) ? JOINT_OK : JOINT_NOT_FOUND;
}

Joint_Status_TypeDef Joint_SetSpeed(Joint_TypeDef *hj, float speed)
{
    if (hj == NULL) {return JOINT_BADARG;}
    if (!isfinite(speed) || speed <= 0.0f) {return JOINT_BADARG;}

    float saved_vmax = Stepper_GetVelocity(&hj->hs);
    float amax = speed * JOINT_VEL_ACC_MULTIPLIER;

    // Velocity and Acceleration are dependent of each other
    if (Stepper_SetVelocity(&hj->hs, speed * hj->gear_ratio) != STEPPER_OK) {return JOINT_ERR;}
    if (Stepper_SetAcceleration(&hj->hs, amax * hj->gear_ratio) != STEPPER_OK) {
        (void)Stepper_SetVelocity(&hj->hs, saved_vmax);
        return JOINT_ERR;
    }

    return JOINT_OK;
}

// -----------------------------------------------------------------------------
// MOTION
// -----------------------------------------------------------------------------
Joint_Status_TypeDef Joint_MoveTo(Joint_TypeDef *hj, float deg)
{
    if (hj == NULL) {return JOINT_BADARG;}
    if (!hj->homed) {return JOINT_NOT_HOMED;}
    if (!isfinite(deg)) {return JOINT_BADARG;}

    if (hj->limited_min && deg < hj->limit_min) {return JOINT_OVER_LIMIT;}
    if (hj->limited_max && deg > hj->limit_max) {return JOINT_OVER_LIMIT;}

    if (Stepper_MoveTo(&hj->hs, deg * hj->gear_ratio) != STEPPER_OK) {return JOINT_ERR;}

    return JOINT_OK;
}

Joint_Status_TypeDef Joint_MoveBy(Joint_TypeDef *hj, float deg)
{
    if (hj == NULL) {return JOINT_BADARG;}
    if (!hj->homed) {return JOINT_NOT_HOMED;}
    if (!isfinite(deg)) {return JOINT_BADARG;}

    float cur_pos = Joint_GetPosition(hj);

    return Joint_MoveTo(hj, cur_pos + deg);
}

Joint_Status_TypeDef Joint_Stop(Joint_TypeDef *hj)
{
    if (hj == NULL) {return JOINT_BADARG;}

    if (Stepper_Stop(&hj->hs) != STEPPER_OK) {return JOINT_ERR;}

    return JOINT_OK;
}

Joint_Status_TypeDef Joint_WaitStopped(Joint_TypeDef *hj)
{
    if (hj == NULL) {return JOINT_BADARG;}

    switch (Stepper_WaitStopped(&hj->hs)) {
        case STEPPER_OK:      return JOINT_OK;
        case STEPPER_TIMEOUT: return JOINT_TIMEOUT;
        default:              return JOINT_ERR;
    }
}

Joint_Status_TypeDef Joint_Move(Joint_TypeDef *hj, float deg)
{
    Joint_Status_TypeDef status = Joint_MoveTo(hj, deg);
    if (status != JOINT_OK) {return status;}

    return Joint_WaitStopped(hj);
}

Joint_Status_TypeDef Joint_MoveHome(Joint_TypeDef *hj)
{
    Joint_Status_TypeDef status = Joint_MoveTo(hj, hj->home);
    if (status != JOINT_OK) {return status;}

    return Joint_WaitStopped(hj);
}

// -----------------------------------------------------------------------------
// STATUS and READBACK
// -----------------------------------------------------------------------------
float Joint_GetPosition(Joint_TypeDef *hj)
{
    if (hj == NULL) {return 0.0f;}

    return Stepper_GetPosition(&hj->hs) / hj->gear_ratio;
}

float Joint_GetSpeed(Joint_TypeDef *hj)
{
    if (hj == NULL) {return 0.0f;}

    return Stepper_GetVelocity(&hj->hs) / hj->gear_ratio;
}

bool Joint_IsMoving(Joint_TypeDef *hj)
{
    if (hj == NULL) {return false;}

    return Stepper_IsMoving(&hj->hs);
}

bool Joint_PositionReached(Joint_TypeDef *hj)
{
    if (hj == NULL) {return false;}

    return Stepper_PositionReached(&hj->hs);
}