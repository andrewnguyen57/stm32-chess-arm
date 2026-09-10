/**
 * @file    TMC5160_Registers.h
 * @brief   TMC5160 register address map (datasheet pg 32 to 49).
 */

#ifndef TMC5160_REGISTERS_H
#define TMC5160_REGISTERS_H

// --- REGISTERS ---

// -- Global config --
#define TMC5160_GCONF       0x00 // Global configuration flags
#define TMC5160_GSTAT       0x01 // Global status flags
#define TMC5160_IOIN        0x04 // Input pin states and IC version

// -- Current --
#define TMC5160_IHOLD_IRUN  0x10 // Run and hold current control
#define TMC5160_TPOWERDOWN  0x11 // Standstill-to-powerdown delay (0 to 4 s)
#define TMC5160_TPWMTHRS    0x13 // SpreadCycle/StealthChop crossover velocity
#define TMC5160_TCOOLTHRS   0x14 // CoolStep/StallGuard threshold velocity

// -- Ramp generator --
#define TMC5160_RAMPMODE    0x20 // Ramp mode select
#define TMC5160_XACTUAL     0x21 // Actual position (signed)
#define TMC5160_VACTUAL     0x22 // Actual velocity from ramp generator (signed)
#define TMC5160_VSTART      0x23 // Motor start velocity
#define TMC5160_A1          0x24 // Acceleration between VSTART and V1
#define TMC5160_V1          0x25 // First accel/decel threshold velocity
#define TMC5160_AMAX        0x26 // Acceleration between V1 and VMAX
#define TMC5160_VMAX        0x27 // Target velocity (positioning needs VMAX >= VSTART)
#define TMC5160_DMAX        0x28 // Deceleration between VMAX and V1
#define TMC5160_D1          0x2A // Deceleration between V1 and VSTOP
#define TMC5160_VSTOP       0x2B // Motor stop velocity
#define TMC5160_XTARGET     0x2D // Target position for position mode (signed)
#define TMC5160_RAMPSTAT    0x35 // Ramp & Reference Switch Status Register 

// -- Driver --
#define TMC5160_CHOPCONF    0x6C // Chopper and driver configuration
#define TMC5160_COOLCONF    0x6D // CoolStep and StallGuard2 configuration
#define TMC5160_DRV_STATUS  0x6F // StallGuard2 value and driver error flags
#define TMC5160_PWMCONF     0x70 // StealthChop voltage PWM configuration

// -- Protection and scaling --
#define TMC5160_SHORT_CONF   0x09 // Short-circuit protection configuration
#define TMC5160_DRV_CONF     0x0A // Driver configuration
#define TMC5160_GLOBAL_SCALER 0x0B // Global motor current scaling

#endif