/**
 * @file PID_params.h
 * @author Ian Gant (Igant01)
 * @brief PID parameter management for control loops
 * @version 0.1
 * @date 2026-03-17
 */

#ifndef PID_PARAMS_H
#define PID_PARAMS_H

#include <EEPROM.h>

// EEPROM Layout:
// 0-3:     Rear SAS Left calibration
// 2-3:     Rear SAS Right calibration
// 4:       Rear SAS valid flag
// 5-15:    Reserved
// 16-17:   Front SAS Left calibration
// 18-19:   Front SAS Right calibration
// 20:      Front SAS valid flag
// 21-23:   Reserved
// 24-31:   Torque PID: Kp (double)
// 32-39:   Torque PID: Ki (double)
// 40-47:   Torque PID: Kd (double)
// 48:      Torque PID valid flag
// 49-55:   Reserved
// 56-63:   Velocity PID: Kp (double)
// 64-71:   Velocity PID: Ki (double)
// 72-79:   Velocity PID: Kd (double)
// 80:      Velocity PID valid flag
// 81-87:   Reserved
// 88-95:   Position PID: Kp (double)
// 96-103:  Position PID: Ki (double)
// 104-111: Position PID: Kd (double)
// 112:     Position PID valid flag
// 113-119: Reserved

struct PIDParameters {
    double Kp;
    double Ki;
    double Kd;
    
    PIDParameters() : Kp(0.0), Ki(0.0), Kd(0.0) {}
    PIDParameters(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}
};

class PIDManager {
public:
    // Default PID values
    static const PIDParameters DEFAULT_TORQUE_PID;
    static const PIDParameters DEFAULT_VELOCITY_PID;
    static const PIDParameters DEFAULT_POSITION_PID;
    
    // Torque PID (loop 1)
    static PIDParameters torquePID;
    
    // Velocity PID (loop 2)
    static PIDParameters velocityPID;
    
    // Position PID (loop 3)
    static PIDParameters positionPID;
    
    // Functions to load/save from EEPROM
    static void loadTorquePID();
    static void saveTorquePID();
    
    static void loadVelocityPID();
    static void saveVelocityPID();
    
    static void loadPositionPID();
    static void savePositionPID();
    
    static void loadAllPID();
    static void saveAllPID();
    
    static bool torquePIDValid();
    static bool velocityPIDValid();
    static bool positionPIDValid();
    
    // Get/Set methods
    static void setTorquePID(double kp, double ki, double kd);
    static void setVelocityPID(double kp, double ki, double kd);
    static void setPositionPID(double kp, double ki, double kd);
};

#endif
