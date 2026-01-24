#ifndef PID_T_H
#define PID_T_H

#include <float.h>

// -----------------------------
// CONFIGURATION ENUMS
// -----------------------------

// Controller state
enum PIDMode { MANUAL, AUTOMATIC };

// Operating direction
enum PIDDirection { DIRECT, REVERSE };

// Calculation type
enum PIDType { ABSOLUTE, INCREMENTAL };

// Anti-overshoot style
enum PIDAntiOvershootStyle { CLASSIC, CUSTOM };

// Precision level
enum PIDPrecision { PRECISE, NORMAL };

// -----------------------------
// STRUCT FOR LOGS
// -----------------------------
struct PidLogs {
    // -----------------------------
    // TUNING CONSTANTS
    // -----------------------------
    double Kp;   // Proportional gain
    double Ki;   // Integral gain
    double Kd;   // Derivative gain

    // -----------------------------
    // SAMPLE TIME CONTROL
    // -----------------------------
    unsigned long sampleTime;       // Sampling interval
    unsigned long lastComputeTime;  // Last computation timestamp

    // -----------------------------
    // INTERNAL CALCULATION VARIABLES
    // -----------------------------
    double prevError;       // Previous error (for derivative)
    double integralTerm;    // Accumulated integral term

    // -----------------------------
    // LIMITS
    // -----------------------------
    double outputMin;       // Minimum output limit
    double outputMax;       // Maximum output limit
    double integralMin;     // Minimum integral limit
    double integralMax;     // Maximum integral limit

    // -----------------------------
    // PROCESS VARIABLES
    // -----------------------------
    double input;     // Measured value (feedback)
    double output;    // Calculated output
    double setpoint;  // Desired target

    // -----------------------------
    // MODE CONFIGURATIONS
    // -----------------------------
    PIDDirection direction;       // Direct or reverse
    PIDMode mode;                 // Manual or automatic
    PIDType type;                 // Absolute or incremental
    PIDAntiOvershootStyle antiOvershoot;   // Classic or custom anti-windup
    PIDPrecision precision;       // micros() or millis()
};

// -----------------------------
// MAIN CLASS PID_t
// -----------------------------
class PID_t {
private:
    // -----------------------------
    // TUNING CONSTANTS
    // -----------------------------
    double Kp;   // Proportional gain
    double Ki;   // Integral gain
    double Kd;   // Derivative gain

    // -----------------------------
    // SAMPLE TIME CONTROL
    // -----------------------------
    unsigned long sampleTime;       // Sampling interval (ms)
    unsigned long lastComputeTime;  // Last computation timestamp

    // -----------------------------
    // INTERNAL CALCULATION VARIABLES
    // -----------------------------
    double prevError;       // Previous error (for derivative)
    double integralTerm;    // Accumulated integral term

    // -----------------------------
    // LIMITS
    // -----------------------------
    double outputMin;       // Minimum output limit
    double outputMax;       // Maximum output limit
    double integralMin;     // Minimum integral limit
    double integralMax;     // Maximum integral limit

    // -----------------------------
    // PROCESS VARIABLES
    // -----------------------------
    double input;     // Measured value (feedback)
    double output;    // Calculated output
    double setpoint;  // Desired target

    // -----------------------------
    // MODE CONFIGURATIONS
    // -----------------------------
    PIDDirection direction;       // Direct or reverse
    PIDMode mode;                 // Manual or automatic
    PIDType type;                 // Absolute or incremental
    PIDAntiOvershootStyle antiOvershoot;   // Classic or custom anti-windup
    PIDPrecision precision;       // micros() or millis()

    // -----------------------------
    // INTERNAL HELPER FUNCTIONS
    // -----------------------------
    inline double computeIntegral(double err, double dt);   // Integral term calculation
    inline double computeDerivative(double err, double dt); // Derivative term calculation
    inline void applyLimits();                              // Apply output and integral limits

public:
    // -----------------------------
    // CONSTRUCTOR / DESTRUCTOR
    // -----------------------------
    PID_t();
    ~PID_t();

    // -----------------------------
    // MAIN FUNCTION
    // -----------------------------
    double compute();   // Execute PID calculation

    // -----------------------------
    // TUNINGS CONFIGURATION
    // -----------------------------
    void setTunings(double Kp_init, double Ki_init, double Kd_init);

    // -----------------------------
    // OPERATION CONFIGURATIONS
    // -----------------------------
    void setSampleTime(unsigned long newSampleTime);        // Sampling interval
    void setMode(PIDMode newMode);                          // Set mode
    void setType(PIDType newType);                          // Set type
    void setDirection(PIDDirection newDirection);           // Set direction
    void setAntiOvershootStyle(PIDAntiOvershootStyle newStyle);      // Set anti-windup style
    void setPrecision(PIDPrecision newPrecision);           // Set sampling precision
    void setOutputLimits(double minVal, double maxVal);     // Output limits
    void setIntegralLimits(double minVal, double maxVal);   // Integral limits
    void setSetpoint(double newSetpoint);                   // Set target
    void setInput(double newInput);                         // Set input
    double getOutput();                                     // Read current output

    // -----------------------------
    // UTILITIES
    // -----------------------------
    void reset();   // Reset internal variables

    // -----------------------------
    // LOGS
    // -----------------------------
    void getLogs(PidLogs &log);
};

// External utility function
void ensureOrder(double &a, double &b);

#endif



