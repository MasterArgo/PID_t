/****************************************************************************************
 * PIDController Library - Version 2.2.2
 * Language: C++
 * Author: Ícaro Razera (icarorazera@gmail.com)
 * Target platform: Arduino IDE
 *
 * Description:
 *      Implementation of a flexible and educational PID controller.
 *          - Supports modes: MANUAL / AUTOMATIC
 *          - Direction: DIRECT / REVERSE
 *          - Type: ABSOLUTE / INCREMENTAL
 *          - Anti-overshoot style: CLASSIC / CUSTOM
 *          - Precision: PRECISE / NORMAL
 *
 * History:
 *  v1.0.0 - First version in C language
 *  v2.0.0 - Rewritten in C++ with support for anti-windup styles
 *  v2.0.1 - Reorganization of the compute() method and inline call of internal helpers.
 *           The compute function now returns the output.
 *  v2.1.2 - Added a more precise mode using micros(). Optimization of the compute() method.
 *  v2.2.2 - Added a structure for collecting logs.
 *  v2.3.2 - Translation to English.
 *
 * License:
 *  This code is licensed under the MIT License.
 *  See the LICENSE file for more details.
 ****************************************************************************************/

#include "PIDController.h"
#include <Arduino.h>

/*----------------------------------------------------------------------------------------
 * PIDController()
 *
 * Constructor of the PIDController class.
 *
 * Default initialization:
 *      - Execution interval: 200 ms
 *      - Mode: AUTOMATIC (controller starts active)
 *      - Constants Kp, Ki, Kd: initialized to 0.0
 *      - Direction: DIRECT
 *      - Type: INCREMENTAL
 *      - Anti-windup style: CUSTOM
 *      - Output limits: [-FLT_MAX, FLT_MAX]
 *      - Integral limits: [-FLT_MAX, FLT_MAX] (no initial restriction)
 *
 * Notes:
 *      - After creation, the user must configure the constants Kp, Ki, Kd
 *        using setTunings().
 *      - Other variables (setpoint, input, limits, style, etc.) can be
 *        adjusted as needed in setup().
 *---------------------------------------------------------------------------------------*/

PIDController::PIDController()
    : Kp(0), Ki(0), Kd(0),
      sampleTime(200),
      prevError(0),
      integralTerm(0),
      outputMin(-FLT_MAX),
      outputMax(FLT_MAX),
      direction(DIRECT),
      mode(AUTOMATIC),
      type(INCREMENTAL),
      precision(NORMAL),
      lastComputeTime(precision == PRECISE ? micros() : millis()),
      antiOvershoot(CUSTOM),
      integralMin(-FLT_MAX),
      integralMax(FLT_MAX) {}


/*----------------------------------------------------------------------------------------
 * ~PIDController()
 *
 * Destructor of the PIDController class.
 *
 * Behavior:
 *      - Currently does not perform any specific action, since the class does not
 *      allocate dynamic resources (such as memory via new or pointers).
 *      - It is kept to ensure interface consistency and allow future expansions,
 *      in case it becomes necessary to release external resources or perform
 *      some cleanup.
 *
 * Notes:
 *      - In simple Arduino applications, there is usually no need for logic in
 *        the destructor, since objects are automatically deallocated at the end
 *        of execution.
 *      - The explicit presence of the destructor makes maintenance easier and
 *        clarifies that there are no pending release tasks.
 *---------------------------------------------------------------------------------------*/

PIDController::~PIDController(){
    // Por enquanto, nada...
}


/*----------------------------------------------------------------------------------------
 * compute()
 *
 * Main function of the PID controller.
 * Executes the output calculation based on the defined setpoint and input.
 *
 * Flow:
 *  1. Checks if the mode is AUTOMATIC.
 *  2. Calculates the error (setpoint - input).
 *  3. Evaluates the elapsed time since the last execution.
 *  4. If the defined minimum time has passed:
 *      - Converts time to seconds.
 *      - Inverts the error if the direction is REVERSE.
 *      - Calculates the proportional, integral, and derivative terms.
 *      - Updates the output:
 *          • ABSOLUTE: replaces with the newly calculated output.
 *          • INCREMENTAL: adds the calculated variation to the previous output.
 *      - Updates the last error for future use.
 *      - Applies output and integral limits (anti-windup).
 *
 * Notes:
 *  - The function only operates in AUTOMATIC mode.
 *  - The execution interval is controlled by DefIntervalo().
 *  - The direction (DIRECT/REVERSE) defines whether the error increases or decreases the output.
 *  - The limits ensure that the output and the integral do not exceed valid values.
 *
 * Return:
 *  - Returns the Output.
 *---------------------------------------------------------------------------------------*/

double PIDController::compute() {
    if (mode == AUTOMATIC) {
        unsigned long now = (precision == PRECISE ? micros() : millis());
        unsigned long elapsedTime = now - lastComputeTime;

        if (elapsedTime >= sampleTime) {
            double outputAux;
            double err = setpoint - input;

            lastComputeTime = now;

            // PID calculation
            outputAux = Kp * err + Ki * computeIntegral(err, elapsedTime) + Kd * computeDerivative(err, elapsedTime);

            output = (type == ABSOLUTE) ? outputAux : output + outputAux;

            // Update error
            prevError = err;

            // Apply limits
            applyLimits();
        }
    }

    return output;
}


/*----------------------------------------------------------------------------------------
 * setTunings(...)
 *
 * Defines the gains of the PID controller:
 *      - Kp: proportional gain
 *      - Ki: integral gain
 *      - Kd: derivative gain
 *
 * Parameters:
 *      Kp_init - proportional constant
 *      Ki_init - integral constant
 *      Kd_init - derivative constant
 *
 * Behavior:
 *      - These values determine the intensity of the controller's response.
 *      - Kp controls the immediate reaction to the error.
 *      - Ki accumulates the error over time (corrects offset).
 *      - Kd reacts to the rate of change of the error (smooths oscillations).
 *      - The function only stores the values; the calculation is performed in Compute().
 *
 * Example of use:
 *      MyPID.setTunings(1.0, 2.0, 0.1);
 *
 * Note:
 *      Proper tuning of Kp, Ki, and Kd is essential for stability.
 *      Values that are too high may cause oscillation; values that are too low may make the system slow.
 *---------------------------------------------------------------------------------------*/

void PIDController::setTunings(double Kp_init, double Ki_init, double Kd_init) {
    constexpr double ADJUST_MICROS = 1000000.0;
    constexpr double ADJUST_MILLIS = 1000.0;
    double adjust = (precision == PRECISE ? ADJUST_MICROS : ADJUST_MILLIS);

    Kp = Kp_init;
    // Constants are adjusted at initialization so that the conversion
    // from ms to s does not need to be performed at each compute() execution
    Ki = Ki_init / adjust;
    Kd = Kd_init * adjust;

    if (direction == REVERSE) {
        Kp = -Kp;
        Ki = -Ki;
        Kd = -Kd;
    }

    return;
}


/*----------------------------------------------------------------------------------------
 * setSampleTime(...)
 *
 * Defines the minimum time interval (in milliseconds) between each execution of the PID calculation.
 *
 * Parameters:
 *      newSampleTime - value in milliseconds representing the controller's sampling period.
 *
 * Behavior:
 *      - The value stored in 'tempo' will be used inside the Compute() function.
 *      - Compute() will only execute the calculation if the elapsed time since the last execution
 *        is greater than or equal to 'tempo'.
 *      - This prevents the PID from being recalculated at very short intervals, ensuring
 *        stability and reducing CPU overhead.
 *
 * Example of use:
 *      MyPID.setSampleTime(100);   // Executes the PID every 100 ms
 *
 * Note:
 *      The chosen value must be compatible with the dynamics of the physical system.
 *      Very small intervals may generate noise; very large intervals may
 *      make the control sluggish.
 *---------------------------------------------------------------------------------------*/

void PIDController::setSampleTime(unsigned long newSampleTime) {
    sampleTime = newSampleTime;

    return;
}


/*----------------------------------------------------------------------------------------
 * setMode(...)
 *
 * Switches the operating mode of the PID controller.
 *
 * Parameters:
 *      newNode - can be AUTOMATIC or MANUAL
 *
 * Behavior:
 *      - AUTOMATIC: the controller executes the PID calculations normally within the Compute() function.
 *      - MANUAL: the controller is paused; Compute() does not change the output.
 *                In this mode, the user can manually set the value of 'output'.
 *
 * Example of use:
 *      pid.setMode(MANUAL);      // pauses the PID and allows manual control of the output
 *      pid.setMode(AUTOMATIC);   // enables the PID to automatically calculate the output
 *
 * Notes:
 *      - Useful for starting the system in MANUAL and only later enabling the PID in AUTOMATIC.
 *      - Can also be used for testing or situations where automatic control
 *        needs to be temporarily disabled.
 *---------------------------------------------------------------------------------------*/

void PIDController::setMode(PIDMode newMode) {
    mode = newMode;

    return;
}


/*----------------------------------------------------------------------------------------
 * setType(...)
 *
 * Defines the calculation type of the PID controller.
 *
 * Parameters:
 *   newType - can be ABSOLUTE or INCREMENTAL
 *
 * Behavior:
 *      - ABSOLUTE: the calculated output directly replaces the previous value.
 *                   Example: output = newValue;
 *      - INCREMENTAL: the calculated output is added to the previous value,
 *                     epresenting only the required variation.
 *                   Example: output = output + delta;
 *
 * Example of use:
 *      pid.setType(ABSOLUTE);     // output always recalculated from zero
 *      pid.setType(INCREMENTAL);  // output adjusted incrementally
 *
 * Notes:
 *      - The ABSOLUTE mode is more intuitive and common in simple systems.
 *      - The INCREMENTAL mode can be useful in discrete systems or when only
 *        relative corrections are desired.
 *---------------------------------------------------------------------------------------*/

void PIDController::setType(PIDType newType) {
    type = newType;

    return;
}


/*----------------------------------------------------------------------------------------
 * setDirection(...)
 *
 * Defines the action direction of the PID controller.
 *
 * Parameters:
 *      newDirection - can be DIRECT or REVERSE
 *
 * Behavior:
 *      - DIRECT: a positive error increases the output.
 *      - REVERSE: a positive error decreases the output (the error is internally inverted).
 *
 * Example of use:
 *      pid.setDirection(DIRECT);   // output increases when input < setpoint
 *      pid.setDirection(REVERSE);  // output decreases when input < setpoint
 *
 * Notes:
 *      - Useful for adapting the controller to the physical system.
 *      - Example: in a heater, DIRECT increases power when the temperature
 *        is below the setpoint. In a cooling system, it may be necessary
 *        to use REVERSE.
 *---------------------------------------------------------------------------------------*/

void PIDController::setDirection(PIDDirection newDirection) {
    direction = newDirection;

    return;
}


/*----------------------------------------------------------------------------------------
 * setAntiOvershootStyle(...)
 *
 * Defines the anti-windup style (technique to prevent overshoot caused by integral saturation).
 *
 * Parameters:
 *      newStyle - can be CLASSIC or CUSTOM
 *
 * Behavior:
 *      - CLASSIC: applies the traditional anti-windup method, limiting the integral in a simple way.
 *      - CUSTOM: allows greater flexibility, applying user-defined limits
 *                via DefIntegralLimits().
 *
 * Example of use:
 *      pid.setAntiOvershootStyle(CLASSIC);       // uses standard anti-windup
 *      pid.setAntiOvershootStyle(CUSTOM);        // uses user-defined limits
 *
 * Notes:
 *      - The CLASSIC style is sufficient for most systems.
 *      - The CUSTOM style is useful when more precise control of the integral
 *        behavior is desired, avoiding overshoot or sluggish response.
 *---------------------------------------------------------------------------------------*/

void PIDController::setAntiOvershootStyle(PIDAntiOvershootStyle newStyle) {
    antiOvershoot = newStyle;

    return;
}


/*----------------------------------------------------------------------------------------
 * setPrecision(...)
 *
 * Defines the sampling precision.
 *
 * Parameters:
 *      newPrecision - can be NORMAL or PRECISE
 *
 * Behavior:
 *      - NORMAL: uses the millis() function for sampling, which returns time with
 *                1 millisecond precision.
 *      - PRECISE: uses the micros() function for sampling, which returns time with
 *                 4 microseconds precision.
 *
 * Example of use:
 *      pid.setPrecision(NORMAL);
 *      pid.setPrecision(PRECISE);
 *
 * Notes:
 *      - The NORMAL style is sufficient for most systems.
 *      - The PRECISE style is useful when tighter control of PID execution timing
 *        is required, ensuring regular calculation intervals.
 *        On ARM-based microcontrollers, such as the ESP32, the use of micros()
 *        is efficient and provides higher temporal resolution without significant penalty.
 *        On classic Arduino boards (AVR), however, the use of micros() is more costly,
 *        so millis() is recommended when 1 ms resolution is sufficient.
 *        Therefore, the PRECISE style should only be chosen when the application
 *        demands high temporal accuracy, such as in motor control or fast measurements.
 *---------------------------------------------------------------------------------------*/

void PIDController::setPrecision(PIDPrecision newPrecision) {
    precision = newPrecision;

    return;
}


/*----------------------------------------------------------------------------------------
 * setOutputLimits(...)
 *
 * Defines the minimum and maximum limits of the PID controller output.
 *
 * Parameters:
 *      minVal - minimum allowed output value
 *      maxVal - maximum allowed output value
 *
 * Behavior:
 *      - The values are stored in 'minimum' and 'maximum'.
 *      - The ensureOrder(minVal, maxVal) function ensures that the smaller value is stored
 *        in 'minimum' and the larger in 'maximum', even if passed in reversed order.
 *      - During the calculation in Compute(), the output is constrained within this range.
 *
 * Example of use:
 *      pid.setOutputLimits(0, 255);   // output limited between 0 and 255
 *
 * Notes:
 *      - Useful for systems with physical restrictions (e.g., PWM from 0–255).
 *      - Prevents saturation or invalid output values.
 *---------------------------------------------------------------------------------------*/

void PIDController::setOutputLimits(double minVal, double maxVal) {
    ensureOrder(minVal, maxVal);

    outputMin = minVal;
    outputMax = maxVal;

    return;
}


/*----------------------------------------------------------------------------------------
 * setIntegralLimits(...)
 *
 * Defines the minimum and maximum limits for the integral term of the PID controller.
 *
 * Parameters:
 *      minVal - lower limit of the integral
 *      maxVal - upper limit of the integral
 *
 * Behavior:
 *      - The values are stored in 'integral_min' and 'integral_max'.
 *      - The ensureOrder(minVal, maxVal) function ensures that the smaller value is stored
 *        in 'integralMin' and the larger in 'integralMax'.
 *      - During the calculation, the sum of error * dt is constrained within this range
 *        to prevent integral saturation (anti-windup).
 *
 * Example of use:
 *      pid.setIntegralLimits(-100, 100);   // integral limited between -100 and 100
 *
 * Notes:
 *      - Essential to prevent the integral term from growing indefinitely.
 *      - Overly restrictive values may reduce the corrective action of the integral.
 *---------------------------------------------------------------------------------------*/

void PIDController::setIntegralLimits(double minVal, double maxVal) {
    ensureOrder(minVal, maxVal);

    integralMax = maxVal;
    integralMin = minVal;

    return;
}


/*----------------------------------------------------------------------------------------
 * setSetpoint(...)
 *
 * Defines the target value (setpoint) of the PID controller.
 *
 * Parameters:
 *      newSetpoint - desired value for the controlled process
 *
 * Behavior:
 *      - The value is stored in 'setpoint'.
 *      - During the calculation in compute(), the error is obtained as (setpoint - input).
 *
 * Example of use:
 *      pid.setSetpoint(100.0);   // sets target to 100 units
 *
 * Notes:
 *      - The setpoint can be dynamically changed during execution.
 *      - It is the value that the system will attempt to reach and maintain.
 *---------------------------------------------------------------------------------------*/

void PIDController::setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;

    return;
}


/*----------------------------------------------------------------------------------------
 * setInput(...)
 *
 * Defines the input value (feedback) of the PID controller.
 *
 * Parameters:
 *      newInput - measured value of the process
 *
 * Behavior:
 *      - The value is stored in 'input'.
 *      - During the calculation in compute(), the error is obtained as (setpoint - input).
 *
 * Example of use:
 *      pid.setInput(sensorValue);   // sets input from sensor reading
 *
 * Notes:
 *      - Must be updated every cycle with the actual system value.
 *      - Represents the controlled variable (e.g., temperature, speed, position).
 *---------------------------------------------------------------------------------------*/

void PIDController::setInput(double newInput) {
    input = newInput;

    return;
}


/*----------------------------------------------------------------------------------------
 * getOutput()
 *
 * Returns the current output value calculated by the PID controller.
 *
 * Return:
 *      output - output value after the last calculation in compute()
 *
 * Behavior:
 *      - The value stored in 'output' is returned.
 *      - This value should be applied to the actuator (e.g., PWM, motor, valve).
 *
 * Example of use:
 *      double outputVal = pid.getOutput();
 *      analogWrite(pinPWM, outputVal);
 *
 * Notes:
 *      - getOutput() does not perform any recalculation; it only returns the last value.
 *      - To update the output, compute() must be called beforehand.
 *---------------------------------------------------------------------------------------*/

double PIDController::getOutput() {

    return output;
}


/*----------------------------------------------------------------------------------------
 * reset(...)
 *
 * Resets the integral accumulator (integral_error_sum).
 *
 * Behavior:
 *      - Clears any value accumulated in the integral term.
 *      - Useful to prevent saturation (windup) when the system changes state
 *        or when the controller needs to be restarted.
 *
 * Example of use:
 *      pid.reset();   // clears the integral term
 *
 * Notes:
 *      - Can be called in situations such as system reset or setpoint change.
 *      - Does not affect the proportional or derivative terms.
 *---------------------------------------------------------------------------------------*/

void PIDController::reset() {
    integralTerm = 0;

    return;
}


/*----------------------------------------------------------------------------------------
 * applyLimits(...)
 *
 * Applies the limits defined for the PID controller output.
 *
 * Behavior:
 *      - If 'output' is less than 'minimum', it is set to 'minimum'.
 *      - If 'output' is greater than 'maximum', it is set to 'maximum'.
 *      - Ensures that the output is always within the allowed range.
 *
 * Example of use:
 *      pid.setOutputLimits(0, 255);
 *      pid.compute();
 *      pid.applyLimits();   // output will remain between 0 and 255
 *
 * Notes:
 *      - Automatically called inside Compute().
 *      - Prevents invalid values or actuator saturation.
 *---------------------------------------------------------------------------------------*/

inline void PIDController::applyLimits() {
    if (output < outputMin) {
        output = outputMin;
    } else if (output > outputMax) {
        output = outputMax;
    }

    return;
}


/*----------------------------------------------------------------------------------------
 * computeIntegral(...)
 *
 * Calculates the integral term of the PID controller.
 *
 * Parameters:
 *      err - current error (setpoint - input)
 *      dt  - elapsed time in seconds
 *
 * Behavior:
 *      - If the error changes sign (crosses the setpoint) and the style is CUSTOM,
 *        the integral accumulator is reset (anti-overshoot).
 *      - If the output is already saturated (at maximum or minimum), the integral
 *        is not accumulated to prevent windup.
 *      - Otherwise, accumulates err * dt into integral_error_sum.
 *      - Applies limits defined in integral_min and integral_max.
 *
 * Notes:
 *      - In low-noise systems, it improves the response.
 *      - In systems with oscillating setpoints, it may reduce smoothness.
 *---------------------------------------------------------------------------------------*/

inline double PIDController::computeIntegral(double err, double dt) {
    // Prevents overshoot in some types of systems
    // In low-noise systems, the response is cleaner
    // On the other hand, in systems with setpoint oscillation,
    // this may reduce the smoothness of the response
    if (antiOvershoot == CUSTOM && err * prevError <= 0) {
        integralTerm = 0;
    }

    // If output is saturated, do not accumulate integral
    if (output >= outputMax && err > 0) {
        // already at maximum and error is positive -> do not accumulate
        return integralTerm;
    } else if (output <= outputMin && err < 0) {
        // already at minimum and error is negative -> do not accumulate
        return integralTerm;
    }

    integralTerm += err * dt;

    // Limit the integral sum
    if (integralTerm > integralMax) {
        integralTerm = integralMax;
    } else if (integralTerm < integralMin) {
        integralTerm = integralMin;
    }

    return integralTerm;
}


/*----------------------------------------------------------------------------------------
 * computeDerivative(...)
 *
 * Calculates the derivative term of the PID controller.
 *
 * Parameters:
 *      err - current error (setpoint - input)
 *      dt  - elapsed time in seconds
 *
 * Behavior:
 *      - If dt == 0, returns 0 (avoids division by zero).
 *      - Otherwise, calculates the error variation:
 *       (err - last_error) / dt
 *
 * Notes:
 *      - The derivative term helps to smooth oscillations.
 *      - It may amplify noise if the system has unstable measurements.
 *---------------------------------------------------------------------------------------*/

inline double PIDController::computeDerivative(double err, double dt) {
    if (dt == 0) {
        return 0;
    }

    return (err - prevError) / dt;
}


/*----------------------------------------------------------------------------------------
 * getLogs(...)
 *
 * Retrieves the current state and configuration of the PID controller.
 *
 * Parameters:
 *      log - reference to a PidLogs structure where all values will be stored
 *
 * Behavior:
 *      - Copies internal variables of the PID controller into the provided log structure.
 *      - Includes configuration parameters (Kp, Ki, Kd, mode, type, precision, direction).
 *      - Includes operational values (input, output, setpoint, integralTerm, prevError).
 *      - Includes limits (outputMin, outputMax, integralMin, integralMax).
 *      - Includes timing information (sampleTime, lastComputeTime).
 *      - Also stores antiOvershoot style currently in use.
 *
 * Example of use:
 *      PidLogs log;
 *      pid.getLogs(log);   // retrieves all PID internal data
 *
 * Notes:
 *      - Useful for debugging or monitoring the PID controller.
 *      - Provides a snapshot of both configuration and runtime values.
 *---------------------------------------------------------------------------------------*/

void PIDController::getLogs(PidLogs &log) {
    log.antiOvershoot = antiOvershoot;
    log.input = input;
    log.integralMax = integralMax;
    log.integralMin = integralMin;
    log.Kd = Kd;
    log.Ki = Ki;
    log.Kp = Kp;
    log.outputMax = outputMax;
    log.outputMin = outputMin;
    log.mode = mode;
    log.output = output;
    log.precision = precision;
    log.direction = direction;
    log.setpoint = setpoint;
    log.integralTerm = integralTerm;
    log.sampleTime = sampleTime;
    log.type = type;
    log.prevError = prevError;
    log.lastComputeTime = lastComputeTime;

    return;
}


/*----------------------------------------------------------------------------------------
 * ensureOrder(...)
 *
 * Auxiliary function to ensure that two values are in ascending order.
 *
 * Parameters:
 *      a - first value
 *      b - second value
 *
 * Behavior:
 *      - If 'a' is greater than 'b', the values are swapped.
 *      - Used in limit functions to guarantee that the smaller value is stored
 *        as minimum and the larger as maximum.
 *
 * Example of use:
 *      double min = 10, max = 5;
 *      swap(min, max);   // now min = 5, max = 10
 *
 * Notes:
 *      - Simple utility.
 *      - Prevents configuration errors when limits are passed in reversed order.
 *---------------------------------------------------------------------------------------*/

void ensureOrder(double &a, double &b){
    if(a > b){
        double temp = a;
        a = b;
        b = temp;
    }

    return;
}

// End
