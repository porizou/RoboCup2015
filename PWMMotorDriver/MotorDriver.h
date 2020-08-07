#include "mbed.h"
/** PWM/Direction Motor Driver
 *  Compatible with VNH3SP30 Motor Driver Carrier MD01B
 *
 * Example: 
 * @code
 * #include "MotorDriver.h"
 * MotorDriver motor(p21, p5, p6);
 * int main() {
 *     while(true) {
 *         motor = -0.5;
 *         wait(0.5); 
 *         motor = -1;
 *         wait(0.5);
 *         motor = -0.5;
 *         wait(0.5); 
 *         motor = 0;
 *         wait(0.5)
 *         motor = 0.5;
 *         wait(0.5)
 *         motor = 1;
 *         wait(0.5);
 *         motor = 0.5;
 *         wait(0.5);
 *         motor = 0;
 *         wait(0.5);
 *   }
 * }
 * @endcode
 * @author Michael Vartan
 */
class MotorDriver {
public:
    /**
     * Create a PWM/Direction motor on the specified pins.
     * @param   PwmPin          The pin which sends PWM Output to the motor driver
     * @param   directionPin1   The first of two direction pins to the motor driver
     * @param   directionPin2   The second of two direction pins to the motor driver
     */
    MotorDriver(PinName PWMPin, PinName directionPin1, PinName directionPin2);    
    
    /**
     * Sets the PWM duty cycle to the absolute value of your percentage, and the direction 
     * as the sign of your value.
     * @param   value   From -1.0 to 1.0, the percentage you want to run your motor at.
     */
    void SetValue(float value);  
    
    /**
     * Gets the last value assigned to the motor driver.
     *
     * @returns motor driver value
     */
    float GetValue() const;
    
    /**
     * Shorthand for GetValue
     */
    MotorDriver& operator=(float value);
    
private:
    PwmOut mPWM;
    DigitalOut mDir1;
    DigitalOut mDir2;
    float mValue;
};