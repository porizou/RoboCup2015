#include "MotorDriver.h"
#include "mbed.h"
MotorDriver::MotorDriver(PinName PWMPin, PinName directionPin1, PinName directionPin2) : 
 mPWM(PWMPin), mDir1(directionPin1), mDir2(directionPin2) {
    this->SetValue(0);
}
void MotorDriver::SetValue(float value) {
    if(value > 1.0)       value = 1.0;
    else if(value < -1.0) value = -1.0;
    
    this->mValue = value;
    
    this->mDir2 = 0; // Prevent driving 1 on both pins.
    this->mDir1 = value > 0;
    this->mDir2 = !this->mDir1;
    
    this->mPWM  = value;
}
MotorDriver& MotorDriver::operator=(float value) {
    this->SetValue(value);
    return *this;
}
float MotorDriver::GetValue() const {
    return this->mValue;
}
