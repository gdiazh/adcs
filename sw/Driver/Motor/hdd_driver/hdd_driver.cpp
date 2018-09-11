/**
 * @brief HDD motor Device Library
 */
 
/*Author: Gustavo Diaz*/

#include "hdd_driver.h"

void HddDriver::init_esc()
{
    esc_.writeMicroseconds(1000);   //1000[us] = 1[ms]
    delay(5000);           //necesary to init the ESC
}

int HddDriver::voltage_to_pwm(float voltage)
{
    float pwm = 58.8536*voltage*voltage-205.0240*abs(voltage)+1618.0829;
    int pwm_ms = (int) pwm;
    if (abs(voltage)>4.8)
        return 2000;
    else if (abs(voltage)<2)
        return 1000;
    else
        return pwm_ms;
}

uint8_t HddDriver::direction_handler(float voltage)
{
    if (voltage>=2.0) return 1;
    else if (voltage<=-2.0) return 0;
    else return 2;
}

//-------------------------- Public High Level Abstract Methods --------------------------
void HddDriver::init(void)
{
    /*Assign ouput pins*/
    // pinMode(esc_pwm_pin_, OUTPUT);
    // pinMode(esc_dir_pin_, OUTPUT);

    esc_.attach(esc_pwm_pin_);
    init_esc();

    /*initialize state variables*/
    state_vars_.hdd_angle = 0;       //[rad/s]
    state_vars_.hdd_vel = 0;         //[rad/s]

    // initialize output variables
    output_vars_.output_voltage = 0;     //[V]
    output_vars_.output_pwm = 1000;      //[us]
    output_vars_.output_dir = 0;         //[0:right, 1:left]
}

void HddDriver::idle()
{
    esc_.writeMicroseconds(1000);
}

void HddDriver::rotate(float voltage)
{
    output_vars_.output_voltage = voltage;
    output_vars_.output_pwm = voltage_to_pwm(output_vars_.output_voltage);
    output_vars_.output_dir = direction_handler(output_vars_.output_voltage);
    /*Update outputs*/
    // debug_port_->print("output_voltage: ");
    // debug_port_->println(output_vars_.output_voltage);
    // debug_port_->print("output_dir: ");
    // debug_port_->println(output_vars_.output_dir);
    if (output_vars_.output_pwm>=min_pwm_ && output_vars_.output_pwm<=max_pwm_) esc_.writeMicroseconds(output_vars_.output_pwm);
    if (output_vars_.output_dir == 1) digitalWrite(esc_dir_pin_, LOW);
    else if (output_vars_.output_dir == 0) digitalWrite(esc_dir_pin_, HIGH);
    // else digitalWrite(esc_dir_pin_, HIGH);
    // esc_.writeMicroseconds((int) velocity);// ---------------------------------------------------------------------------------TEST!!!!!!!!!!!!!!!!!!!!!!!!!-------------------------------------
}

/*void HddDriver::printVariables(void)
{
    debug_port_->print(state_vars_.hdd_angle*RAD_A_DEG);
    debug_port_->print(";");
    debug_port_->print(state_vars_.hdd_vel*RAD_A_DEG);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_voltage);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_pwm);
    debug_port_->print(";");
    debug_port_->print(output_vars_.output_dir);
    debug_port_->println(";");
}*/