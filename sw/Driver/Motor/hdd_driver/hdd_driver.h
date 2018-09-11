/**
 * @brief HDD motor Device Library
 */

/*Author: Gustavo Diaz*/

/*Requiered Libraries*/
#include <Arduino.h>
#include <Servo.h>

/*System behavior*/
#define SET_POINT 0
#define JUMPING_VEL 1550
#define JUMPING_DELAY 4000

// Motor parameters
#define Km 0.075
#define K_ESC 100
#define MAX_TORQUE 700

typedef struct StateVariables{
    float hdd_angle;
    float hdd_vel;
} StateVariables;

typedef struct OutputVariables{
    float output_voltage;
    float output_pwm;
    int output_dir;
} OutputVariables;

/**
 * @class HddDriver
 * @brief Class for manage HDD motor with an ESC.
 */

class HddDriver
{
    /*Members*/
    uint8_t esc_pwm_pin_;
    uint8_t esc_dir_pin_;
    uint16_t min_pwm_;
    uint16_t max_pwm_;

    Servo esc_;

    StateVariables state_vars_;
    OutputVariables output_vars_;

    HardwareSerial *debug_port_;

public:
    /*Public Members*/

    /*constructor de base (null)*/
    HddDriver() {}

    // constructror parametrizado
    HddDriver(uint8_t esc_pwm_pin, uint8_t esc_dir_pin, uint16_t min_pwm, uint16_t max_pwm, HardwareSerial *debug_port):
        esc_pwm_pin_(esc_pwm_pin),
        esc_dir_pin_(esc_dir_pin),
        min_pwm_(min_pwm),
        max_pwm_(max_pwm),
        debug_port_(debug_port)
    {
        // Config pins
        pinMode(esc_pwm_pin_, OUTPUT);
        pinMode(esc_dir_pin_, OUTPUT);
        // esc_.attach(esc_pwm_pin_);
    }

    // methods
    void init(void);
    void idle();
    void init_esc();
    void rotate(float voltage);
    // void printVariables(void);

private:
    // methods
    
    // Declaration of Output Handlers
    int voltage_to_pwm(float voltage);
    uint8_t direction_handler(float voltage);
};