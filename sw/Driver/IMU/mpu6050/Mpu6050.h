/**
 * @brief Bluethooth HC-06 Manager Device Library
 */
 
// Author: Gustavo Diaz

// Requiered Libraries
#include <Arduino.h>
#include <Wire.h>

//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG 57.2957
#define DEG_A_RAD 0.0174

class Mpu6050
{
    // Members
    /*float *angle_;
    float *accel_;
    float *gyro_;*/
    float ts_;

public:
    // Public Members
    float angle_[2];
    float accel_[2];
    float gyro_[2];

    // constructor de base (null)
    Mpu6050() {}

    // constructror parametrizado
    // Mpu6050():
    // {}

    // methods
    void init(void);
    void updateAngle(void);
    void readAccel(void);
    void readGyro(void);
    void complementaryFilter(void);

/*private:
    // methods
    
    void read(void);
    void write(int data);*/

};