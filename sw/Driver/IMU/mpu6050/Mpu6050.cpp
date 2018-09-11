/**
 * @brief MPU6050
 */
 
// Author: Gustavo Diaz

// Requiered Libraries
#include "Mpu6050.h"

void Mpu6050::init(void)
{
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    angle_[0] = 0;
    angle_[1] = 0;
    accel_[0] = 0;
    accel_[1] = 0;
    gyro_[0] = 0;
    gyro_[1] = 0;
    ts_ = 0.01;
}

void Mpu6050::updateAngle(void)
{
    readAccel();
    readGyro();
    complementaryFilter();    //aplica filtro y guarda valor en variable "angle_"
    delay(1);
    // return angle_;
}

void Mpu6050::readAccel(void)
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
    int16_t AcX=Wire.read()<<8|Wire.read(); //Cada valor esta en 2 registros en el MPU
    int16_t AcY=Wire.read()<<8|Wire.read();
    int16_t AcZ=Wire.read()<<8|Wire.read();

    //A partir de los valores del acelerometro, se calculan los angulos Y, X
    //respectivamente, con la formula de la tangente.
    // float accel[2];
    accel_[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_A_DEG;
    accel_[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_A_DEG;
    // return accel;
}

void Mpu6050::readGyro(void)
{
    //Leer los valores del Giroscopio
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
    int16_t GyX=Wire.read()<<8|Wire.read();
    int16_t GyY=Wire.read()<<8|Wire.read();

    //Calculo del angulo del Giroscopio
    // float gyro[2];
    gyro_[0] = GyX/G_R;
    gyro_[1] = GyY/G_R;
}

void Mpu6050::complementaryFilter(void)
{
    //Aplicar el Filtro Complementario
    angle_[0] = 0.98 *(angle_[0]+gyro_[0]*ts_) + 0.02*accel_[0];
    angle_[1] = 0.98 *(angle_[1]+gyro_[1]*ts_) + 0.02*accel_[1];
}