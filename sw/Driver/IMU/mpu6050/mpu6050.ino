#include "Mpu6050.h"

Mpu6050 mpu;
// float *angle;

void setup()
{
	Serial.begin(9600);
	mpu.init();
}

void loop()
{
	// angle = mpu.getAngle();
	mpu.updateAngle();
	//Mostrar los valores por consola
	Serial.print(mpu.angle_[0]); Serial.print(";"); Serial.println(mpu.angle_[1]);

	delay(10); //Nuestra dt sera, pues, 0.010, que es el intervalo de tiempo en cada medida
}