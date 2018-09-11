/**
 * @brief Arduino Driver v1.0 to control and HDD brushless motor with an ESC
 */
 
/*Author: Gustavo Diaz*/

/*Requiered Libraries*/
#include "hdd_driver.h"

/*Output pins for ESC control*/
#define ESC_PWM_PIN_OUT 2
#define ESC_DIR_PIN_OUT 6

// SoftwareSerial Bluetooth(7,8);    // RX, TX Arduino PINs, conected to Bluetooth

/*Bluetooth Comunication Interface*/
// char command = 's';
// comInterface com_interface(&Bluetooth, &command, &Serial);

/*Device Control Handler*/
HddDriver hdd(ESC_PWM_PIN_OUT, ESC_DIR_PIN_OUT, &Serial);

int vel = 0;
int new_vel = 0;

void setup()
{
	/*Init Serial port*/
	Serial.begin(115200);
  // Bluetooth.begin(115200);

  //Init HDD
  hdd.init();
  Serial.println("HDD ready!");
}

void loop()
{
  /* test behavior */
  hdd.rotate(vel);

  if(Serial.available() >= 1)
  {
    new_vel = Serial.parseInt(); //Leer un entero por serial
    if(new_vel != 0 and new_vel != -1)
    {
      vel = new_vel;
      Serial.print("New speed set: ");
      Serial.println(vel);
    }
    else if (new_vel == -1)
    {
      vel = 0;
      hdd.idle();
      Serial.println("Motor Stoped");
    }
  }

  /* remote control (bluetooth) mode */
  // com_interface.getCommand(); // Serial.println(command);
  // cube1d.remoteControl(&command);
  // cube1d.sendVariables(&Bluetooth);
}