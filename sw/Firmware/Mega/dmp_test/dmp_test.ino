/**
 * @brief Simple IMU Example
 */
 
/*Author: Gustavo Diaz*/

#include "imu.h"
#include "serialCom.h"


/*Pin definitions*/
#define INTERRUPT_PIN 31

/*Object Definitions*/
IMU imu(INTERRUPT_PIN, &Serial);
serialCom console;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    Serial.begin(115200);
    // initialize IMU
    imu.init();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    imu.updateData();
    // DEBUG2_PRINT("ypr:\t");
    // DEBUG2_PRINT(imu.ypr[0]*57.29);DEBUG2_PRINT("\t");
    // DEBUG2_PRINT(imu.ypr[1]*57.29);DEBUG2_PRINT("\t");
    // DEBUG2_PRINTLN(imu.ypr[2]*57.29);
    // delay(1000); //for testing worst case
    console.send_data(1, millis(), imu.ws.x, imu.ws.y, imu.ws.z);
    console.send_data(2, millis(), imu.gx_raw, imu.gy_raw, imu.gz_raw);
    // console.send_data(2, imu.q.x, imu.q.y, imu.q.z, imu.q.w);
    // console.send_data(2, 5, ypr[0]*57.2958, ypr[1]*57.2958, ypr[2]*57.2958);
}