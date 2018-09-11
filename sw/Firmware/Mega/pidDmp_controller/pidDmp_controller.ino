/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <PID_v1.h>

/**
 * @brief Arduino Driver for Satellite Attitude Control
   @Author: Gustavo Diaz
 */
 
/*This code use DMP example for the Attitude Determination*/

/*Requiered Libraries*/
#include "hdd_driver.h"
// #include <TimerThree.h>
// #include <TimerOne.h>

#include <Filters.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 3  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===               FRAME TEST PARAMS                          ===
// ================================================================

#define SET_VOLTAGE 1
#define SET_TORQUE 2
#define SET_SPEED 3
#define KEEP_ATTITUDE 4
#define SET_MODE 5
#define STOP 6
#define USE_CURRENT_SETPOINT 7
#define INCREASE_SETPOINT 8
#define DECREASE_SETPOINT 9
#define PRINT_SPEED 10
#define AUTOMATIC_MODE 11
#define STOP_X 12
#define STOP_Y 13
#define STOP_Z 14
#define SET_CONTROL_MODE 15
#define CHANGE_CURRENT_GAINS 16
#define CHANGE_SPEED_GAINS 17

// ================================================================
// ===               Actuators params                           ===
// ================================================================

/*Output pins for ESC control*/
#define ESC_PWM_PIN_OUT 7
#define ESC_DIR_PIN_OUT 6

#define ESC2_PWM_PIN_OUT 4
#define ESC2_DIR_PIN_OUT A14

#define ESC3_PWM_PIN_OUT 5
#define ESC3_DIR_PIN_OUT A15

#define HDD_ZERO_SPEED 1000
#define MIN_VOLTAGE 0.0

#define MOTOR_STOPPED 0
#define MOTOR_HOR 1
#define MOTOR_ANT 2

/*Device Control Handler*/
HddDriver hddx(ESC_PWM_PIN_OUT, ESC_DIR_PIN_OUT, 1000, 2000, &Serial);
HddDriver hddy(ESC2_PWM_PIN_OUT, ESC2_DIR_PIN_OUT, 1000, 2000, &Serial);
HddDriver hddz(ESC3_PWM_PIN_OUT, ESC3_DIR_PIN_OUT, 1000, 2000, &Serial);

float cmdVoltageMx = MIN_VOLTAGE;
float lastIStpt = MIN_VOLTAGE;
int8_t currentxSign = 1;
float cmdTorqueMx = 0;
float cmdSpeedMx = HDD_ZERO_SPEED;
float cmdyawMx = 0;
uint8_t motorx_state = 0;
uint8_t motorx_ref_dir_hasChange = 1;

float cmdVoltageMy = MIN_VOLTAGE;
float cmdTorqueMy = 0;
float cmdSpeedMy = HDD_ZERO_SPEED;

float cmdVoltageMz = MIN_VOLTAGE;

#define TORQUE_MODE 0
#define SPEED_MODE 1
#define POS_MODE 2
#define OPENLOOP_MODE 0
#define CLOSELOOP_MODE 1

uint8_t operationMode = OPENLOOP_MODE;
uint8_t controlMode = TORQUE_MODE;

// ================================================================
// ===               Sensor PARAMS                              ===
// ================================================================

#define HALL1 2
// #define HALL2 18
// #define HALL3 19

unsigned long time_ref = 0;
volatile uint8_t steps = 0;
float speed_rpm = 0.0;
float filtered_speed = 0.0;
float speedFilterFrecuency = 0.7; //[Hz]
FilterOnePole lowpassFilter(LOWPASS, speedFilterFrecuency);
float filtered_speed_calib = 0.0;

// ================================================================
// ===               Satellite speed                            ===
// ================================================================
unsigned long time_ref_pitch = 0;
float satellite_speed = 0.0;
float last_yaw = 0.0;
float last_ypr0 = 0.0;

float sateliteFiltered_speed = 0.0;
FilterOnePole satelliteSpeedFilter(LOWPASS, 0.2);

// ================================================================
// ===       PID Yaw Controller PARAMS                          ===
// ================================================================
double yawSetpointMx, yawInputMx, yawControlTorqueMx, yawControlTorqueMx2;
double Kp_yaw=0.6, Ki_yaw=0.00, Kd_yaw=0.0;
PID yawControllerMx(&yawInputMx, &yawControlTorqueMx, &yawSetpointMx, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);
double sat_turns = 0;

// ================================================================
// ===       PID Pitch Controller PARAMS                        ===
// ================================================================
double pitchSetpointMx, pitchInputMx, pitchControlTorqueMx, pitchControlTorqueMx2;
double Kp_pitch=0.7, Ki_pitch=0.00, Kd_pitch=0.00;
PID pitchControllerMx(&pitchInputMx, &pitchControlTorqueMx, &pitchSetpointMx, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

// ================================================================
// ===       PID Speed Controller PARAMS                        ===
// ================================================================
double speedSetpointMx, speedInputMx, controlTorqueMx;
double Kp_wx=0.001, Ki_wx=0.00, Kd_wx=0.00;
PID speedControllerMx(&speedInputMx, &controlTorqueMx, &speedSetpointMx, Kp_wx, Ki_wx, Kd_wx, DIRECT);

// ================================================================
// ===       PID Current Controller PARAMS, Motor x             ===
// ================================================================
double currentSetpointMx, currentInputMx, controlVoltageMx;
double Kp_imx=4, Ki_imx=8, Kd_imx=0;
PID currentControllerMx(&currentInputMx, &controlVoltageMx, &currentSetpointMx, Kp_imx, Ki_imx, Kd_imx, DIRECT);

// ================================================================
// ===       PID Current Controller PARAMS, Motor x-N           ===
// ================================================================
double currentSetpointMxN, currentInputMxN, controlVoltageMxN, controlVoltageMxB;
double Kp_imxN=4, Ki_imxN=8, Kd_imxN=0;
PID currentControllerMxN(&currentInputMxN, &controlVoltageMxN, &currentSetpointMxN, Kp_imxN, Ki_imxN, Kd_imxN, DIRECT);

// ================================================================
// ===       PID Current Controller PARAMS, Motor y             ===
// ================================================================
double currentSetpointMy, currentInputMy, controlVoltageMy;
double Kp_imy=4, Ki_imy=8, Kd_imy=0;
PID currentControllerMy(&currentInputMy, &controlVoltageMy, &currentSetpointMy, Kp_imy, Ki_imy, Kd_imy, DIRECT);

unsigned long i_time = 0;
unsigned long i_time2 = 0;

// ================================================================
// ===               Comunication                               ===
// ================================================================
uint8_t Timeout = 0;
uint8_t data_id = 0;

// ================================================================
// ===               Current Sensor Mx                          ===
// ================================================================
#define CURRENT_SENSOR_Mx A2
float current = 0;
float filtered_current = 0.0;
float currentFilterFrecuency = 0.6; //[Hz]
FilterOnePole currentlowpassFilter(LOWPASS, 2);

// ================================================================
// ===               Current Sensor My                          ===
// ================================================================
#define CURRENT_SENSOR_My A3
float current_my = 0;
float filtered_current_my = 0.0;
FilterOnePole currentlowpassFilterMy(LOWPASS, 2);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial2.begin(115200);
    while (!Serial2); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // Yaw Controller Initialization
    yawInputMx = 0;
    yawSetpointMx = 0.0;                          //[rad]
    yawControllerMx.SetOutputLimits(1.9, 4.8);   //[Nm]
    yawControllerMx.SetMode(AUTOMATIC);

    // Pitch Controller Initialization
    pitchInputMx = 0;
    pitchSetpointMx = 0.0;                            //[rad]
    pitchControllerMx.SetOutputLimits(1.9, 4.8);   //[Nm]
    pitchControllerMx.SetMode(AUTOMATIC);

    // Speed Controller Initialization
    speedInputMx = 0;
    speedSetpointMx = 0.0;                          //[RPM]
    speedControllerMx.SetOutputLimits(-3.0, 3.0);   //[Nm]
    speedControllerMx.SetMode(AUTOMATIC);

    // Current Controller Mx Initialization
    currentInputMx = 0;
    currentSetpointMx = 0.0;                    //[A]
    currentControllerMx.SetOutputLimits(1.9, 4.8); //[V]
    currentControllerMx.SetMode(AUTOMATIC);

    // Current Controller MxN Initialization
    currentInputMxN = 0;
    currentSetpointMxN = 0.0;                     //[A]
    currentControllerMxN.SetOutputLimits(-4.8, -1.9); //[V]
    currentControllerMxN.SetMode(AUTOMATIC);

    // Current Controller My Initialization
    currentInputMy = 0;
    currentSetpointMy = 0.0;                    //[A]
    currentControllerMy.SetOutputLimits(0, 4.8); //[V]
    currentControllerMy.SetMode(AUTOMATIC);

    //Hall Encoder pins
    pinMode(HALL1, INPUT);
    attach_halls();

    //Current sensors
    pinMode(CURRENT_SENSOR_Mx, INPUT);
    pinMode(CURRENT_SENSOR_My, INPUT);


    //Init HDD
    hddx.init();
    hddy.init();
    hddz.init();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    // if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            /*Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);*/
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            /*Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            /*Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            /*Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);*/
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            // Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    readCommands();

    //Angle domain transformation
    if (last_ypr0>3.05 && ypr[0]<0)
    {
        sat_turns++;    //Una vuelta en sentido horario
    }
    else if (last_ypr0<-3.05 && ypr[0]>0)
    {
        sat_turns--;    //Una vuelta en sentido antihorario
    }

    last_ypr0 = ypr[0];

    //Update Reference for Yaw Controller
    // yawSetpointMx = cmdyawMx+6.2831*sat_turns;
    // yawInputMx = ypr[0]+1.5707+6.2831*sat_turns;
    yawSetpointMx = cmdyawMx;
    yawInputMx = ypr[0]+3.1415;

    //Satellite Speed Calculation
    update_satellite_speed();
    sateliteFiltered_speed = satelliteSpeedFilter.input(satellite_speed);

    //Yaw Controller Calculation
    yawControllerMx.Compute();        //This update the yawControlTorqueMx variable

    //Update Reference for Speed Controller
    speedSetpointMx = cmdSpeedMx;
    // speedSetpointMy = cmdSpeedMy;

    //Speed Calculation
    update_speed();
    filtered_speed = lowpassFilter.input(speed_rpm);
    if (filtered_speed>5500)
        filtered_speed_calib = filtered_speed*0.68+870;             //[RPM]
    else if (filtered_speed<80)
        filtered_speed_calib = 0;                                   //[RPM]
    else
        filtered_speed_calib = filtered_speed;                      //[RPM]

    //Speed Controller Calculation
    speedInputMx = filtered_speed_calib;
    // speedControllerMx.Compute();        //This update the controlTorqueMx variable
    yawControlTorqueMx2 = Kp_yaw*(yawSetpointMx-yawInputMx)- Kd_yaw*(sateliteFiltered_speed);
    if (yawSetpointMx-yawInputMx>=0) yawControlTorqueMx2 = yawControlTorqueMx2+1.9;
    else yawControlTorqueMx2 = 0.0;
    if (yawControlTorqueMx2>4.8)
    {
        yawControlTorqueMx2 = 4.8;
    }
    else if (yawControlTorqueMx2<0)
    {
        yawControlTorqueMx2 = 0.0;
    }

    //pitch controller
    // pitchControlTorqueMx2 = Kp_pitch*(pitchSetpointMx-pitchInputMx);
    pitchControlTorqueMx2 = -Kp_pitch*(yawSetpointMx-yawInputMx)+1.9;
    if (pitchControlTorqueMx2>4.8)
    {
        pitchControlTorqueMx2 = 4.8;
    }
    else if (pitchControlTorqueMx2<0)
    {
        pitchControlTorqueMx2 = 0;
    }

    //Update Reference for Current Controller
    if (controlMode == POS_MODE)
    {
        currentSetpointMx = yawControlTorqueMx2;//   /KM
        currentSetpointMxN = yawControlTorqueMx2;//   /KM
        currentSetpointMy = pitchControlTorqueMx2;//   /KM
    }
    else if (controlMode == SPEED_MODE)
    {
        currentSetpointMx = controlTorqueMx;//   /KM
    }
    else if (controlMode == TORQUE_MODE)
    {
        currentSetpointMx = cmdTorqueMx;//      /KM
        currentSetpointMxN = cmdTorqueMx;//      /KM
        currentSetpointMy = cmdTorqueMy;//      /KM
    }

    //Current Mx Calculations
    float current_raw = analogRead(CURRENT_SENSOR_Mx);
    if (current_raw>=511) current = (current_raw-511)*0.06;
    else current = 0;
    filtered_current = currentlowpassFilter.input(current);     //[A]

    //motor state
    if (filtered_current<=0.16)
    {
        motorx_state = MOTOR_STOPPED;
    }
    else
    {
        motorx_state = MOTOR_ANT;
    }

    if ((lastIStpt>=0 && currentSetpointMx<0)||(lastIStpt<0 && currentSetpointMx>=0))
    {
        motorx_ref_dir_hasChange = 1;
    }

    //Current My Calculations
    float current_raw_my = analogRead(CURRENT_SENSOR_My);
    if (current_raw_my>=506) current_my = (current_raw_my-506)*0.06;
    else current_my = 0;
    filtered_current_my = currentlowpassFilterMy.input(current_my);  //[A]

    //Current Controller Mx Calculation
    if (motorx_ref_dir_hasChange==1 && filtered_speed_calib<=400)
    {
        if (currentSetpointMx>=0) currentxSign=1;
        else currentxSign=-1;
        motorx_ref_dir_hasChange = 0;
    }
    currentInputMx = filtered_current*currentxSign;
    currentInputMxN = filtered_current*currentxSign;
    lastIStpt = currentSetpointMx;

    currentControllerMx.Compute();      //This update the controlVoltageMx variable
    currentControllerMxN.Compute();      //This update the controlVoltageMxN variable

    //Current Controller My Calculation
    // i_time = millis()-i_time2;
    currentInputMy = filtered_current_my;
    currentControllerMy.Compute();      //This update the controlVoltageMy variable
    // i_time2 = millis();
    if (motorx_ref_dir_hasChange==1)
    {
        controlVoltageMxB = 0;
    }
    else if (currentSetpointMx>=0)
    {
        controlVoltageMxB = controlVoltageMx;
    }
    else
    {
        controlVoltageMxB = controlVoltageMxN;
    }

    //Set Motor Voltage Based on Operation Mode
    if (operationMode == OPENLOOP_MODE)      //Open Loop
    {
        hddx.rotate(cmdVoltageMx);
        hddy.rotate(cmdVoltageMy);
        hddz.rotate(cmdVoltageMz);
    }
    else if (operationMode == CLOSELOOP_MODE) //Close Loop
    {
        hddx.rotate(yawControlTorqueMx);
        hddy.rotate(pitchControlTorqueMx2);
        // hddz.rotate(pitchControlTorqueMx2);
    }
    send_data(1, yawSetpointMx, yawInputMx, yawControlTorqueMx, cmdVoltageMz);
    // send_data(2, controlVoltageMy, currentInputMx, currentSetpointMy, currentInputMy);
    // send_data(2, current_my, millis(), currentSetpointMy, currentInputMy);
}

//---------------Comunication Methos-------------------------------------------------------------

void sendFrame(uint8_t frame[], uint8_t sz)
{   
    for (int j=0;j<sz;j++) Serial2.write(frame[j]);
}

uint8_t checksum(uint8_t *packet, uint8_t n)
{
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00FF;
}

void bytesEncode(float number, uint8_t encode_bytes[])
{
    uint16_t int_part = (uint16_t) abs(number);          //[0-65535]
    float decimal_part = (abs(number) - int_part)*100;   //[0-99]

    uint8_t NH = (int_part)>>8;                //Number High Byte
    uint8_t NL = (int_part) & 0x00FF;          //Number Low Byte
    uint8_t D = (int)decimal_part;             //Decimal part (7 bits)
    uint8_t SD = D;                            //Sign and Decimal Byte
    if (number<=0) SD = D|0b10000000;          //Sign bit

    encode_bytes[0] = NH;
    encode_bytes[1] = NL;
    encode_bytes[2] = SD;
}

void encode(uint8_t id, float data[], uint8_t packet[])
{
    uint8_t num1_bytes[3];
    uint8_t num2_bytes[3];
    uint8_t num3_bytes[3];
    uint8_t num4_bytes[3];
    bytesEncode(data[0], num1_bytes);
    bytesEncode(data[1], num2_bytes);
    bytesEncode(data[2], num3_bytes);
    bytesEncode(data[3], num4_bytes);

    packet[0] = id;

    packet[1] = num1_bytes[0];
    packet[2] = num1_bytes[1];
    packet[3] = num1_bytes[2];

    packet[4] = num2_bytes[0];
    packet[5] = num2_bytes[1];
    packet[6] = num2_bytes[2];

    packet[7] = num3_bytes[0];
    packet[8] = num3_bytes[1];
    packet[9] = num3_bytes[2];

    packet[10] = num4_bytes[0];
    packet[11] = num4_bytes[1];
    packet[12] = num4_bytes[2];

    packet[13] = checksum(packet, 14);
}

void printFrame(uint8_t frame[], uint8_t sz)
{
    Serial.print("frame = [");
    for (int i = 0; i < sz-1; i++)
    {
        Serial.print(frame[i]);
        Serial.print(",");
    }
    Serial.print(frame[sz-1]);
    Serial.println("]");
}

void send_data(uint8_t id, float D1, float D2, float D3, float D4)
{
    float data[4] = {D1, D2, D3, D4};
    uint8_t frame_test[14];
    encode(id, data, frame_test);
    sendFrame(frame_test, 14);
    // printFrame(frame_test, 14);
}

float getNumber(uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    // Numer Elements
    uint16_t int_part = (byte1<<8)|byte2;
    uint8_t sign = byte3>>7;
    uint16_t decimal_part = byte3&(0b01111111);
    // Number
    float number;
    if (sign == 0) number = int_part+((float)decimal_part/100);
    else number = -(int_part+((float)decimal_part/100));
    return number;
}

float saturate(float data, int16_t MIN, int16_t MAX)
{
    // ---------------------------------------------------------------------------------CHECK!!!!!!!!!!!!!!!!!!!!!!!!!-------------------------------------
    return data;
    if (data>MAX)
        return MAX;
    else if (data<MIN)
        return MIN;
    else
        return data;
}

void decode(uint8_t frame[], float data[])
{
    data[0] = saturate(getNumber(frame[0], frame[1], frame[2]), 0, 100);
    data[1] = saturate(getNumber(frame[3], frame[4], frame[5]), -800, 800);
    data[2] = saturate(getNumber(frame[6], frame[7], frame[8]), -360, 360);
    data[3] = saturate(getNumber(frame[9], frame[10], frame[11]), -360, 360);
}

void stop_read(void)
{
    // Timer1.detachInterrupt();
    Timeout = 1;
}

uint8_t read(uint8_t frame[])
{
    uint8_t i = 0;
    uint8_t k = 0;
    uint8_t sz = 13;
    // Timer1.initialize(30000);
    // Timer1.attachInterrupt(stop_read);
    while (k < 2*sz)// and !Timeout)
    // while (Serial.available() >= 1)
    {
        if (Serial2.available() >= 1)
        {
            uint8_t byte = Serial2.read();
            frame[i] = byte;
            i+=1;
            if (i==sz)
            {
                uint8_t chksm = checksum(frame, sz);
                if (chksm == frame[sz-1] && chksm !=0)
                {
                    return 1; // packet received OK
                }
                else
                {
                    // Bad checksum
                    for (uint8_t j = 0; j < sz-1; j++)
                    {
                        frame[j] = frame[j+1]; // Shift frame Left
                    }
                    frame[sz-1] = 0; //Clean last byte to receive other packet
                    i = sz-1;
                }
            }
            k+=1;
        }
    }
    // Frame not received Correctly
    for (uint8_t j = 0; j < sz; j++) frame[j] = 0; // Reset packet
    // Timeout = 0;
    // while(Serial.available()) Serial.read();
    return 0;
}

void readCommands()
{
  if(Serial2.available() >= 1)
  {
    uint8_t frame[13];
    float command[4];
    read(frame);
    decode(frame, command);
    if(command[0]==SET_VOLTAGE)
    {
      cmdVoltageMx = command[1];
      cmdVoltageMy = command[2];
      cmdVoltageMz = command[3];
    }
    else if(command[0]==SET_TORQUE)
    {
      cmdTorqueMx = command[1];
      cmdTorqueMy = command[2];
    }
    else if(command[0]==SET_SPEED)
    {
      cmdSpeedMx = command[1];
      cmdSpeedMy = command[2];
    }
    else if (command[0]==KEEP_ATTITUDE)
    {
      cmdyawMx = command[1]*0.0175;
      /*cmdPitchSetpoint = command[2];
      cmdRollSetpoint = command[3];*/
    }
    else if (command[0]==SET_MODE)
    {
      operationMode = command[1];
    }
    else if (command[0]==SET_CONTROL_MODE)
    {
      controlMode = command[1];
    }
    else if (command[0]==CHANGE_CURRENT_GAINS)
    {
      Kp_imx = command[1];
      Ki_imx = command[2];
      Kd_imx = command[3];
      currentControllerMx.SetTunings(Kp_imx, Ki_imx, Kd_imx);
    }
    else if (command[0]==CHANGE_SPEED_GAINS)
    {
      Kp_wx = command[1];
      Ki_wx = command[2];
      Kd_wx = command[3];
      speedControllerMx.SetTunings(Kp_wx, Ki_wx, Kd_wx);
    }
    else if (command[0]==STOP)
    {
      operationMode = OPENLOOP_MODE;
      cmdVoltageMx = MIN_VOLTAGE;
      cmdVoltageMy = MIN_VOLTAGE;
      cmdVoltageMz = MIN_VOLTAGE;
      hddx.idle();
      hddy.idle();
      hddz.idle();
    }
    else if (command[0]==STOP_X)
    {
      operationMode = OPENLOOP_MODE;
      cmdVoltageMx = MIN_VOLTAGE;
      hddx.idle();
    }
    else if (command[0]==STOP_Y)
    {
      operationMode = OPENLOOP_MODE;
      cmdVoltageMy = MIN_VOLTAGE;
      hddy.idle();
    }
    else if (command[0]==STOP_Z)
    {
      operationMode = OPENLOOP_MODE;
      cmdVoltageMz = MIN_VOLTAGE;
      hddz.idle();
    }
    /*else if (command[0]==PRINT_SPEED)
    {
        TODO: send speed
    }*/
    else if (command[0] == AUTOMATIC_MODE)
    {
      operationMode = CLOSELOOP_MODE;
    }
    /*else if (command[0] == USE_CURRENT_SETPOINT)
    {
      cmdYawSetpoint = ypr[0]*180/M_PI;
    }*/
    /*else if (command[0] == INCREASE_SETPOINT)
    {
      cmdYawSetpoint+=10;
    }
    else if (command[0] == DECREASE_SETPOINT)
    {
      cmdYawSetpoint-=10;
    }*/
  }
}
//---------------Sensor Methos-------------------------------------------------------------
//---------------Hall efect Sensor Methos--------------------------------------------------
void attach_halls(void)
{
    attachInterrupt(digitalPinToInterrupt(HALL1), step, FALLING);
    // attachInterrupt(digitalPinToInterrupt(HALL2), step, FALLING);
    // attachInterrupt(digitalPinToInterrupt(HALL3), step, FALLING);
}

void dettach_halls(void)
{
    detachInterrupt(digitalPinToInterrupt(HALL1));
    // detachInterrupt(digitalPinToInterrupt(HALL2));
    // detachInterrupt(digitalPinToInterrupt(HALL3));
}

void update_speed(void)
{
    if (steps)
    {
        // Serial.print("steps:");Serial.println(steps);
        dettach_halls();
        speed_rpm = 60000.0*steps/(millis()-time_ref);
        // Serial.print("speed_rpm:");Serial.println(speed_rpm);

        time_ref = millis();
        steps = 0;

        attach_halls();
    }
}

void step(void)
{
    steps++;
}

//---------------Hall efect Sensor Methos--------------------------------------------------
void update_satellite_speed(void)
{
    satellite_speed = 1000*(yawInputMx-last_yaw)/(millis()-time_ref_pitch);    //[rad/s]
    time_ref_pitch = millis();
    last_yaw = yawInputMx;
}