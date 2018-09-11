/**
 * @brief Arduino Driver for Satellite Attitude Control
   @Author: Gustavo Diaz
 */

/*Requiered Libraries*/
#include "imu.h"
#include <PID_v1.h>
#include "hdd_driver.h"
// #include <TimerThree.h>
// #include <TimerOne.h>
#include <Filters.h>

#define INTERRUPT_PIN 31

IMU imu(INTERRUPT_PIN, &Serial);

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
#define ESC_PWM_PIN_OUT 5
#define ESC_DIR_PIN_OUT A1

#define ESC2_PWM_PIN_OUT 9
#define ESC2_DIR_PIN_OUT A0

#define ESC3_PWM_PIN_OUT 6
#define ESC3_DIR_PIN_OUT A1

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
float last_yaw = 0.0;

float sateliteFiltered_speed = 0.0;
FilterOnePole satelliteSpeedFilter(LOWPASS, 0.2);

// ================================================================
// ===       Eq Controller PARAMS                          		===
// ================================================================
double Kpx_eq=6.5, Kix_eq=0.00, Kdx_eq=0.4;
double Kpy_eq=3.3, Kiy_eq=0.00, Kdy_eq=0.3;
double Kpz_eq=1.4, Kiz_eq=0.00, Kdz_eq=0.2;
float cmdPitchT=0, cmdRollT=0;
double qT1 = 0.02, qT2 = 0.0, qT3 = 0.0, qTw = 1.0;
double qE1 = 0, qE2 = 0, qE3 = 0, qEw = 0;
double RW_Tcx = 0;
double RW_Tcy = 0;
double RW_Tcz = 0;

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
// #define CURRENT_SENSOR_Mx A2
#define TMP_SENSOR A2
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
    Serial.begin(115200);
    imu.init();
    // Yaw Controller Initialization
    // yawInputMx = 0;
    // yawSetpointMx = 0.0;                      	  //[rad]

    //Hall Encoder pins
    pinMode(HALL1, INPUT);
    attach_halls();

    //Current sensors
    // pinMode(CURRENT_SENSOR_Mx, INPUT);
    pinMode(TMP_SENSOR, INPUT);
    pinMode(CURRENT_SENSOR_My, INPUT);

    //Init HDD
    hddx.init();
    hddy.init();
    hddz.init();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    imu.updateData();
    //----------------------------------------------------------------------------------------------------
    readCommands();
    //------------------------ Equilibrium Controller Calculation ----------------------------------------

    //------------------------ Wheel Speed Calibration ---------------------------------------------------
    update_speed();
    filtered_speed = lowpassFilter.input(speed_rpm);
    if (filtered_speed>5500)
        filtered_speed_calib = filtered_speed*0.68+870;             //[RPM]
    else if (filtered_speed<80)
        filtered_speed_calib = 0;                                   //[RPM]
    else
        filtered_speed_calib = filtered_speed;                      //[RPM]
    //------------------------ RW's Actuation ------------------------------------------------------------
    //Set Motor Voltage Based on Operation Mode
    if (operationMode == OPENLOOP_MODE)      //Open Loop
    {
        hddx.rotate(cmdVoltageMx);
        hddy.rotate(cmdVoltageMy);
        hddz.rotate(cmdVoltageMz);
    }
    else if (operationMode == CLOSELOOP_MODE) //Close Loop
    {
    	getQuaternionTarget(cmdyawMx, cmdPitchT, cmdRollT, &qT1, &qT2, &qT3, &qTw);
    	qE1 = -qTw*imu.q.x  -qT3*imu.q.y   +qT2*imu.q.z   +qT1*imu.q.w;
	    qE2 = +qT3*imu.q.x  -qTw*imu.q.y   -qT1*imu.q.z   +qT2*imu.q.w;
	    qE3 = -qT2*imu.q.x  +qT1*imu.q.y   -qTw*imu.q.z   +qT3*imu.q.w;
	    qEw = +qT1*imu.q.x  +qT2*imu.q.y   +qT3*imu.q.z   +qTw*imu.q.w;    //scalar part

	    RW_Tcx = 0.0;//+(Kpx_eq*qE1*qEw + Kdx_eq*imu.ws.x);
	    RW_Tcy = 0.0;//+(Kpy_eq*qE2*qEw + Kdy_eq*imu.ws.y);
	    RW_Tcz = 0.0+(Kpz_eq*qE3*qEw + Kdz_eq*imu.ws.z);

        hddx.rotate(1.3+abs(RW_Tcx));
        hddy.rotate(1.3+abs(RW_Tcy));
        hddz.rotate(1.3+abs(RW_Tcz));
    }
	//--------------------------------TMP-Sensor----------------------------------------------------------
	float tmp_reading = analogRead(TMP_SENSOR);
	float tmpC = (tmp_reading*0.00488-0.5)*100;
    //------------------------ Tx Data -------------------------------------------------------------------
    send_data(1, millis(), imu.ws.x, imu.ws.y, imu.ws.z);
    send_data(2, imu.q.x, imu.q.y, imu.q.z, imu.q.w);
}

//---------------Comunication Methos-------------------------------------------------------------

void sendFrame(uint8_t frame[], uint8_t sz)
{   
    for (int j=0;j<sz;j++) Serial.write(frame[j]);
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
        if (Serial.available() >= 1)
        {
            uint8_t byte = Serial.read();
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
  if(Serial.available() >= 1)
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
      cmdPitchT = command[2]*0.0175;
      cmdRollT = command[3]*0.0175;
    }
    else if (command[0]==SET_MODE)
    {
      operationMode = command[1];
      /*qT1 = imu.q.x;
      qT2 = imu.q.y;
      qT3 = imu.q.z;
      qTw = imu.q.w;*/
    }
    else if (command[0]==SET_CONTROL_MODE)
    {
      controlMode = command[1];
    }
    /*else if (command[0]==CHANGE_CURRENT_GAINS)
    {
      Kp_imx = command[1];
      Ki_imx = command[2];
      Kd_imx = command[3];
      // currentControllerMx.SetTunings(Kp_imx, Ki_imx, Kd_imx);
    }
    else if (command[0]==CHANGE_SPEED_GAINS)
    {
      Kp_wx = command[1];
      Ki_wx = command[2];
      Kd_wx = command[3];
      // speedControllerMx.SetTunings(Kp_wx, Ki_wx, Kd_wx);
    }*/
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
      cmdYawSetpoint = imu.ypr[0]*180/M_PI;
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

void getQuaternionTarget(float cmdyawMx, float cmdPitchT, float cmdRollT, double *qT1, double *qT2, double *qT3, double *qTw)
{
	//abbreviations
	double cy = cos(cmdyawMx*0.5);
    double sy = sin(cmdyawMx*0.5);
    double cr = cos(cmdRollT*0.5);
    double sr = sin(cmdRollT*0.5);
    double cp = cos(cmdPitchT*0.5);
    double sp = sin(cmdPitchT*0.5);
    //Calc Target quaternion
    *qT1 = cy * sr * cp - sy * cr * sp;
    *qT2 = cy * cr * sp + sy * sr * cp;
    *qT3 = sy * cr * cp - cy * sr * sp;
    *qTw = cy * cr * cp + sy * sr * sp;   //scalar part
}