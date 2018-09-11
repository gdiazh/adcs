/**
   @brief Simple ATMS Library
*/

/*Author: Gustavo Diaz*/

/*Requiered Libraries*/
#include <Arduino.h>

/**
   @class serialCom
   @brief Class for manage Atmospheric sensors
*/

class serialCom
{
    /*Private Members*/

    // Debug
    // HardwareSerial *debug_port_;

  public:
    /*Public Members*/
    /*constructor de base (null)*/
    serialCom() {}

    // methods
    void sendFrame(uint8_t frame[], uint8_t sz);
    uint8_t checksum(uint8_t *packet, uint8_t n);
    void bytesEncode(float number, uint8_t encode_bytes[]);
    void encode(uint8_t id, float data[], uint8_t packet[]);
    void printFrame(uint8_t frame[], uint8_t sz);
    void send_data(uint8_t id, float D1, float D2, float D3, float D4);
    // private:
    // methods
};
