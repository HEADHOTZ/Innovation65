/*******************************************************************************
 * ET-ESP32(WROVER) RS485 V3
 * Tools->Board:"ESP32 Wrover Module"
 *******************************************************************************
 * I2C Interface & I2C Bus
 * -> IO22                = I2C_SCL
 * -> IO21                = I2C_SDA
 * -> I2C RTC:DS3231      = I2C Address : 0x68:1100100(x)
 * -> I2C EEPROM 24LC16   = I2C Address : 0x50:1010000(x)
 * -> I2C ADC MCP3423     = I2C Address : 0x6D:1100101(x)
 * -> I2C Sensor:BME280   = I2C Address : 0x76:1110110(x)
 * -> I2C Sebsor:SHT31    = I2C Address : 0x44:1000100(x)/0x45:1010101(x)
 * SPI Interface SD Card
 * -> SD_CS               = IO4
 * -> SPI_MISO            = IO19
 * -> SPI_MOSI            = IO23
 * -> SPI_SCK             = IO18
 * UART2 RS485 Half Duplex Auto Direction
 * -> IO26                = RX2
 * -> IO27                = TX2
 * Led
 * -> IO2                 = LED Status
 * User Switch
 * -> IO36                = User Switch
 * Buzzer
 * -> IO32                = Buzzer
 * Opto Output
 * -> IO25                = Opto Output
 *******************************************************************************/
 
//=================================================================================================
#include <Wire.h> 
//=================================================================================================

//=================================================================================================
// Start of Default Hardware : ET-ESP32(WROVER) RS485 V3
//=================================================================================================
// Remap Pin USART -> C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//                    C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug           Serial                                                              // USB Serial(Serial0)
//=================================================================================================
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2                                                             // Serial2(IO27=TXD,IO26=RXD)
//=================================================================================================
#define SerialLora_RX_PIN     14
#define SerialLora_TX_PIN     13
#define SerialLora            Serial1                                                             // Serial1(IO13=TXD,IO14=RXD)
//=================================================================================================
#define LORA_RES_PIN          33                                                                  // ESP32-WROVER :IO33(LoRa-RESET)
#define LORA_RES_PRESS        LOW
#define LORA_RES_RELEASE      HIGH
//=================================================================================================
#define I2C_SCL_PIN           22                                                                  // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN           21                                                                  // ESP32-WROVER : IO21(SDA1)
//=================================================================================================
#define LedPin                2                                                                   // ESP32-WROVER  : IO2
#define LedLogicOn            HIGH
#define LedLogicOff           LOW
#define InitialLed()          pinMode(LedPin,OUTPUT)
#define OnLed()               digitalWrite(LedPin, LedLogicOn)
#define OffLed()              digitalWrite(LedPin, LedLogicOff)
//=================================================================================================
#define UserSwitchPin         36                                                                  // ESP32-WROVER :IO36
#define SwLogicPress          LOW
#define SwLogicRelease        HIGH 
#define InitialUserSwitch()   pinMode(UserSwitchPin,INPUT_PULLUP)
//=================================================================================================
#define BuzzerPin             32                                                                  // ESP32-WROVER :IO32
#define BuzzerLogicOff        LOW
#define BuzzerLogicOn         HIGH 
#define InitialBuzzer()       pinMode(BuzzerPin,OUTPUT)
#define OnBuzzer()            digitalWrite(BuzzerPin, BuzzerLogicOn) 
#define OffBuzzer()           digitalWrite(BuzzerPin, BuzzerLogicOff)
//=================================================================================================
#define OptoOutputPin         25                                                                  // ESP32-WROVER : IO25
#define OptoOutputLogicOn     LOW
#define OptoOutputLogicOff    HIGH
#define InitialOptoOutput()   pinMode(OptoOutputPin,OUTPUT)
#define OnOptoOutput()        digitalWrite(OptoOutputPin, OptoOutputLogicOn)
#define OffOptoOutput()       digitalWrite(OptoOutputPin, OptoOutputLogicOff)
//=================================================================================================
// End of Default Hardware : ET-ESP32(WROVER) RS485 V3
//=================================================================================================
 
//=================================================================================================
#include "ETT_PCF8574.h"
//=================================================================================================
ETT_PCF8574 exp_i2c_io(PCF8574A_ID_DEV0);                                                         // ET-ESP32-RS485 V3 : Input/Output(PCF8574:ID0)
//=================================================================================================
#define ry0_onboard_pin       0                                                                   // P0 = Relay[0]
#define ry1_onboard_pin       2                                                                   // P2 = Relay[1]
#define ry2_onboard_pin       4                                                                   // P4 = Relay[2]
#define ry3_onboard_pin       6                                                                   // P6 = Relay[3]
//=================================================================================================
#define RelayOn               LOW                                                                 // On Relay Ative(LOW)
#define RelayOff              HIGH                                                                // Off Relay Active(LOW)
//=================================================================================================
int relay_onboard_pin[4] =    {ry0_onboard_pin, ry1_onboard_pin, 
                               ry2_onboard_pin, ry3_onboard_pin};                                 // ett relay/pcf8574
//=================================================================================================
#define opto_in0_pin          1                                                                   // P1 Read Opto In(0) Active(LOW)
#define opto_in1_pin          3                                                                   // P3 Read Opto In(1) Active(LOW)
#define opto_in2_pin          5                                                                   // P5 Read Opto In(2) Active(LOW)
#define opto_in3_pin          7                                                                   // P7 Read Opto In(3) Active(LOW)
//=================================================================================================
#define OptoLogicOn           LOW                                                                 // Opto In On(LOW)
#define OptoLogicOff          HIGH                                                                // Opto In Off(HIGH)
//=================================================================================================
int opto_input_pin[4] = {opto_in0_pin, opto_in1_pin, opto_in2_pin, opto_in3_pin};                 // ett opto-in/pcf8574
//=================================================================================================

//=================================================================================================
// Read Switch by PCF8574 Pin
//=================================================================================================
#define InitialOptoInput(i)   exp_i2c_io.writePin(opto_input_pin[i], HIGH)                        // Initial Input Pin
#define ReadOptoInput(i)      !exp_i2c_io.readPin(opto_input_pin[i])                              // Read Opto Input : 0 = OFF, 1 = ON
//=================================================================================================  

//=================================================================================================
// Control Relay by PCF8574 Pin
//=================================================================================================
#define OnRelay(r)            exp_i2c_io.writePin(relay_onboard_pin[r], RelayOn)                  // Open  = ON  Relay
#define OffRelay(r)           exp_i2c_io.writePin(relay_onboard_pin[r], RelayOff)                 // Close = OFF Relay
#define ToggleRelay(r)        exp_i2c_io.writePin(relay_onboard_pin[r], !exp_i2c_io.readPin(r))   // Close = OFF Relay
//=================================================================================================
#define ReadRelay(r)          !exp_i2c_io.readPin(relay_onboard_pin[r])                           // Read Relay ON(1)/OFF(0) Status
//=================================================================================================

void setup() 
{
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V3
  //===============================================================================================
  InitialLed();
  InitialUserSwitch();
  InitialBuzzer();
  InitialOptoOutput();
  //===============================================================================================
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);                                                      
  //===============================================================================================
  SerialDebug.begin(115200);
  while(!SerialDebug);
  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while(!SerialRS485);
  //===============================================================================================
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //===============================================================================================

  //===============================================================================================
  exp_i2c_io.begin(0xFF);
  //===============================================================================================
  OffRelay(0);
  OffRelay(1);
  OffRelay(2);
  OffRelay(3);
  InitialOptoInput(0);
  InitialOptoInput(1);
  InitialOptoInput(2);
  InitialOptoInput(3);
  //=============================================================================================== 
  
  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Ready");
  //===============================================================================================
}

void loop() 
{
  //===============================================================================================
  OnLed();
  OnBuzzer();
  delay(250);
  OffLed();
  OffBuzzer();
  //===============================================================================================
  for(int i=0; i<4; i++)
  {
    OnRelay(i);
    delay(1000);  
  }
  //===============================================================================================
  OnOptoOutput();
  //===============================================================================================

  //===============================================================================================
  OnLed();
  OnBuzzer();
  delay(250);
  OffLed();
  OffBuzzer();
  //===============================================================================================
  for(int i=0; i<4; i++)
  {
    OffRelay(i);
    delay(1000);  
  } 
  //===============================================================================================
  OffOptoOutput();
  //===============================================================================================
}



