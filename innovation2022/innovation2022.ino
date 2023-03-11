/*
 * Demo ET-ESP32-RS485 Hardware Board
 * MCU      : ESP32-WROVER
 *          : Arduino Tools Board : ESP32 Wrover Module
 * Software : https://dl.espressif.com/dl/package_esp32_index.json        
 * Reserve  : Boot Config       
 *          : -> IO0(BOOT)
 *          : -> IO2(Don't Card on 3V3V Version)
 *          : -> IO5
 *          : -> IO12(MTDI)
 *          : -> IO15(MTDO)
 * Reserve  : SFLASH        
 *          : -> CMD
 *          : -> CLK
 *          : -> SD0
 *          : -> SD1
 *          : -> SD2
 *          : -> SD3
 * Debug    : Serial0 : USART0(USB)        
 *          : -> RX0(Debug & Download:IO3)
 *          : -> TX0(Debug & Download:IO1)
 * NB-IoT   : Serial1 : SIM7020E(BK-7020 V2)       
 *          : -> RX1(T:IO14)
 *          : -> TX1(R:IO13)
 *          : -> PWRKEY(K:IO33)
 *          : -> SLEEP(S:IO32)
 * RS485    : Serial1 : RS485  
 *          : -> RX2(IO26)
 *          : -> TX2(IO27)
 *          : -> DIR(Direction : IO25 : LOW = Receive, HIGH = Send)
 * I2C#1    : I2C BUS
 *          : -> SCL1(IO22)
 *          : -> SDA1(IO21)
 * RTC      : -> RTC:DS3231/DS3232
 *          : -> PCF8574/A(Relay8)
 *          : -> PCF8574/A(DC-IN8)
 * LED      : LED Status(Active High)
 *          : -> LED0(IO2)
 * Demo     : RS485 Interface -> ET-MODBUS RTU SHT31 BOX V2
 *          : -> Setup ET-MODBUS RTU SHT31 BOX V2 ID = 247 
 *          : ET-ESP32-RS485 : Setup RS422/485 Hardware
 *          : -> FULL/HALF   = HALF
 *          : -> 422/485     = 485
 *          : -> 4WIRE/2WIRE = 2WIRE
 *          : -> RZ(ENA/DIS) = ENA
 *          : -> RH(ENA/DIS) = ENA
 *          : -> RL(ENA/DIS) = ENA
 */

//=================================================================================================
#include <Wire.h>
//=================================================================================================
#include "ETT_ModbusRTU.h"
//=================================================================================================

//=================================================================================================
// Start of Default Hardware : ET-ESP32-RS485
//=================================================================================================
// Remap Pin USART -> C:\Users\Admin\Documents\Arduino\hardware\espressif\esp32\cores\esp32\HardwareSerial.cpp
//                    C:\Users\Admin\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.0\cores\esp32\HardwareSerial.cpp
//=================================================================================================
#include <HardwareSerial.h>
//=================================================================================================
#define SerialDebug Serial  // USB Serial(Serial0)
//=================================================================================================
#define SerialNBIOT_RX_PIN 14
#define SerialNBIOT_TX_PIN 13
#define SerialNBIOT Serial1  // Serial1(IO13=TXD,IO14=RXD)
//=================================================================================================
#define SerialRS485_RX_PIN 26
#define SerialRS485_TX_PIN 27
#define SerialRS485 Serial2  // Serial2(IO27=TXD,IO26=RXD)
//=================================================================================================
#define RS485_DIRECTION_PIN 25  // ESP32-WROVER :IO25
#define RS485_RXD_SELECT LOW
#define RS485_TXD_SELECT HIGH
//=================================================================================================
#define I2C_SCL1_PIN 22  // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA1_PIN 21  // ESP32-WROVER : IO21(SDA1)
//=================================================================================================

//=================================================================================================
// End of Default Hardware : ET-ESP32-RS485
//=================================================================================================

//=================================================================================================
// Data Array For Modbus Network Sharing for ET-Modbus RTU SHT31 Box V2
//=================================================================================================
// Input[0]       = Firmware Version
// Input[1]       = SHT31 Temperature Sensor Signed 16Bit(Signed/10)
// Input[2]       = SHT31 Humidity Sensor Signed 16Bit(Signed/10)
// Input[3]       = DS18B20 Temperature Sensor Signed 16Bit(Signed/10)
// Input[4]       = BH1750 Light Sensor Signed 16Bit(Signed/10)
// Input[5:6]     = SHT31 Temperature Sensor Float 32 Bit(Big-endian)
// Input[7:8]     = SHT31 Humidity Sensor Float 32Bit(Big-endian)
// Input[9:10]    = DS18B20 Temperature Sensor Float 32 Bit(Big-endian)
// Input[11:12]   = BH1750 Light Sensor Float 32 Bit(Big-endian)
//=================================================================================================
uint16_t InputRegister[13];
//=================================================================================================
// Holding[0]     = Slave ID
// Holding[1]     = SHT31 Temperature Adjust Signed 16Bit(Signed/10)
// Holding[2]     = SHT31 Humidity Adjust Signed 16Bit(Signed/10)
// Holding[3]     = DS18B20 Temperature Adjust Signed 16Bit(Signed/10)
// Holding[4]     = BH1750 Light Adjust Signed 16Bit(Signed/10)
// Holding[5:6]   = SHT31 Temperature Adjust Float 32 Bit(Big-endian)
// Holding[7:8]   = SHT31 Humidity Adjust Float 32Bit(Big-endian)
// Holding[9:10]  = DS18B20 Temperature Adjust Float 32 Bit(Big-endian)
// Holding[11:12] = BH1750 Light Adjust Float 32 Bit(Big-endian)
//=================================================================================================
uint16_t HoldingRegister[13];
//=================================================================================================
uint8_t u8state;  // machine state
uint8_t u8query;  // pointer to message query
//=================================================================================================

//=================================================================================================
// u_int8     : [+0][+1][+2][+3]
// u_int16    : [+0][+1]
// u_int32    : [0]
//=================================================================================================
union {
  //===============================================================================================
  uint8_t u_uint8[4];    // [MSB...LSB = 0:1:2:3] = 78 : 56 : 34 : 12
  uint16_t u_uint16[2];  // [MSB...LSB = 0:1]     = 7856 : 3412
  int16_t u_int16[2];    // [MSB...LSB = 0:1]     = 7856 : 3412
  int32_t u_int32;       // 78563412
  uint32_t u_uint32;     // 78563412
  //===============================================================================================
  float u_float;
  //===============================================================================================
} u_var;
//=================================================================================================

//=================================================================================================
uint16_t sensor_sht31_temperature_signed_value;
//=================================================================================================
float sensor_sht31_temperature_float_value;
//=================================================================================================
uint16_t sensor_sht31_humidity_signed_value;
//=================================================================================================
float sensor_sht31_humidity_float_value;
//=================================================================================================
uint16_t sensor_ds18b20_temperature_signed_value;
//=================================================================================================
float sensor_ds18b20_temperature_float_value;
//=================================================================================================
uint16_t sensor_bh1750_light_signed_value;
//=================================================================================================
float sensor_bh1750_light_float_value;
//=================================================================================================

//=================================================================================================
uint16_t adjust_sht31_temperature_signed_value;
//=================================================================================================
float adjust_sht31_temperature_float_value;
//=================================================================================================
uint16_t adjust_sht31_humidity_signed_value;
//=================================================================================================
float adjust_sht31_humidity_float_value;
//=================================================================================================
uint16_t adjust_ds18b20_temperature_signed_value;
//=================================================================================================
float adjust_ds18b20_temperature_float_value;
//=================================================================================================
uint16_t adjust_bh1750_light_signed_value;
//=================================================================================================
float adjust_bh1750_light_float_value;
//=================================================================================================


/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0,                     // node id = 0(master)
              SerialRS485,           // Serial2
              RS485_DIRECTION_PIN);  // RS485 Modbus

/**
 * This is an structe which contains a query to an slave device
 */
//=================================================================================================
modbus_t telegram[2];  // 2-Modbus Frame Service
//=================================================================================================
unsigned long u32wait;
//=================================================================================================
unsigned long lastScanBusTime = 0;
//=================================================================================================
#define BLYNK_TEMPLATE_ID "TMPLMJsU4WAj"
#define BLYNK_DEVICE_NAME "Quickstart Device"
#define BLYNK_AUTH_TOKEN "NX-0kkvROFUt7tgQFIS1QN86daPr-K9I"
//=================================================================================================
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "iotkid";
char pass[] = "50470108";
int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;
//=================================================================================================
#include "ETT_PCF8574.h"
ETT_PCF8574 exp_i2c_io(PCF8574A_ID_DEV0);
#define RY1 0
#define RY2 2
#define RY3 4
#define RY4 6
#define RelayOn LOW
#define RelayOff HIGH
int relay_onboard_pin[4] = { RY1, RY2, RY3, RY4 };
#define OnRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], RelayOn)                     // Open  = ON  Relay
#define OffRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], RelayOff)                   // Close = OFF Relay
#define ToggleRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], !exp_i2c_io.readPin(r))  // Close = OFF Relay
#define ReadRelay(r) !exp_i2c_io.readPin(relay_onboard_pin[r])                            // Read Relay ON(1)/OFF(0) Status
//=================================================================================================

BLYNK_WRITE(V0) {
  //int val = param.asInt();
  val1 = param.asInt();
  /*if (val == 1) OnRelay(0);
  else if (val == 0) OffRelay(0);*/
}

BLYNK_WRITE(V1) {
  val2 = param.asInt();
  /*int val = param.asInt();
  if (val == 1) OnRelay(1);
  else if (val == 0) OffRelay(1);*/
}

BLYNK_WRITE(V2) {
  val3 = param.asInt();
  /*int val = param.asInt();
  if (val == 1) OnRelay(2);
  else if (val == 0) OffRelay(2);*/
}

BLYNK_WRITE(V3) {
  val4 = param.asInt();
  /*int val = param.asInt();
  if (val == 1) OnRelay(3);
  else if (val == 0) OffRelay(3);*/
}

void setup() {
  //===============================================================================================
  Blynk.begin(auth, ssid, pass);
  //===============================================================================================
  exp_i2c_io.begin(0xFF);
  OffRelay(0);
  OffRelay(1);
  OffRelay(2);
  OffRelay(3);
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32-RS485
  //===============================================================================================
  Wire.begin(I2C_SDA1_PIN, I2C_SCL1_PIN);
  //===============================================================================================
  pinMode(RS485_DIRECTION_PIN, OUTPUT);  // RS485 Direction
  digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT);
  //===============================================================================================
  SerialDebug.begin(115200);
  while (!SerialDebug)
    ;
  //===============================================================================================
  SerialNBIOT.begin(115200, SERIAL_8N1, SerialNBIOT_RX_PIN, SerialNBIOT_TX_PIN);
  while (!SerialNBIOT)
    ;
  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while (!SerialRS485)
    ;
  //===============================================================================================
  // End of Initial Default Hardware : ESP32
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println();
  SerialDebug.println("ET-ESP32-RS485...Demo Modbus RTU RS485");
  SerialDebug.println("Interface...ET-Modbus RTU SHT31 DS18B20 BH1750 V2 Box");
  //===============================================================================================

  //===============================================================================================
  // telegram 0: Read Input Register
  //===============================================================================================
  telegram[0].u8id = 247;  // Slave Address
  //===============================================================================================
  telegram[0].u8fct = 4;                // Function 0x04(Read Input Register)
  telegram[0].u16RegAdd = 0;            // Start Address Read(0x0000)
  telegram[0].u16CoilsNo = 13;          // Number of Register to Read(13 Input Register)
  telegram[0].au16reg = InputRegister;  // Pointer to Buffer Save Input Register
  //===============================================================================================

  //===============================================================================================
  // telegram 1: Read Holding Register
  //===============================================================================================
  telegram[1].u8id = 247;  // Slave Address
  //===============================================================================================
  telegram[1].u8fct = 3;                  // Function 0x03(Read Holding Register)
  telegram[1].u16RegAdd = 0;              // Start Address Read(0x0000)
  telegram[1].u16CoilsNo = 13;            // Number of Register to Read(13 Holding Register)
  telegram[1].au16reg = HoldingRegister;  // Pointer to Buffer Save Holding Register
  //===============================================================================================

  //===============================================================================================
  master.begin(SerialRS485);  // Mosbus Interface
  master.setTimeOut(2000);    // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 2000;
  u8state = u8query = 0;
  //===============================================================================================
  lastScanBusTime = millis();
  //===============================================================================================
}

void loop() {
  Blynk.run();
  if (val1 == 1) OnRelay(0);
  else if (val1 == 0) OffRelay(0);
  if (val2 == 1) OnRelay(1);
  else if (val2 == 0) OffRelay(1);
  if (val3 == 1) OnRelay(2);
  else if (val3 == 0) OffRelay(2);
  if (val4 == 1) OnRelay(3);
  else if (val4 == 0) OffRelay(3);
   
  switch (u8state) {
    case 0:
      //===========================================================================================
      if (millis() > u32wait) u8state++;  // wait state
      //===========================================================================================
      break;

    case 1:
      //===========================================================================================
      master.query(telegram[u8query]);  // send query (only once)
      u8state++;
      u8query++;
      //===========================================================================================
      if (u8query > 1) u8query = 0;  // telegram[0],telegram[1], ----> ,telegram[0]
      //===========================================================================================
      break;

    case 2:
      //===========================================================================================
      if (master.poll())  // check incoming messages
      {
        //=========================================================================================
        // Start of ET-MOSBUS RTU SHT31 Box V2 Service Data
        //=========================================================================================
        SerialDebug.println("Demo ET-ESP32 RS485 & ET-Modbus RTU SHT31 DS18B20 BH1750 Box V2");
        //=========================================================================================

        //=========================================================================================
        // Modbus Holding Register
        //=========================================================================================
        SerialDebug.print("RS485 Modbus Device Slave ID    = ");
        SerialDebug.println(HoldingRegister[0]);
        //=========================================================================================
        adjust_sht31_temperature_signed_value = HoldingRegister[1];
        adjust_sht31_humidity_signed_value = HoldingRegister[2];
        adjust_ds18b20_temperature_signed_value = HoldingRegister[3];
        adjust_bh1750_light_signed_value = HoldingRegister[4];
        //=========================================================================================
        u_var.u_int16[1] = HoldingRegister[5];  // LSB:Float sht31 Temperature
        u_var.u_int16[0] = HoldingRegister[6];  // MSB:Float sht31 Temperature
        //=========================================================================================
        adjust_sht31_temperature_float_value = u_var.u_float;
        //=========================================================================================
        u_var.u_int16[1] = HoldingRegister[7];  // LSB:Float sht31 Humidity
        u_var.u_int16[0] = HoldingRegister[8];  // MSB:Float sht31 Humidity
        //=========================================================================================
        adjust_sht31_humidity_float_value = u_var.u_float;
        adjust_ds18b20_temperature_float_value = u_var.u_float;
        //=========================================================================================
        u_var.u_int16[1] = HoldingRegister[11];  // LSB:Float bh1750 Light
        u_var.u_int16[0] = HoldingRegister[12];  // MSB:Float bh1750 Light
        //=========================================================================================
        adjust_bh1750_light_float_value = u_var.u_float;
        //=========================================================================================
        // Modbus Input Register
        //=========================================================================================
        SerialDebug.print("Modbus Device Firmware Version  = ");
        SerialDebug.println(InputRegister[0]);
        //=========================================================================================
        sensor_sht31_temperature_signed_value = InputRegister[1];
        sensor_sht31_humidity_signed_value = InputRegister[2];
        sensor_ds18b20_temperature_signed_value = InputRegister[3];
        sensor_bh1750_light_signed_value = InputRegister[4];
        //=========================================================================================
        u_var.u_int16[1] = InputRegister[5];  // LSB:Float SHT31 Temperature
        u_var.u_int16[0] = InputRegister[6];  // MSB:Float SHT31 Temperature
        //=========================================================================================
        sensor_sht31_temperature_float_value = u_var.u_float;
        //=========================================================================================
        u_var.u_int16[1] = InputRegister[7];  // LSB:Float SHT31 Humidity
        u_var.u_int16[0] = InputRegister[8];  // MSB:Float SHT31 Humidity
        //=========================================================================================
        sensor_sht31_humidity_float_value = u_var.u_float;
        //=========================================================================================
        u_var.u_int16[1] = InputRegister[11];  // LSB:Float BH1750 Light
        u_var.u_int16[0] = InputRegister[12];  // MSB:Float BH1750 Light
        //=========================================================================================
        sensor_bh1750_light_float_value = u_var.u_float;
        //=========================================================================================
        SerialDebug.print("Sensor SHT31 Temperature Float Value  = ");
        SerialDebug.println(sensor_sht31_temperature_float_value);
        Blynk.virtualWrite(V5, sensor_sht31_temperature_float_value);
        //=========================================================================================
        SerialDebug.print("Sensor SHT31 Humidity Float Value     = ");
        SerialDebug.println(sensor_sht31_humidity_float_value);
        Blynk.virtualWrite(V6, sensor_sht31_humidity_float_value);
        //=========================================================================================
        SerialDebug.print("Sensor BH1750 Light Float Value  = ");
        SerialDebug.println(sensor_bh1750_light_float_value);
        Blynk.virtualWrite(V4, sensor_bh1750_light_float_value);
        //=========================================================================================
        SerialDebug.println("");
        //=========================================================================================
        // Start of ET-MOSBUS RTU SHT31 Box V2 Service Data
        //=========================================================================================
      }
      //===========================================================================================

      //===========================================================================================
      if (master.getState() == COM_IDLE) {
        u8state = 0;
        u32wait = millis() + 2000;
      }
      //===========================================================================================
      break;
  }
}
