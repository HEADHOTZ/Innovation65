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
// demo interface modbus sensor
//=================================================================================================
// modbus sensor   : ET-MODBUS RTU SHT31 & BH1750 BOX V2/V3 setup Slave ID =247
// modbus moisture : SOIL MOISTURE H/T/EC setup Slave ID = 244
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
// Moisture Sensor : SOIL MOISTURE H/T/EC MODBUS RTU
//=================================================================================================
// Red    = +5V or 24V(3.6-30VDC)
// Black  = GND
// White  = RS485:B(-)
// Yellow = RS485:A(+)
// Green  = NC
//=================================================================================================
// Holding[512] = Slave ID
// Input[0]     = Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C)
// Input[1]     = Soil Moisture/100(%H)   -> 0 - 10000 : 0 - 100(%H)
// Input[2]     = Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
//=================================================================================================

//=================================================================================================
// Modbus Sensor : ET-Modbus RTU SHT31 Box V2/V3
//=================================================================================================
// Red   = 7-24V
// Black = GND
// White = RS485:A(+)
// Green = RS485:B(-)
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

//=================================================================================================
#include "ModbusMaster.h"                                                                         // https://github.com/4-20ma/ModbusMaster
//=================================================================================================
ModbusMaster nodeSensorWeather;
ModbusMaster nodeSoilMoisture;
//=================================================================================================
#define modbusMoisture_SlaveID    244                                                             // modbus Moisture : SOIL MOISTURE H/T/EC[244]
#define modbusSensor_SlaveID      247                                                             // modbus Sensor   : ET-MODBUS RTU SHT31 & BH1750 BOX V2/V3[247]
//=================================================================================================
uint8_t read_modbus_status;
//=================================================================================================
float sht31_temperature;
float sht31_humidity;
float bh1750_lux;
//=================================================================================================
float soil_temperature;
float soil_moisture;
float soil_ec;
//=================================================================================================
unsigned long lastGetModbusTime = 0;
bool indexGetModbus;                                                                              // Ture = Sensor(SHT31,BH1750), False = Moisture
//=================================================================================================

void setup() 
{
  //===============================================================================================
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V3
  //===============================================================================================
  InitialLed();
  //===============================================================================================
  SerialDebug.begin(115200);
  while(!SerialDebug);
  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while(!SerialRS485);
  //===============================================================================================
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V3
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Start");
  //===============================================================================================
 
  //===============================================================================================
  // Start of Config Modbus RTU RS485  
  //===============================================================================================
  nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485);                                     // ET-MODBUS RTU SHT31 & BH1750 BOX V2
  nodeSoilMoisture.begin(modbusMoisture_SlaveID, SerialRS485);                                    // SOIL MOISTURE H/T/EC MODBUS RTU
  //===============================================================================================
  // End of Config Modbus RTU RS485  
  //===============================================================================================

  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Ready");
  SerialDebug.println();
  //===============================================================================================
}

void loop() 
{
  //===============================================================================================
  if((millis() - lastGetModbusTime) > 10000ul)                                                    // 10-Second
  {
    //=============================================================================================
    OnLed();
    OnBuzzer();
    //=============================================================================================
    indexGetModbus = !indexGetModbus;                                                             // Toggle Status Access Modbus(Sensor/Moisture)
    //=============================================================================================
    
    //=============================================================================================
    // Start of Read Modbus Sensor  
    //=============================================================================================
    if(indexGetModbus)
    {
      //===========================================================================================
      // Modbus Sensor : ET-Modbus RTU SHT31 Box V2
      //===========================================================================================
      // Input[0]       = Firmware Version
      // Input[1]       = SHT31 Temperature Sensor Signed 16Bit(Signed/10)
      // Input[2]       = SHT31 Humidity Sensor Signed 16Bit(Signed/10)
      // Input[3]       = DS18B20 Temperature Sensor Signed 16Bit(Signed/10)
      // Input[4]       = BH1750 Light Sensor Signed 16Bit(Signed/10)
      // Input[5:6]     = SHT31 Temperature Sensor Float 32 Bit(Big-endian)
      // Input[7:8]     = SHT31 Humidity Sensor Float 32Bit(Big-endian)
      // Input[9:10]    = DS18B20 Temperature Sensor Float 32 Bit(Big-endian)
      // Input[11:12]   = BH1750 Light Sensor Float 32 Bit(Big-endian)
      //===========================================================================================
      //===========================================================================================
      read_modbus_status = nodeSensorWeather.readInputRegisters(0x0001, 4);                       // 4-InputRegister Read(Input[1..4]
      //===========================================================================================
      if(read_modbus_status == nodeSensorWeather.ku8MBSuccess)
      {
        //=========================================================================================
        sht31_temperature = (int16_t)nodeSensorWeather.getResponseBuffer(0)/10;                   // SHT31 Temperature   
        sht31_humidity = (int16_t)nodeSensorWeather.getResponseBuffer(1)/10;                      // SHT31 Humidity   
        bh1750_lux = (int16_t)nodeSensorWeather.getResponseBuffer(3)/10;                          // BH1750 Light   
        //=========================================================================================
        SerialDebug.print("SHT31 Temperature = ");
        SerialDebug.print(sht31_temperature,1);
        SerialDebug.print(" : Humidity = ");
        SerialDebug.print(sht31_humidity,1);
        SerialDebug.print(" : BH1750 Lux = ");
        SerialDebug.println(bh1750_lux,1);
        //=========================================================================================
      }
    }
    else
    {
      //===========================================================================================
      // Moisture Sensor : SOIL MOISTURE H/T/EC MODBUS RTU
      //===========================================================================================
      // Holding[512] = Slave ID
      // Input[0]     = Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C)
      // Input[1]     = Soil Moisture/100(%H)   -> 0 - 10000 : 0 - 100(%H)
      // Input[2]     = Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
      //===========================================================================================
      read_modbus_status = nodeSoilMoisture.readInputRegisters(0x0000, 3);                        // 3-InputRegister Read(Input[0..2])
      //===========================================================================================
      if(read_modbus_status == nodeSoilMoisture.ku8MBSuccess)
      {
        //=========================================================================================
        soil_temperature = (int16_t)nodeSoilMoisture.getResponseBuffer(0)/100;                    // Soil Temperature/100(C) -> -4000 to +8000 : -40.0(C) to +80.0(C) 
        soil_moisture = (uint16_t)nodeSoilMoisture.getResponseBuffer(1)/100;                      // Soil Moisture/100(%RH)  -> 0 - 10000 : 0 - 100(%RH) 
        soil_ec = (uint16_t)nodeSoilMoisture.getResponseBuffer(2);                                // Soil EC(uS/cm)          -> 0 - 20000 : 0-20000(uS/cm)
        //=========================================================================================
        SerialDebug.print("Soil Temperature = ");
        SerialDebug.print(soil_temperature,1);
        SerialDebug.print(" : Soil Moisture = ");
        SerialDebug.print(soil_moisture,1);
        SerialDebug.print(" : Soil EC = ");
        SerialDebug.println(soil_ec,1);
        SerialDebug.println();
        //=========================================================================================
      }
      //===========================================================================================
    }
    //=============================================================================================
    // End of Read Modbus Sensor  
    //=============================================================================================
    
    //=============================================================================================
    OffLed();  
    OffBuzzer();
    //=============================================================================================
    lastGetModbusTime = millis();
    //=============================================================================================
  }
  //===============================================================================================
}



