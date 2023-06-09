/*
 * File    : ETT_PCF8574.H
 * Author  : ETT CO.,LTD 
 *         : Modify From https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
 * Update  : October 2018        
 * Purpose : Library PCF8574/A For I2C Interface Arduino
 * Support : ESP32
 *         : ESP8266
 *         : MEGA32U4
 */
 
//=================================================================================
#include "ETT_PCF8574.h"
#include <Wire.h>
//=================================================================================

//=================================================================================
//=================================================================================
ETT_PCF8574::ETT_PCF8574(const uint8_t deviceAddress)
{
  //===============================================================================
  _address    = deviceAddress;
  _dataIn     = 0xFF;
  _dataOut    = 0xFF;
  _error      = PCF8574_OK;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Initial PCF8574/A
//=================================================================================
// Input : val = Begin Port Statte Value
//=================================================================================
void ETT_PCF8574::begin(uint8_t val)
{
  //===============================================================================
  ETT_PCF8574::writePort(val);                                                   // val = _dataOut -> Port
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Read PCF8574/A Single Pin
//=================================================================================
// Input    : pin(0...7)
// Output   : pin State(0,1)
// Variable : _dataIn -> Change
//=================================================================================
uint8_t ETT_PCF8574::readPin(const uint8_t pin)
{
  //===============================================================================
  if(pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return 0;
  }
  //===============================================================================
  ETT_PCF8574::readPort();                                                       // _dataIn <- Port
  //===============================================================================
  return (_dataIn & (1 << pin)) > 0;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Write PCF8574/A Single Pin
//=================================================================================
// Input    : pin(0..7)
//          : value(0,1)
// Variable : _dataOut -> Change
//=================================================================================
void ETT_PCF8574::writePin(const uint8_t pin, const uint8_t value)
{
  //===============================================================================
  if(pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return;
  }
  if (value == LOW)
  {
    _dataOut &= ~(1 << pin);
  }
  else
  {
    _dataOut |= (1 << pin);
  }
  //===============================================================================
  writePort(_dataOut);                                                           // _dataOut -> Port
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Read PCF8574/A Port
//=================================================================================
// Input    : none
// Output   : Pin/Port State(0,1)
// Variable : _dataIn -> Change
//=================================================================================
uint8_t ETT_PCF8574::readPort()
{
  //===============================================================================
  if(Wire.requestFrom(_address, (uint8_t)1) != 1)
  {
    _error = PCF8574_I2C_ERROR;
    return _dataIn;                                                              // last value
  }
  //===============================================================================
#if (ARDUINO <  100)
    _dataIn = Wire.receive();
#else
    _dataIn = Wire.read();
#endif
  //===============================================================================
  return _dataIn;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Write PCF8574/A Port
//=================================================================================
// Input    : value -> Port State
// Output   : none
// Variable : _dataOut -> Change
//=================================================================================
void ETT_PCF8574::writePort(const uint8_t value)
{
  //===============================================================================
  _dataOut = value;
  //===============================================================================
  Wire.beginTransmission(_address);
  Wire.write(_dataOut);
  _error = Wire.endTransmission();
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Toggle PCF8574/A Single Pin 
//=================================================================================
// Input    : pin(0..7)
// Output   : none
// Variable : _dataOut -> Change
//=================================================================================
void ETT_PCF8574::togglePin(const uint8_t pin)
{
  //===============================================================================
  if(pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return;
  }
  //===============================================================================
  togglePinMask(1 << pin);                                                       // Update Pin State 
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Toggle Pin/Port(Toggle only Pin Position Mask(1) of Pin/Port)
//=================================================================================
// Input    : pin(0..7)
// Output   : none
// Variable : _dataOut -> Change
//=================================================================================
// Example  : Toggle Pin-P0          -> toggleMask(0x01)
//          : Toggle Pin-P1          -> toggleMask(0x02)
//          :      ...                        ...
//          : Toggle Pin-P7          -> toggleMask(0x80)
//          : Toggle All Port(8 Bit) -> toggleMask(0xFF)
//=================================================================================
void ETT_PCF8574::togglePinMask(const uint8_t mask)
{
  //===============================================================================
  _dataOut ^= mask;                                                              // invertAll() = toggleMask(0xFF)
  //===============================================================================
  ETT_PCF8574::writePort(_dataOut);                                              // val = _dataOut -> Port
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Read PCF8574/A Pin(1 Bit)
///================================================================================
// Input    : pin(0...7)
// Output   : pin State(0,1)
// Variable : _dataIn -> Change
//=================================================================================
uint8_t ETT_PCF8574::readButtonPin(const uint8_t pin)
{
  //===============================================================================
  uint8_t temp = _dataOut;                                                       // Save Now Port State
  uint8_t rtn;
  //===============================================================================
  if(pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return 0;
  }
  //===============================================================================
  ETT_PCF8574::writePin(pin, HIGH);                                              // Set Pin = High Before Read
  //===============================================================================
  rtn = ETT_PCF8574::readPin(pin);                                               // Read Pin
  //===============================================================================
  ETT_PCF8574::writePort(temp);                                                  // temp = _dataOut -> Port(Restore Port State Before Read)
  //===============================================================================
  return rtn;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Read PCF8574/A Button Port(8 Bit)
// Set Position Mask Pin = High Before Read
//=================================================================================
// Input    : mask(position(1) -> Input Pin Write/Read)
// Output   : Port Value
// Variable : _dataIn -> Change
//=================================================================================
// Write Pin Port of Position Mask(1) = High Before Read
// -> setButtonMask(?) With Input Position Bit = 1 Before Use This Command
// -> Pin Output Not Change
// Example P0=Out,P1=Out,P4=In,P5=In
// -> readButtonPort(00110000) -> Pin 0,1,2,3,6,7 not Write Pin = High Before Read
//=================================================================================
uint8_t ETT_PCF8574::readButtonPort(const uint8_t mask)
{
  //===============================================================================
  uint8_t temp = _dataOut;                                                        // Get Last Port State
  //===============================================================================
  ETT_PCF8574::writePort(mask | _dataOut);                                        // mask | _dataOut = _dataOut -> Port(Set Pin Read = High Before Read)
  //===============================================================================
  ETT_PCF8574::readPort();                                                        // _dataIn <- Port(Read Now Port State)
  //===============================================================================
  ETT_PCF8574::writePort(temp);                                                   // temp = _dataOut -> Port(old Port State)
  //===============================================================================
  return _dataIn;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// Get Last Error State
//=================================================================================
int ETT_PCF8574::lastError()
{
  //===============================================================================
  int e = _error;
  //===============================================================================
  _error = PCF8574_OK;
  //===============================================================================
  return e;
  //===============================================================================
}
//=================================================================================

//=================================================================================
// END OF FILE
//=================================================================================

