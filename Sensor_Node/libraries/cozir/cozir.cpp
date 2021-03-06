// 
//       FILE: Cozir.cpp
//     AUTHOR: Rob Tillaart & Michael Hawthorne
//    VERSION: 0.1.01
//    PURPOSE: library for COZIR range of sensors for Arduino
//        URL: http://www.cozir.com/
//  DATASHEET: http://www.co2meters.com/Documentation/Datasheets/COZIR-Data-Sheet-RevC.pdf
// USER GUIDE: http://www.co2meters.com/Documentation/Manuals/COZIR-Software-User-Guide-AL12-RevA.pdf
// READ DATASHEET BEFORE USE OF THIS LIB !
//
// Released to the public domain
//

#include "Cozir.h"
#include "SoftwareSerial.h"

////////////////////////////////////////////////////////////
//
// CONSTRUCTOR
//
COZIR::COZIR(SoftwareSerial& nss) : CZR_Serial(nss)
{
  CZR_Serial.begin(9600);
}

////////////////////////////////////////////////////////////
//
// OPERATING MODE
//
// note: use CZR_COMMAND to minimize power consumption
// CZR_POLLING and CZR_STREAMING use an equally amount 
// of power as both sample continuously...
//
void COZIR::SetOperatingMode(uint8_t mode)
{
  sprintf(buffer, "K %u", mode);
  Command(buffer);
}



////////////////////////////////////////////////////////////
//
// POLLING MODE
//
// you need to set the polling mode explicitely before 
// using these functions. SetOperatingMode(CZR_POLLING);
// this is the default behaviour of this Class but
// not of the sensor!!
//
float COZIR::Fahrenheit()
{
  return (Celsius() * 1.8) + 32;
}

float COZIR::Celsius()
{
  float f = 0;
  uint16_t rv = Request("T");
  if (rv < 1000)
  {
      f = 0.1 * rv;
  }
  else
  {
    f = -0.1 * (rv-1000);
  }
  return f;
}

float COZIR::Humidity()
{
  return 0.1 * Request("H");
}

// TODO UNITS UNKNOWN
float COZIR::Light()
{
  return 1.0 * Request("L");
}

uint16_t COZIR::CO2()
{
  return Request("Z");
}

// CALLIBRATION - USE THESE WITH CARE
// use these only in pollingmode (on the Arduino)

// FineTuneZeroPoint()
// a reading of v1 will be reported as v2
// sort of mapping
// check datasheet for detailed description
uint16_t COZIR::FineTuneZeroPoint(uint16_t v1, uint16_t v2)
{
  sprintf(buffer, "F %u %u", v1, v2);
  return Request(buffer);
}

// mostly the default calibrator
uint16_t COZIR::CalibrateFreshAir()
{
  return Request("G");
}

uint16_t COZIR::CalibrateNitrogen()
{
  return Request("U");
}

uint16_t COZIR::CalibrateKnownGas(uint16_t value)
{
  sprintf(buffer, "X %u", value);
  return Request(buffer); 
}

// NOT RECOMMENDED, see datasheet
uint16_t COZIR::CalibrateManual(uint16_t value)
{
  return 0;
  //sprintf(buffer, "u %u", value);
  //return Request(buffer); 
}

// NOT RECOMMENDED, see datasheet
uint16_t COZIR::SetSpanCalibrate(uint16_t value)
{
  return 0;
  //sprintf(buffer, "S %u", value);
  //return Request(buffer);
}

// NOT RECOMMENDED, see datasheet
uint16_t COZIR::GetSpanCalibrate()
{
  return Request("s");
}

// DIGIFILTER, use with care
// default value = 32,
// 1=fast (noisy) 255=slow (smoothed)
// 0 = special. details see datasheet
void COZIR::SetDigiFilter(uint8_t value)
{
  sprintf(buffer, "A %u", value);
  Command(buffer);
}

uint8_t COZIR::GetDigiFilter()
{
  return Request("a");
}



////////////////////////////////////////////////////////////
//
// STREAMING MODE
//
// outputfields should be OR-ed
// e.g. SetOutputFields(CZR_HUMIDITY | CZR_RAWTEMP | CZR_RAWCO2);
//
// you need to set the STREAMING mode explicitely
// SetOperatingMode(CZR_STREAMING);
//
// in STREAMING mode you must parse the output of serial yourself
//
void COZIR::SetOutputFields(uint16_t fields)
{
  sprintf(buffer, "M %u", fields);
  Command(buffer);
}

// For Arduino you must read the serial yourself as
// the internal buffer of this Class cannot handle
// large output - can be > 100 bytes!!
void COZIR::GetRecentFields()
{
  Command("Q");
}


////////////////////////////////////////////////////////////
//
// EEPROM - USE WITH CARE 
//
// SEE DATASHEET 7.2 EEPROM FOR DETAILS
//
// TODO 
// - defines for addresses 
// - do HILO values in one call
//
void COZIR::SetEEPROM(uint8_t address, uint8_t value)
{
  sprintf(buffer, "P %u %u", address, value); 
  Command(buffer);
}

uint8_t COZIR::GetEEPROM(uint8_t address)
{
  sprintf(buffer, "p %u", address);
  return Request(buffer);
}



////////////////////////////////////////////////////////////
//
// COMMAND MODE
//
// read serial yourself
//
void COZIR::GetVersionSerial()
{
  Command("Y");
}

void COZIR::GetConfiguration()
{
  Command("*");
}

/////////////////////////////////////////////////////////
// PRIVATE
  
void COZIR::Command(char* s)
{
  CZR_Serial.print(s);
  CZR_Serial.print("\r\n");
}

uint16_t COZIR::Request(char* s)
{
  Command(s);
  // empty buffer
  buffer[0] = '\0';
  // read answer; there may be a 100ms delay!
  // TODO: PROPER TIMEOUT CODE.
  delay(150);  
  int idx = 0;
  while(CZR_Serial.available())
  {
	buffer[idx++] = CZR_Serial.read();
  }
  buffer[idx] = '\0';
  uint16_t rv = 0;
  switch(buffer[1])
  {
    case 'T' :
            rv = atoi(&buffer[5]);
            if (buffer[4] == 1) rv += 1000;
            break;
    default :
            rv = atoi(&buffer[2]);
            break;
  }
  return rv;
}
