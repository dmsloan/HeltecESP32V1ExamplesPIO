/*
 * HelTec Automation(TM) ESP32 Series Dev boards OLED Drawing Function test code
 *
 * - Some OLED Drawing Function function test;
 *
 * by lxyzn from HelTec AutoMation, ChengDu, China
 * 
 * www.heltec.cn
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
*/


// This example just provide basic function test;
// For more informations, please vist www.heltec.cn or mail to support@heltec.cn

#include "Arduino.h"
#include <SparkFunBME280.h>
BME280 mySensor;
BME280 mySensor1;

#include "heltec.h"

// Constants that appear in the serial message.
const String mDELIMETER = "";    // cordoba add-in expects a comma delimeted string, added for connection to Excel
const int delayRead = 1000;      // Delay between readings

float pressure_correction_offset = 0.0;

//Given a value, print it in HEX with leading 0x and any leading 0s
void printyPrintHex(byte value)
{
  Serial.print("0x");
  if (value < 0x10) Serial.print("0");
  Serial.println(value, HEX);
}

void printRegisters()
{
Serial.print("ID(0xD0): ");
  printyPrintHex(mySensor.readRegister(BME280_CHIP_ID_REG));

  Serial.print("Reset register(0xE0): ");
  printyPrintHex(mySensor.readRegister(BME280_RST_REG));

  Serial.print("ctrl_meas(0xF4): ");
  printyPrintHex(mySensor.readRegister(BME280_CTRL_MEAS_REG));

  Serial.print("ctrl_hum(0xF2): ");
  printyPrintHex(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG));

  Serial.println();

  Serial.println("Displaying all regs:");
  byte memCounter = 0x80;
  for (byte row = 8 ; row < 16 ; row++)
  {
    Serial.print("0x");
    Serial.print(row, HEX);
    Serial.print("0:");

    for (byte column = 0 ; column < 16 ; column++)
    {
      byte tempReadData = mySensor.readRegister(memCounter);

      if(tempReadData < 0x10) Serial.print("0");
      Serial.print(tempReadData, HEX);
      Serial.print(" ");

      memCounter++;
    }
    Serial.println();
  }

  Serial.println();

  Serial.println("Displaying concatenated calibration words:");
  Serial.print("dig_T1, uint16: ");
  Serial.println(mySensor.calibration.dig_T1);
  Serial.print("dig_T2, int16: ");
  Serial.println(mySensor.calibration.dig_T2);
  Serial.print("dig_T3, int16: ");
  Serial.println(mySensor.calibration.dig_T3);

  Serial.print("dig_P1, uint16: ");
  Serial.println(mySensor.calibration.dig_P1);
  Serial.print("dig_P2, int16: ");
  Serial.println(mySensor.calibration.dig_P2);
  Serial.print("dig_P3, int16: ");
  Serial.println(mySensor.calibration.dig_P3);
  Serial.print("dig_P4, int16: ");
  Serial.println(mySensor.calibration.dig_P4);
  Serial.print("dig_P5, int16: ");
  Serial.println(mySensor.calibration.dig_P5);
  Serial.print("dig_P6, int16: ");
  Serial.println(mySensor.calibration.dig_P6);
  Serial.print("dig_P7, int16: ");
  Serial.println(mySensor.calibration.dig_P7);
  Serial.print("dig_P8, int16: ");
  Serial.println(mySensor.calibration.dig_P8);
  Serial.print("dig_P9, int16: ");
  Serial.println(mySensor.calibration.dig_P9);

  Serial.print("dig_H1, uint8: ");
  Serial.println(mySensor.calibration.dig_H1);
  Serial.print("dig_H2, int16: ");
  Serial.println(mySensor.calibration.dig_H2);
  Serial.print("dig_H3, uint8: ");
  Serial.println(mySensor.calibration.dig_H3);
  Serial.print("dig_H4, int16: ");
  Serial.println(mySensor.calibration.dig_H4);
  Serial.print("dig_H5, int16: ");
  Serial.println(mySensor.calibration.dig_H5);
  Serial.print("dig_H6, int8: ");
  Serial.println(mySensor.calibration.dig_H6);

  Serial.println();
  Serial.println();

  Serial.println("Reading all registers from first BME280 designated as mySensor at address 0x77");
  Serial.println("Displaying concatenated calibration words:");
  Serial.print("dig_T1, uint16: ");
  Serial.println(mySensor1.calibration.dig_T1);
  Serial.print("dig_T2, int16: ");
  Serial.println(mySensor1.calibration.dig_T2);
  Serial.print("dig_T3, int16: ");
  Serial.println(mySensor1.calibration.dig_T3);

  Serial.print("dig_P1, uint16: ");
  Serial.println(mySensor1.calibration.dig_P1);
  Serial.print("dig_P2, int16: ");
  Serial.println(mySensor1.calibration.dig_P2);
  Serial.print("dig_P3, int16: ");
  Serial.println(mySensor1.calibration.dig_P3);
  Serial.print("dig_P4, int16: ");
  Serial.println(mySensor1.calibration.dig_P4);
  Serial.print("dig_P5, int16: ");
  Serial.println(mySensor1.calibration.dig_P5);
  Serial.print("dig_P6, int16: ");
  Serial.println(mySensor1.calibration.dig_P6);
  Serial.print("dig_P7, int16: ");
  Serial.println(mySensor1.calibration.dig_P7);
  Serial.print("dig_P8, int16: ");
  Serial.println(mySensor1.calibration.dig_P8);
  Serial.print("dig_P9, int16: ");
  Serial.println(mySensor1.calibration.dig_P9);

  Serial.print("dig_H1, uint8: ");
  Serial.println(mySensor1.calibration.dig_H1);
  Serial.print("dig_H2, int16: ");
  Serial.println(mySensor1.calibration.dig_H2);
  Serial.print("dig_H3, uint8: ");
  Serial.println(mySensor1.calibration.dig_H3);
  Serial.print("dig_H4, int16: ");
  Serial.println(mySensor1.calibration.dig_H4);
  Serial.print("dig_H5, int16: ");
  Serial.println(mySensor1.calibration.dig_H5);
  Serial.print("dig_H6, int8: ");
  Serial.println(mySensor1.calibration.dig_H6);

  Serial.println();
}

void printReadings()
{
  Serial.print("Humidity: ");
  Serial.print(mDELIMETER);
  Serial.print(mySensor.readFloatHumidity(), 0);
  Serial.print(mDELIMETER);

  Serial.print(" Pressure: ");
  Serial.print(mDELIMETER);
  Serial.print(mySensor.readFloatPressure()/100, 2);
  Serial.print(" hPa");
  Serial.print(mDELIMETER);

  Serial.print(" Alt: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeFeet(), 1);
  Serial.print(mDELIMETER);

  Serial.print(" Temp: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor.readTempF(), 2);
  Serial.print(mDELIMETER);

  Serial.print("   Humidity1: ");
  Serial.print(mDELIMETER);
  Serial.print(mySensor1.readFloatHumidity(), 0);
  Serial.print(mDELIMETER);

  Serial.print(" Pressure1: ");
  Serial.print(mDELIMETER);
  Serial.print(mySensor1.readFloatPressure()/100, 2);
  Serial.print(" hPa");
  Serial.print(mDELIMETER);

  Serial.print(" Alt1: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor1.readFloatAltitudeFeet(), 1);
  Serial.print(mDELIMETER);

  Serial.print(" Temp1: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor1.readTempF(), 2);
  Serial.print(mDELIMETER);

  Serial.println();
}

void printDiff()
{
  Serial.print("Humidity Diff: ");
  Serial.print(mDELIMETER);
  Serial.print(mySensor.readFloatHumidity()-mySensor1.readFloatHumidity(), 2);
  Serial.print(mDELIMETER);

  Serial.print(" Pressure Diff: ");
  Serial.print(mDELIMETER);
  Serial.print(((mySensor.readFloatPressure()-mySensor1.readFloatPressure())/100)-pressure_correction_offset, 2);
  Serial.print(" hPa  ");
  Serial.print(mDELIMETER);

  Serial.print(((mySensor.readFloatPressure()-mySensor1.readFloatPressure())/100-pressure_correction_offset)*0.40146, 2);
  Serial.print(" inH2O ");
  Serial.print(mDELIMETER);

  Serial.print(" Alt Diff: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeFeet()-mySensor1.readFloatAltitudeFeet(), 2);
  Serial.print(mDELIMETER);

  Serial.print(" Temp Diff: ");
  Serial.print(mDELIMETER);
  //Serial.print(mySensor.readTempC(), 2);
  Serial.print(mySensor.readTempF()-mySensor1.readTempF(), 2);
  Serial.print(mDELIMETER);
  Serial.println();
  Serial.println();
}

void printFileName(void) {
  for (int16_t i=0; i<DISPLAY_HEIGHT/2; i+=1) {
    Heltec.display->drawString(0, 0, "The program is called");
    Heltec.display->drawString(0, 10, "HeltecESP32V1ExamplesPIO");
    Heltec.display->display();
    delay(10);
  }
}


// Adapted from Adafruit_SSD1306
void drawRect(void) {
  for (int16_t i=0; i<DISPLAY_HEIGHT/2; i+=1) {
    Heltec.display->drawRect(i, i, DISPLAY_WIDTH-2*i, DISPLAY_HEIGHT-2*i);
    Heltec.display->display();
    delay(10);
  }
}

// Adapted from Adafruit_SSD1306
void fillRect(void) {
  uint8_t color = 1;
  for (int16_t i=0; i<DISPLAY_HEIGHT/2; i+=3) {
    Heltec.display->setColor((color % 2 == 0) ? BLACK : WHITE); // alternate colors
    Heltec.display->fillRect(i, i, DISPLAY_WIDTH - i*2, DISPLAY_HEIGHT - i*2);
    Heltec.display->display();
    delay(10);
    color++;
  }
  // Reset back to WHITE
  Heltec.display->setColor(WHITE);
}


void setup() {
  Wire.begin(); // setup I2C wire port for for the OLED using the default pins SCL_OLED = 15 SDA_OLED = 4;
  Wire1.begin(21, 22); // setup second I2C wire port for the BME280s using SDA = 21 and SCL = 22;
  Serial.begin(115200);
  while(!Serial); //Needed for printing correctly when using a Teensy
  Serial.println("Sketch is called HeltecESP32V1ExamplesPIO");
//  Serial.println("Sketch is called ESP32BME280Example5_ReadAllRegistersPIO");
  Serial.println("Reading all registers from first BME280 designated as mySensor at address 0x76");
  
    //***BME280/BMP280 Driver settings********************************//
    //commInterface can be I2C_MODE or SPI_MODE
    //specify chipSelectPin using arduino pin names
    //specify I2C address.  Can be 0x77(default) or 0x76
  //mySensor.settings.I2CAddress = 0x76; //six lead purple board address, sensor is a BMP280 that does NOT have Humidity
  //mySensor1.settings.I2CAddress = 0x77; //six lead purple board address, sensor is a BMP280 that does NOT have Humidity
  mySensor.settings.I2CAddress = 0x76; //four lead purple board address, sensor is a BME280 that DOES have Humidity
  mySensor1.settings.I2CAddress = 0x77; //four lead purple board address, sensor is a BME280 that DOES have Humidity

if (mySensor.beginI2C(Wire1) == false) //Begin communication over I2C
  {
    Serial.println("The 0 sensor did not respond. Please check wiring.");
    while (1); //Freeze
  }

if (mySensor1.beginI2C(Wire1) == false) //Begin communication over I2C
  {
    Serial1.println("The 1 sensor did not respond. Please check wiring.");
    while (1); //Freeze
  }

Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  
Heltec.display->setContrast(255);
  
//printRegisters();
}

void loop() {   
//  printRegisters();
  printReadings();
  printDiff();

  printFileName();
  delay(1000);
  Heltec.display->clear();

  drawRect();
  delay(1000);
  Heltec.display->clear();

  fillRect();
  delay(1000);
  Heltec.display->clear();
}