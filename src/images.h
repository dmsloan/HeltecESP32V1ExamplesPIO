#define LoRa_Logo_width 59
#define LoRa_Logo_height 39

#define WiFi_Logo_width 60
#define WiFi_Logo_height 36

#define HelTec_LOGO_width 128
#define HelTec_LOGO_height 37

#define BT_width 8
#define BT_height 10

#define BAT_width 20
#define BAT_height 9

#define WIFI_width 14
#define WIFI_height 8

const unsigned char LoRa_Logo_bits[] PROGMEM = {
  0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x1F, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xFE, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xF1, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xF0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x1E, 
  0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0xFF, 0xC1, 0xFF, 0x07, 0x00, 0x00, 
  0x1F, 0xC0, 0xFF, 0xE7, 0xFF, 0x0F, 0x00, 0x00, 0x1F, 0xC0, 0x83, 0xE7, 
  0xFF, 0x1F, 0x00, 0x00, 0x1F, 0x00, 0x00, 0xE0, 0xFF, 0x1F, 0x70, 0x00, 
  0x1F, 0x00, 0xFE, 0xE0, 0x03, 0x1F, 0xFE, 0x01, 0x1F, 0x00, 0xFF, 0xE1, 
  0x03, 0x1F, 0xFF, 0x03, 0x1F, 0x80, 0xFF, 0xE3, 0x03, 0x9F, 0xFF, 0x07, 
  0x1F, 0xC0, 0xFF, 0xE7, 0x03, 0x9F, 0xFF, 0x07, 0x1F, 0xC0, 0xC7, 0xE7, 
  0xFF, 0x9F, 0x8F, 0x07, 0x1F, 0xC0, 0x83, 0xE7, 0xFF, 0x0F, 0xF0, 0x07, 
  0x1F, 0xC0, 0x83, 0xE7, 0xFF, 0x07, 0xFF, 0x07, 0x1F, 0xC0, 0x83, 0xE7, 
  0xFF, 0x83, 0xFF, 0x07, 0x1F, 0xC0, 0x83, 0xE7, 0xE3, 0x87, 0x9F, 0x07, 
  0x1F, 0xC0, 0x83, 0xE7, 0xE3, 0xC7, 0x87, 0x07, 0xFF, 0xCF, 0xC7, 0xE7, 
  0xC3, 0xCF, 0xC7, 0x07, 0xFF, 0xCF, 0xFF, 0xE7, 0xC3, 0xDF, 0xFF, 0x07, 
  0xFF, 0x8F, 0xFF, 0xE3, 0x83, 0x9F, 0xFF, 0x07, 0xFF, 0x0F, 0xFF, 0xE1, 
  0x03, 0x1F, 0xFF, 0x07, 0xFF, 0x0F, 0xFE, 0xE0, 0x03, 0x1F, 0xBE, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x83, 0x07, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x1E, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xE0, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFF, 0xF1, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xFE, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x1F, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char WiFi_Logo_bits[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x07, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xE0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF,
  0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
  0xFF, 0x03, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0xFF, 0x07, 0xC0, 0x83, 0x01, 0x80, 0xFF, 0xFF, 0xFF,
  0x01, 0x00, 0x07, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x0C, 0x00,
  0xC0, 0xFF, 0xFF, 0x7C, 0x00, 0x60, 0x0C, 0x00, 0xC0, 0x31, 0x46, 0x7C,
  0xFC, 0x77, 0x08, 0x00, 0xE0, 0x23, 0xC6, 0x3C, 0xFC, 0x67, 0x18, 0x00,
  0xE0, 0x23, 0xE4, 0x3F, 0x1C, 0x00, 0x18, 0x00, 0xE0, 0x23, 0x60, 0x3C,
  0x1C, 0x70, 0x18, 0x00, 0xE0, 0x03, 0x60, 0x3C, 0x1C, 0x70, 0x18, 0x00,
  0xE0, 0x07, 0x60, 0x3C, 0xFC, 0x73, 0x18, 0x00, 0xE0, 0x87, 0x70, 0x3C,
  0xFC, 0x73, 0x18, 0x00, 0xE0, 0x87, 0x70, 0x3C, 0x1C, 0x70, 0x18, 0x00,
  0xE0, 0x87, 0x70, 0x3C, 0x1C, 0x70, 0x18, 0x00, 0xE0, 0x8F, 0x71, 0x3C,
  0x1C, 0x70, 0x18, 0x00, 0xC0, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x08, 0x00,
  0xC0, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x0C, 0x00, 0x80, 0xFF, 0xFF, 0x1F,
  0x00, 0x00, 0x06, 0x00, 0x80, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x07, 0x00,
  0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xF8, 0xFF, 0xFF,
  0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x01, 0x00, 0x00,
  0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF,
  0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0x1F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  };

const unsigned char HelTec_LOGO_bits[] PROGMEM = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x91, 0x0F, 0x11, 0xF3, 0x1E, 
  0x41, 0x08, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x92, 
  0x10, 0x91, 0x44, 0x22, 0xA1, 0x08, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0x93, 0x92, 0x20, 0x91, 0x40, 0x42, 0x81, 0x08, 0x00, 0x00, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x94, 0x40, 0x91, 0x40, 0x42, 
  0x91, 0x08, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x80, 0x93, 0x94, 
  0x40, 0x11, 0x43, 0x22, 0x11, 0x09, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 
  0x00, 0x80, 0x93, 0x94, 0x40, 0x11, 0x44, 0x12, 0xF9, 0x09, 0x00, 0x00, 
  0x0F, 0x00, 0x00, 0x00, 0x00, 0x80, 0x93, 0x98, 0x20, 0x11, 0x44, 0x12, 
  0x09, 0x09, 0x00, 0x00, 0xCF, 0xFF, 0x00, 0x00, 0x00, 0x80, 0x93, 0x90, 
  0x10, 0x91, 0x44, 0x22, 0x05, 0x0A, 0x00, 0x00, 0xCF, 0xFF, 0x00, 0x00, 
  0x00, 0x80, 0x93, 0x90, 0x0F, 0x0E, 0x43, 0x42, 0x05, 0xFA, 0x00, 0x00, 
  0xCF, 0xFF, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xCF, 0xFF, 0x00, 0x00, 0x00, 0x80, 0xF3, 0x60, 
  0x7C, 0xEF, 0x39, 0x44, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF3, 0x3F, 0x00, 
  0x00, 0x80, 0x13, 0x61, 0x10, 0x22, 0x48, 0x44, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xF3, 0x3F, 0x00, 0x00, 0x80, 0x13, 0x91, 0x10, 0x22, 0x88, 0x28, 
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xF3, 0x3F, 0x00, 0x00, 0x80, 0xF3, 0x90, 
  0x10, 0xE2, 0x89, 0x28, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF3, 0x3F, 0x00, 
  0x00, 0x80, 0x13, 0xF9, 0x11, 0x22, 0x28, 0x10, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x38, 0x00, 0x00, 0x80, 0x13, 0x09, 0x11, 0x22, 0x48, 0x10, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x80, 0x13, 0x05, 
  0x12, 0x22, 0x48, 0x10, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF3, 0x3F, 0x00, 
  0x00, 0x80, 0xF3, 0x04, 0x12, 0xE2, 0x89, 0x10, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xF3, 0x3F, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xF3, 0x3F, 0x00, 0x00, 0x80, 0xF3, 0x0D, 
  0xE1, 0xA1, 0xA1, 0xCF, 0xF7, 0x68, 0x08, 0x0F, 0xFF, 0xF3, 0x3F, 0x00, 
  0x00, 0x80, 0x13, 0x14, 0x11, 0xA6, 0xA2, 0x40, 0x10, 0xAB, 0x88, 0x30, 
  0xFF, 0xF3, 0x3F, 0x00, 0x00, 0x80, 0x13, 0x14, 0x09, 0xA0, 0xA2, 0x40, 
  0x10, 0xAA, 0x48, 0x00, 0xCF, 0xF3, 0x00, 0xF0, 0xFF, 0xFF, 0x13, 0x24, 
  0x09, 0xA0, 0xA4, 0x40, 0x10, 0x2A, 0x49, 0x00, 0xCF, 0xFF, 0x00, 0xF0, 
  0xFF, 0xFF, 0xF3, 0x25, 0xC9, 0xA7, 0xA4, 0xCF, 0x97, 0x29, 0x49, 0x7E, 
  0xCF, 0xFF, 0x00, 0xF0, 0xFF, 0xFF, 0x13, 0x44, 0x09, 0xA4, 0xA8, 0x40, 
  0x10, 0x28, 0x4A, 0x40, 0xCF, 0xFF, 0x00, 0xF0, 0xFF, 0xFF, 0x13, 0x44, 
  0x09, 0xA4, 0xA8, 0x40, 0x90, 0x28, 0x8A, 0x20, 0xCF, 0xFF, 0x00, 0x70, 
  0x00, 0x00, 0x10, 0x84, 0x31, 0xA2, 0xB0, 0x40, 0x10, 0x29, 0x8C, 0x11, 
  0x0F, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0xF3, 0x05, 0xC1, 0xA1, 0xA0, 0xCF, 
  0x17, 0x2A, 0x08, 0x0E, 0x0F, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0x03, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xF0, 
  0xFF, 0xFF, 0x93, 0x11, 0x4E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x0F, 0x00, 0x00, 0xF0, 0xFF, 0xFF, 0x93, 0x12, 0x31, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x80, 0x93, 0x92, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 
  0x00, 0x80, 0x93, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x9C, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x98, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0x93, 0x10, 0x43, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x93, 0x10, 0x1C, 0x02, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00
  };

const unsigned char BT_bits[] PROGMEM = {
  0x18, 0x28, 0x4A, 0x2C, 0x18, 0x2C, 0x4A, 0x28, 0x18, 0x00,
  };

const unsigned char BAT_bits[] PROGMEM = {
  0xFC, 0xFF, 0x0F, 0x04, 0x00, 0x08, 0xF7, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 
  0xF1, 0xDE, 0x0B, 0xF1, 0xDE, 0x0B, 0xF7, 0xDE, 0x0B, 0x04, 0x00, 0x08, 
  0xFC, 0xFF, 0x0F,
  };

const unsigned char WIFI_bits[] PROGMEM = {
  0xF0, 0x03, 0x04, 0x08, 0xF2, 0x13, 0x09, 0x24, 0xE4, 0x09, 0x10, 0x02, 
  0xC0, 0x00, 0xC0, 0x00,
  };


//屏幕下方的小圆点
const unsigned char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const unsigned char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

//Added by Sloeber 
#pragma once


//Added by Sloeber 
#pragma once

