/*
 * CrapScope
 * Version: 0.2a
 * Authors: programagor & woodbin
 * 
 * Interfaces with TFT LCD screen, 
 * shows ADC measurements as a function of time
 * 
 * Distributed under GNU GPL v3 I guess...
*/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#define SIGNAL A5 //This is the input signal


// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

uint16_t buffer[2][320-40]; // Two buffers so that old plot can be erased
bool p; // Buffer selector


float scale;

void setup(void) {
  Serial.begin(9600);
    init_tft();
Serial.println(RED);
Serial.println(CYAN);
Serial.println(GREEN);


scale=1.0; 



}

void loop(void) {
  p=!p;
  /* Read values into buffer */
  pinMode(SIGNAL, INPUT);
  for (uint16_t i=0;i<(320-40);i++){
    buffer[p][i]=analogRead(SIGNAL); //  * (5.0 / 1023.0)
    delayMicroseconds(1000); 
  }
  /* Draw values */
  for(int x=1;x<320-40;x++){
    tft.drawLine(x+20,(int)(240-20-buffer[!p][x-1]*(5.0/256.0)*10*scale+(scale-1)*98),x+21,(int)(240-20-buffer[!p][x]*(5.0/256.0)*10*scale+(scale-1)*98),BLACK);
  }

draw_grid();
  
  for(int x=1;x<320-40;x++){
    tft.drawLine(x+20,(int)(240-20-buffer[p][x-1]*(5.0/256.0)*10*scale+(scale-1)*98),x+21,(int)(240-20-buffer[p][x]*(5.0/256.0)*10*scale+(scale-1)*98),GREEN);
  }
  //delay(250);
}


void init_tft(){
 tft.reset();

  uint16_t identifier = tft.readID();

  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    identifier = 0x9341;
  }

  tft.begin(identifier);
  tft.setRotation(3);  
  tft.fillScreen(BLACK);

tft.setCursor(0, 0);
tft.setTextColor(GREEN);
tft.setTextSize(2);
tft.println("CrapScope ver:0.2a");
tft.setTextSize(1);
tft.setCursor(180, 230);
tft.println("programagor & woodbin");
}

void draw_grid(){
  int w = tft.width() , h = tft.height();
  tft.drawFastHLine(20, h/2 , w-40, 0x0000ff);
  tft.drawFastHLine(20, 20  , w-40, 0x0000ff);
  tft.drawFastHLine(20, h-20, w-40, 0x0000ff);
  for(int x=20; x<=w-20; x+=(w-40)/8) tft.drawFastVLine(x, 20, h-40, 0x0000ff);
}

