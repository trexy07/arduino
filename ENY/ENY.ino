/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor. Designed specifically to work with the Adafruit BMP388 Breakout ----> http://www.adafruit.com/products/3966
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required to interface. Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries. BSD license, all text above must be included in any redistribution
 ***************************************************************************/

// Complete project details: https://RandomNerdTutorials.com/arduino-bmp388/


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
// float baseAlt = 0.0;
// float lastAlt = 0.0;

// const int trigPin = 9;
// const int echoPin = 10;

Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Adafruit BMP388 / BMP390 test");


    // bmp388
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
        //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1);
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // \bmp388

    // button
    pinMode(2, INPUT_PULLUP);

    // \button

    // USR
    // pinMode(trigPin, OUTPUT);
    // pinMode(echoPin, INPUT);
    // \USR


}

void loop() {




    if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }


    Serial.print("Temperature = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");


    if (digitalRead(2) == LOW) {
        baseAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        Serial.println("Ground altitude set!");
    }

    Serial.print("sea Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    float groundAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA)- baseAlt;
    Serial.print("ground altitude = ");
    Serial.print(groundAlt);
    Serial.println(" m");

    // vertical speed
    float verticalSpeed = (groundAlt - lastAlt)/2;
    Serial.print("Vertical velocity = ");
    Serial.print(verticalSpeed);
    Serial.println(" m/s");
    lastAlt = groundAlt;
    // if (groundAlt < 1 && abs(verticalSpeed)<1){// basically on ground
    // 
    // 
    // } 

    // if (groundAlt > 30 && abs(verticalSpeed)<1){// top altitude
    // 
    // 
    // } 


    Serial.println();
    delay(2000);
}
