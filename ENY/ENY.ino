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

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

#define rate 60 // speed of the servo
#define released  0 // released camera
#define pressed  60 // pressign camera
#define trigger  0.25 // 0.25 m/s is variability of pressure reading


// pressure variables
Adafruit_BMP3XX bmp;
// float baseAlt = 0.0;
float notBaseAlt = 0.0;
float lastAlt = 0.0;

// servo variables
bool camera = true;
float angle = 0;
int direction = 1;




void setup() {
    Serial.begin(9600);
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
    // pinMode(2, INPUT_PULLUP);

    // \button
    // signal out
    pinMode(10, OUTPUT);
    // \signal out

    // USR
    // pinMode(trigPin, OUTPUT);
    // pinMode(echoPin, INPUT);
    // \USR

    // ignoring the first 2 readings
    myservo.attach(3);
    notBaseAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("not base altitude = ");
    Serial.print(notBaseAlt);
    Serial.println(" m");
    delay(1000);
    notBaseAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("probably not base altitude = ");
    Serial.print(notBaseAlt);
    Serial.println(" m");
    delay(1000);
    // myservo.write(released);

    // the altitude at the ground
    notBaseAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("base altitude = ");
    Serial.print(notBaseAlt);
    Serial.println(" m");


}

void loop() {




    if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        // return;
    }


    Serial.print("Pressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");


    float seaAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("sea Altitude = ");
    Serial.print(seaAlt);
    Serial.println(" m");

    // if (digitalRead(2) == LOW) {
    // if (first) {
    //     first = false;
    //     notBaseAlt = seaAlt;
    //     Serial.println("Ground altitude set!");
    // }


    float groundAlt = seaAlt - notBaseAlt;
    Serial.print("ground altitude = ");
    Serial.print(groundAlt);
    Serial.println(" m");

    // vertical speed
    float verticalSpeed = (groundAlt - lastAlt)/.500;
    Serial.print("Vertical velocity = ");
    Serial.print(verticalSpeed);
    Serial.println(" m/s");
    lastAlt = groundAlt;
    

    // logic for shuttering the camera
    if (abs(verticalSpeed)<trigger){
        // basically at rest (pressure changes relatively slowly)
        digitalWrite(10, LOW);
        camera = false;
    // } else if (groundAlt > 1 && verticalSpeed < trigger){
    //     // basically falling (pressure increases when pressure is above the base pressure)
    } else if (verticalSpeed < trigger){
        // basically falling (pressure increases)
        camera = true;
        digitalWrite(10, HIGH);
    } else{  
        // going up, falling below 1m
        camera = false;  
        digitalWrite(10, LOW);
    }


    // press the camera shutter button
    myservo.write(released);
    if (camera){
        angle += direction * rate;
        if (angle > pressed || angle < released ){
            direction *= -1;
            angle += direction * rate;
        }
        myservo.write(angle);
    }


    Serial.println("---------------");
    delay(500);
}
