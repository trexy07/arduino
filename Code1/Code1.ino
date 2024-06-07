//Warning! If you use ArduinoISP programmer. Default ArduinoISP sketch too fast for attiny13 @ 128 or 600 KHz ("Invalid device signature" error). 
//To fix that, in ArduinoISP sketch change string: #define SPI_CLOCK (1000000/6) with: #define SPI_CLOCK (128000/6)
// Installation of at13-master.zip hardware (boards):
//for OS X: Copy attiny13 directory into /Applications/Arduino.app/Contents/Java/hardware/
//for Windows:Copy attiny13 directory into C:\arduino\arduino-\<version\>\hardware\

#include "Arduino.h"
//const int
//unsigned char Led1Out = 3;              // Define the indicator LED pin digital pin 13
unsigned char StepUpEn_Out = 4;           // Define the StepUpEn_Out pin digital pin 4
unsigned char Button_In = 3;              // Define the Button_In pin digital pin 3
unsigned char FAN_Out = 0;                // Define the FAN_Out pin digital pin 0 (PWM OC0A)
//#define StepUpEn_Out  4           // Define the StepUpEn_Out pin digital pin 4
//#define Button_In  3              // Define the Button_In pin digital pin 3
//#define FAN_Out  0                // Define the FAN_Out pin digital pin 0 (PWM OC0A)

// //char PWMlevel[5] = {0,25,50,75,100};        //PWM levels in %
// char PWMlevel[5] = {0,64,128,192,255};        //PWM levels in LSB (calculate yourself by *2.55 to save tiny FLASH of this MCU)
//char PWMlevel[4] = {0,50,75,100};        //PWM levels in %
char PWMlevel[4] = {0,128,192,255};        //PWM levels in LSB (calculate yourself by *2.55 to save tiny FLASH of this MCU)
char level_now = 0;                           //variable with PWM level in cell 0 = 0%
int OutputLSB=0;                              //variable with PWM level in LSB new calculation
int OutputLSBwas=0;                           //variable with PWM level in LSB was last time
char Button_counter=0;                        //variable to check how long button pressed


// the setup routine runs once when you press reset:
void setup() {

     pinMode(StepUpEn_Out, OUTPUT);
     pinMode(FAN_Out, OUTPUT);      
     pinMode(Button_In, INPUT);

//OutputLSB=0;

#ifndef InvertOutput
//TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);//Clear OC2A,OC2B on compare match //FAST PWM mode
//TCCR2B = _BV(CS22); //p131 //64prescaler fromT2!
//TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);//Clear OC1A,OC1B on compare match //FAST PWM mode
//TCCR1B = _BV(CS11); //p110 //256prescaler
TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);//Clear OC0A,OC0B on compare match //FAST PWM mode
TCCR0B = _BV(CS01) | _BV(CS00); //p87 //256prescaler
#else
//TCCR2A = _BV(COM2A1) |_BV(COM2A0) | _BV(COM2B1) |_BV(COM2B0) | _BV(WGM21) | _BV(WGM20);//Set OC2A,OC2B on compare match //FAST PWM mode
//TCCR2B = _BV(CS22); //p131 //64prescaler fromT2!
//TCCR1A = _BV(COM1A1) |_BV(COM1A0) | _BV(COM1B1) |_BV(COM1B0) | _BV(WGM11) | _BV(WGM10);//Set OC1A,OC1B on compare match //FAST PWM mode
//TCCR1B = _BV(CS11); //p110 //256prescaler
TCCR0A = _BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) |_BV(COM0B0) | _BV(WGM01) | _BV(WGM00);//Set OC0A,OC0B on compare match //FAST PWM mode
TCCR0B = _BV(CS01) | _BV(CS00); //p87 //256prescaler
#endif

OCR0A = (OutputLSB);
}

void loop() {// the loop routine runs over and over again forever:
    
    if(digitalRead(Button_In) == 0){
      Button_counter++;
    }else{

                if(Button_counter>7){
                    //Button_counter=10; 
                    level_now++;
                }    //change "10" to bigger value if you need longer press

      Button_counter=0;
    };

    if(level_now>3){level_now=0;};

    if(level_now==0){  digitalWrite(StepUpEn_Out, LOW); //Disable StepUp converter circuit
    }else{digitalWrite(StepUpEn_Out, HIGH); }           //Enable StepUp converter circuit

//level_now=3;
  
  //OutputLSB=((2.55*PWMlevel[level_now])+1);
  OutputLSB=(PWMlevel[level_now]);

  if(OutputLSB!=OutputLSBwas){
      OCR0A = (OutputLSB);
      //OCR0A = (PWMlevel[level_now]);
  OutputLSBwas=OutputLSB;
  }
            delay(10); // 20 ms to 200ms
            //delay(2000);
            //level_now++;
}
