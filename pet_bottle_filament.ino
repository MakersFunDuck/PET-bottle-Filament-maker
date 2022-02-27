/*
 * PID controller for PET bottle fimalent maker  by Maker's Fun Duck
 * Below I am respecting the copyright notification of the library creator's request. For the rest of the code, you are free to do anything you want. 
 * Copyright (c) 2016 Gianni Van Hoecke <gianni.vh@gmail.com>
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * User: Gianni Van Hoecke <gianni.vh@gmail.com>
 * Date: 23/05/16
 * Time: 20:19
 *
 * SmoothThermistor (https://github.com/giannivh/SmoothThermistor)
 * A flexible thermistor reading library.
 *
 * The components:
 * - Thermistor (here a 100K thermistor is used)
 * - Resistor (here a 100K resistor is used)
 * - Some wires
 *
 * The easy circuit:
 *
 *                  Analog pin 0
 *                        |
 *    5V |-----/\/\/\-----+-----/\/\/\-----| GND
 *
 *               ^                ^ 
 *        100K thermistor     100K resistor
 *
 * The advanced circuit:
 *
 *          AREF      Analog pin 0
 *           |              |
 *    3.3V |-+---/\/\/\-----+-----/\/\/\-----| GND
 *
 *                 ^                ^ 
 *          100K thermistor     100K resistor
 */
// Libraries
#include <PIDController.h>

// Objects
PIDController pid; // Create an instance of the PID controller class, called "pid"

//thermistor Temperature reading constants
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

/*    i2c LCD Module  ==>   Arduino
 *    SCL             ==>     A5
 *    SDA             ==>     A4
 *    Vcc             ==>     Vcc (5v)
 *    Gnd             ==>     Gnd      */

//I/O
int PWM_pin = 3;  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int button1 = 8;      //Pin 1 from rotary encoder
int button2 = 9;     //Pin 2 from rotary encoder
int button1State = 0;         // variable for reading the pushbutton status
int button2State = 0;         // variable for reading the pushbutton status

//Variables
float set_temperature =253 ;            //Default temperature setpoint. Leave it 0 and control it with button

float temperature_read = 0.0;



void setup() {

    Serial.begin(9600);   // Some methods require the Serial.begin() method to be called fitst



  
  pinMode(PWM_pin,OUTPUT);
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
    pid.begin();          // initialize the PID instance
  pid.setpoint(set_temperature);    // The "goal" the PID controller tries to "reach"
  pid.tune(1, 1, 1);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!


  // use the AREF pin, so we can measure on 3.3v, which has less noise on an Arduino
  // make sure your thermistor is fed using 3.3v, along with the AREF pin
  // so the 3.3v output pin goes to the AREF pin and the thermistor
  // see "the advanced circuit" on top of this sketch
//smoothThermistor.useAREF(true);  



  lcd.init();
  lcd.backlight();
}

void loop() {
 
   // First we read the real value of temperature and print it to the serial
  Vo  = analogRead(A0);

 R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); //Kelvin temp
  temperature_read  = T - 273.15; //Celcius Temp





  
  // read the state of the pushbutton value:
  button1State = digitalRead(  button1);

  button2State = digitalRead(  button2);

  delay(250); //Refresh rate + delay of LCD print
  //lcd.clear();

  
  if (button1State == LOW)
  {    

 set_temperature = set_temperature + 1;  
  delay(250);
  
  }


if (button2State == LOW)
  {    
      
  set_temperature = set_temperature - 1; 

  delay(250);  
  
  }

  lcd.setCursor(0,0);
  lcd.print("Maker's Fun Duck");
  lcd.setCursor(0,1);
  lcd.print("ST:");
  lcd.setCursor(2,1);
  lcd.print(set_temperature,1);
  lcd.setCursor(9,1);
  lcd.print("RT:");
  lcd.setCursor(11,1);
  lcd.print(temperature_read,1);
Serial.println(set_temperature);

  int output = pid.compute(temperature_read);    // Let the PID compute the value, returns the optimal output
//  analogWrite(PWM_pin, output);           // Write the output to the output pin




if (set_temperature +1 > temperature_read)
{
  
  digitalWrite(PWM_pin , HIGH);
  }

else
{
   digitalWrite(PWM_pin , LOW);
  }



}
