//  SpincoaterV1 :  This program sends I2C commands to an Afro Electronic Speed Controller, drives a 2x20 LCD with standard parallel interface, and has a two pushbutton UI
//  It is intended to be used as a spincoater that accurately controls the speed of an inexpensive quadcopter motor and controller.
//  Firmware originally written for the Arduino Micro
//  This is basically a state machine which has the sequence Stop -> Dispense -> Coat and then returns to Stop.  The green button advances the state. When in the "Coat" state, and COAT_TIME has been reached,
//  the state will be advanced back to "Stop".  Instead of implementing a decent state machine, I'm using motor_setRPM to signal state. Evil, I know.
//  This software is in the public domain. I make no guarantees, and am not liable for any use. ben.krasnow@gmail.com
//

#include <Wire.h>
#include <Arduino_I2C_ESC.h>
#include <LiquidCrystal.h>
#include <elapsedMillis.h>


#define DISPENSE_SPEED 400
#define COAT_SPEED 3000
#define COAT_TIME 32000
const float RAMP_TIME = 8000.0;

#define ESC_ADDRESS 0x2A // Change ESC address


#define LCD_ROWS 2
#define LCD_COLUMNS 20

#define SYS_STOP 1
#define SYS_SLOW 2
#define SYS_FAST 3

#define PIN_RED_BUTTON 0
#define PIN_GRN_BUTTON 1

const float MOTOR_TRUE_RPM_FACTOR = 0.85;

const float  PID_KP = 20;
const float  PID_KI = 1.4;
const float  PID_KD = 10;
const float  PID_I_LIMIT = 20000;
const float  PID_MAX_OUTPUT = 25000;

float pid_i;
float pid_output;
float motor_lastRPM;
volatile float motor_setRPM;
float motor_actualRPM;

int motor_RPMerror;

elapsedMillis time_coating = 0;
unsigned long time_red_pressed = 0;
unsigned long time_grn_pressed = 0;
unsigned long time_last_loop = 0;


Arduino_I2C_ESC motor(ESC_ADDRESS);

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);


void setup() {
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.print("RPM");
  lcd.setCursor(0, 1);
  lcd.print("SET");
  lcd.setCursor(11, 0);
  lcd.print("TIME");

  Wire.begin();
  Serial.begin(9600);

  pinMode(PIN_RED_BUTTON, INPUT_PULLUP);
  pinMode(PIN_GRN_BUTTON, INPUT_PULLUP);

  attachInterrupt(21, isr_red_button, FALLING); //INT3 maps to PIN_RED_BUTTON
  attachInterrupt(20, isr_grn_button, FALLING); // 20 pin : SDA for mega


  motor_lastRPM = 0;
  motor_setRPM = 0;
  motor_actualRPM = 0;
  pid_i = 0.0;

  delay(2000);    //Wait for motor controller to boot
  motor.set(0);   // Motor controller must receive a "0" after power-on in order to process any throttle commands
  motor.set(0);
  motor.set(0);
  //  isr_grn_button();
  delay(500);
}

int f = 0;
void loop() {
  //  if(f == 0){
  //
  //    isr_grn_button();
  //    f = 1;
  //  }
  motor.update();  // This will receive information from the motor controller, but will not set the speed
  motor_actualRPM = motor.rpm() * MOTOR_TRUE_RPM_FACTOR;

//  motor.set(1000);

  if ( time_coating >= COAT_TIME && motor_setRPM == COAT_SPEED)
  {
    motor_setRPM = 0;
  }




  if (motor_setRPM > 0) //Only run the PID loop if we intend to drive the motor
  {
    //Run the PID loop

    if (time_coating > RAMP_TIME)
    {
      motor_RPMerror = motor_setRPM - motor_actualRPM;
    }
    else
    {
      motor_RPMerror = (time_coating / RAMP_TIME) * motor_setRPM - motor_actualRPM;
    }


    if (pid_i > PID_I_LIMIT)
    {
      pid_i =  PID_I_LIMIT - 1;
    }
    else if (pid_i < (-1 * PID_I_LIMIT))
    {
      pid_i = (-1 * PID_I_LIMIT) + 1;
    }
    else
    {
      pid_i = pid_i + float(motor_RPMerror);
    }

    pid_output = (PID_KP * motor_RPMerror) + (PID_KI * pid_i) + PID_KD * (motor_lastRPM - motor_actualRPM);
    motor_lastRPM = motor_actualRPM;

    if (pid_output < 1 ) // This is not zero, because sending a zero will stop commutation and engage max braking.  We want the motor to stay in commutation while in PID control
    {
      pid_output = 1;
    }

    if (pid_output > PID_MAX_OUTPUT)
    {
      pid_output = PID_MAX_OUTPUT;
    }
    motor.set(int(pid_output));
  }
  else  // Motor is set to zero speed, so clear the PID loop to get ready for the next acceleration
  {
    motor.set(0);
    motor_RPMerror = 0;
    pid_i = 0;
    pid_output = 0;
  }



  // Serial port debug info
  Serial.print("motor_setRPM  ");
  Serial.println(motor_setRPM);
  Serial.print("motor_actualRPM  ");
  Serial.println(motor_actualRPM);
  Serial.print("motor_RPMerror  ");
  Serial.println(motor_RPMerror);
  Serial.print("pid_i  ");
  Serial.println(pid_i);
  Serial.print("pid_output  ");
  Serial.println(pid_output);
  Serial.println(" ");



  //Update LCD
  lcd.setCursor(5, 0);
  lcd.print("      ");
  lcd.setCursor(5, 0);
  lcd.print(int(motor_actualRPM));
  Serial.println(int(motor_actualRPM));

  lcd.setCursor(5, 1);
  lcd.print("    ");
  lcd.setCursor(5, 1);
  lcd.print(int(motor_setRPM));
  Serial.println(int(motor_setRPM));

  lcd.setCursor(17, 0);
  lcd.print("   ");
  lcd.setCursor(17, 0);
  if (time_coating > 0 && motor_setRPM == COAT_SPEED)
  {
    lcd.print(int(time_coating) / 1000);
    Serial.println(int(time_coating) / 1000);
  }



  while (Serial.available())
  {
    char x = (char)Serial.read();
    if (x == '1')
    {
      motor_setRPM = 1000;
    }
    else if (x == '2')
    {
      motor_setRPM = 2000;
    }
    else if (x == '3')
    {
      motor_setRPM = 3000;
    }
    else if (x == '4')
    {
      motor_setRPM = 4000;
    }
    else if (x == '0')
    {
      motor_setRPM = 0;
    }
  }

  time_last_loop = millis();
  while (millis() - time_last_loop <= 100);
}


void isr_grn_button()
{
  Serial.println("GRN BUTTON++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
     if(millis() - time_grn_pressed > 200) //debounce
        {
  if ( motor_setRPM == 0)
  {
    motor_setRPM = DISPENSE_SPEED;
  }
  else if (  motor_setRPM == DISPENSE_SPEED)
  {
    motor_setRPM = COAT_SPEED;
    time_coating = 0;
  }
  time_grn_pressed = millis();
        }

}

void isr_red_button()
{
  Serial.println("RED BUTTON-----------------------------------------------------------------------------------------");
  if (millis() - time_red_pressed > 200)
  {
    motor_setRPM = 0;
    time_coating = 0;
    time_red_pressed = millis();
  }
}









