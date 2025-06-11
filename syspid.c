/* ----------------------------
   MAX6675 Module Pin Mapping
   ----------------------------
   MAX6675        ==>   Arduino
   CS             ==>   D10
   SO             ==>   D12
   SCK            ==>   D13
   Vcc            ==>   5V
   GND            ==>   GND

   I2C LCD Module Pin Mapping
   ----------------------------
   LCD            ==>   Arduino
   SCL            ==>   A5
   SDA            ==>   A4
   Vcc            ==>   5V
   GND            ==>   GND
*/

#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // Change to 0x3F if needed

// Pins
#define PWM_pin 3
#define MAX6675_CS 10
#define MAX6675_SO 12
#define MAX6675_SCK 13

int clk = 8;  // Rotary encoder CLK
int data = 9; // Rotary encoder DT
int button = 11; // Rotary encoder push button

// Variables
float set_temperature = 0;
float temperature_read = 0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated = 0;
float last_set_temperature = 0;

// Encoder state
int clk_State;
int Last_State;
bool dt_State;

// PID constants
int kp = 9.1, ki = 0.6, kd = 1.8;
int PID_p = 0, PID_i = 0, PID_d = 0;
float last_kp = 0, last_ki = 0, last_kd = 0;
int PID_values_fixed = 0;

void setup() {
  pinMode(PWM_pin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | 0x03; // PWM frequency ~928.5 Hz

  Time = millis();
  Last_State = (PINB & B00000001);

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT3);

  pinMode(button, INPUT);
  pinMode(data, INPUT);
  pinMode(clk, INPUT);

  lcd.init();
  lcd.backlight();

  Serial.begin(9600);
  Serial.println("PID Temperature Controller Initialized");
}

void loop() {
  if (menu_activated == 0) {
    temperature_read = readThermocouple();
    PID_error = set_temperature - temperature_read + 3;
    PID_p = 0.01 * kp * PID_error;
    PID_i = 0.01 * PID_i + (ki * PID_error);

    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000;

    PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
    PID_value = PID_p + PID_i + PID_d;

    PID_value = constrain(PID_value, 0, 255);
    analogWrite(PWM_pin, 255 - PID_value);
    previous_error = PID_error;

    delay(250);
    lcd.setCursor(0, 0);
    lcd.print("PID TEMP control");
    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.setCursor(2, 1);
    lcd.print(set_temperature, 1);
    lcd.setCursor(9, 1);
    lcd.print("R:");
    lcd.setCursor(11, 1);
    lcd.print(temperature_read, 1);

    Serial.print("Set Temp: "); Serial.print(set_temperature);
    Serial.print(" | Real Temp: "); Serial.println(temperature_read);
  }

  if (menu_activated == 1 && set_temperature != last_set_temperature) {
    analogWrite(PWM_pin, 255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set  temperature");
    lcd.setCursor(0, 1);
    lcd.print(set_temperature);
    last_set_temperature = set_temperature;
  }

  if (menu_activated == 2 && kp != last_kp) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set   P  value  ");
    lcd.setCursor(0, 1);
    lcd.print(kp);
    last_kp = kp;
  }

  if (menu_activated == 3 && ki != last_ki) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set   I  value  ");
    lcd.setCursor(0, 1);
    lcd.print(ki);
    last_ki = ki;
  }

  if (menu_activated == 4 && kd != last_kd) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set   D  value  ");
    lcd.setCursor(0, 1);
    lcd.print(kd);
    last_kd = kd;
  }
}

double readThermocouple() {
  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);

  digitalWrite(MAX6675_CS, LOW);
  delayMicroseconds(10);

  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  digitalWrite(MAX6675_CS, HIGH);

  if (v & 0x4) return NAN;
  v >>= 3;
  return v * 0.25;
}

ISR(PCINT0_vect) {
  if (menu_activated >= 1 && menu_activated <= 4) {
    clk_State = (PINB & B00000001);
    dt_State  = (PINB & B00000010);

    if (clk_State != Last_State) {
      int increment = (dt_State != clk_State) ? 1 : -1;
      if (menu_activated == 1) set_temperature += 0.5 * increment;
      if (menu_activated == 2) kp += increment;
      if (menu_activated == 3) ki += increment;
      if (menu_activated == 4) kd += increment;
    }
    Last_State = clk_State;
  }

  if (PINB & B00001000) button_pressed = 1;
  else if (button_pressed == 1) {
    if (menu_activated == 4) {
      menu_activated = 0;
      PID_values_fixed = 1;
    } else {
      menu_activated++;
      PID_values_fixed = 0;
      if (menu_activated == 1) set_temperature++;
      if (menu_activated == 2) kp++;
      if (menu_activated == 3) ki++;
      if (menu_activated == 4) kd++;
    }
    button_pressed = 0;
    delay(1000);
  }
}