#include <Wire.h>

// define PINs
#define pin_stp_q1 2
#define pin_dir_q1 3
#define pin_stp_q2 4
#define pin_dir_q2 5
#define pin_MS1 6
#define pin_MS2 7


// Communication
#define pin_m1_start 8 
#define pin_m1_dir 9
#define pin_m2_start 10 
#define pin_m2_dir 11

#define pin_com 13

// Time
unsigned long startTime; 
unsigned long endTime; 
unsigned long duration;

// Robot
int x = 0;
bool direction_q1;
bool direction_q2;

// Nice to change
bool MS1 = LOW;
bool MS2 = HIGH;
int motordelay= 1;

// Communication
volatile bool motor1_start = false;
volatile bool motor1_dir = false;

// Communication
volatile bool motor2_start = false;
volatile bool motor2_dir = false;

void setup() {
  pinMode(pin_stp_q1, OUTPUT);
  pinMode(pin_dir_q1, OUTPUT);
  pinMode(pin_stp_q2, OUTPUT);
  pinMode(pin_dir_q2, OUTPUT);
  
  pinMode(pin_MS1, OUTPUT);
  pinMode(pin_MS2, OUTPUT);
  digitalWrite(pin_MS1,MS1);
  digitalWrite(pin_MS2,MS2);

  pinMode(pin_m1_start, INPUT); 
  pinMode(pin_m1_dir, INPUT);
  pinMode(pin_m2_start, INPUT); 
  pinMode(pin_m2_dir, INPUT);

  Serial.begin(9600); // Initialisiere die serielle Kommunikation
}


void loop() {

  // Read inputs
  motor1_start = digitalRead(pin_m1_start); 
  motor1_dir = digitalRead(pin_m1_dir);
  motor2_start = digitalRead(pin_m2_start); 
  motor2_dir = digitalRead(pin_m2_dir);

  //Serial.println(digitalRead(pin_MS1)); 
  //Serial.println(digitalRead(pin_MS2));

  //Serial.println(motor1_start);
  //startTime = millis();
  if (motor1_start){
    // check direction
    digitalWrite(pin_dir_q1, motor1_dir ? HIGH : LOW);

    // move the motor
    digitalWrite(pin_stp_q1,HIGH);
    delay(motordelay);
    digitalWrite(pin_stp_q1,LOW);
    delay(motordelay);
  }
  //endTime = millis();
  //Serial.println(endTime-startTime);

  //Serial.println(motor2_start);
  if (motor2_start){
    // check direction
    digitalWrite(pin_dir_q2, motor2_dir ? HIGH : LOW);

    // move the motor
    digitalWrite(pin_stp_q2,HIGH);
    delay(motordelay);
    digitalWrite(pin_stp_q2,LOW);
    delay(motordelay);
  }

}