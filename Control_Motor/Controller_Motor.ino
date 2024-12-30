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



// Robot
int x = 0;
bool direction_q1;
bool direction_q2;

// Nice to change
bool MS1 = LOW;
bool MS2 = LOW;
int motordelay= 3;

// Communication
volatile bool motor_start = false;
volatile bool motor_dir = false;

void setup() {
  pinMode(pin_stp_q1, OUTPUT);
  pinMode(pin_dir_q1, OUTPUT);
  
  pinMode(pin_MS1, OUTPUT);
  pinMode(pin_MS2, OUTPUT);
  digitalWrite(pin_MS1,MS1);
  digitalWrite(pin_MS2,MS2);

  pinMode(pin_m1_start, INPUT); 
  pinMode(pin_m1_dir, INPUT);

  Serial.begin(9600); // Initialisiere die serielle Kommunikation
}


void loop() {

  // Read inputs
  motor_start = digitalRead(pin_m1_start); 
  motor_dir = digitalRead(pin_m1_dir);
  Serial.println(motor_start);
  if (motor_start){
    
    //Serial.println(motor_dir);
    //Serial.println(pin_stp_q1);

    // check direction
    digitalWrite(pin_dir_q1, motor_dir ? HIGH : LOW);

    // move the motor
    digitalWrite(pin_stp_q1,HIGH);
    delay(3);
    digitalWrite(pin_stp_q1,LOW);
    delay(3);
  }
}