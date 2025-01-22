#include <Wire.h>

#define pin_encoder1_A 2
#define pin_encoder1_B 3

#define pin_motor1_start 4 
#define pin_motor1_dir 5

#define pin_encoder2_A 6
#define pin_encoder2_B 7 

#define pin_motor2_start 8 
#define pin_motor2_dir 9

#define pin_com 13

// Time
unsigned long startTime; 
unsigned long endTime; 
unsigned long duration;

// Status Motor 1 
bool motor1_start = false;
bool motor1_dir = true;
float motor1_current_angle = 90;
float motor1_target_angle;

// Status Motor 2 
bool motor2_start = false;
bool motor2_dir = true;
float motor2_current_angle = 90;
float motor2_target_angle;


// Encoder  1
float encoder1_Pos = 450; // Equal to 90 degress
int encoder1_Pin_A_Last = LOW;
int encoder1_Pin_A_Now = LOW;
int encoder1_Pin_B_Now = LOW;

// Encoder  2
float encoder2_Pos = 450; // Equal to 90 degress
int encoder2_Pin_A_Last = LOW;
int encoder2_Pin_A_Now = LOW;
int encoder2_Pin_B_Now = LOW;

// Robot Data
float L_A1 = 134;
float L_A2 = 96;
float L_B1 = 134;
float L_B2 = 94;
float D = 80;

// Working stuff
int x;
int steps;
float q1_ist_grad = 90;
float q2_ist_grad = 90;
float X;
float Y;
int circle = 1;

bool check_angle_is_valid(int angle,int upper_limit,int lower_limit){
    bool valid = false;
    if (angle > lower_limit and angle < upper_limit){
        valid = true;
    }
    return valid;
}

float* findMax(float array[], int size) {
  float* maxVal = new float[2];
  maxVal[0] = array[0];
  maxVal[1] = 0;

  for (int i = 1; i < size; i++) {
    if (array[i] > maxVal[0]) {
      maxVal[0] = array[i];
      maxVal[1] = i;
    }
  }
  return maxVal;
}

float* pick_angle(float q1_1,float q1_2,float q2_1,float q2_2,float upper_limit_q1,float lower_limit_q1, float upper_limit_q2, float lower_limit_q2,float L_A1,float L_B1,float D){

    /* init*/
    float* q_soll = new float[2];
    float diff[4];
    String error_Msg_PosNotReachable = "At least one joint is out of range";
    String error_Msg_JointsTooClose = "Joints are to close to each other, risk of mechanical damage";
    int condition_case = 0;


    /* the prefered angles are the ones with the biggest distance between the two free joints*/
    /* Calculate difference between the two free joints*/
    float X_q1_1 = L_A1 * cos(radians(q1_1));
    float X_q1_2 = L_A1 * cos(radians(q1_2));
    float X_q2_1 = L_B1 * cos(radians(q2_1)) + D;
    float X_q2_2 = L_B1 * cos(radians(q2_2)) + D;
    
    diff[0] = (X_q2_1 - X_q1_1);
    diff[1] = (X_q2_1 - X_q1_2);
    diff[2] = (X_q2_2 - X_q1_1);
    diff[3] = (X_q2_2 - X_q1_2);
    
    float* max_diff = findMax(diff,4);

    /* check which angles are within the allowed range*/
    bool q1_valid = check_angle_is_valid(q1_1,upper_limit_q1,lower_limit_q1) or check_angle_is_valid(q1_2,upper_limit_q1,lower_limit_q1);
    bool q2_valid = check_angle_is_valid(q2_1,upper_limit_q2,lower_limit_q2) or check_angle_is_valid(q2_2,upper_limit_q2,lower_limit_q2);
    
    bool q1_1_valid = check_angle_is_valid(q1_1,upper_limit_q1,lower_limit_q1);
    bool q1_2_valid = check_angle_is_valid(q1_2,upper_limit_q1,lower_limit_q1);
    bool q2_1_valid = check_angle_is_valid(q2_1,upper_limit_q2,lower_limit_q2);
    bool q2_2_valid = check_angle_is_valid(q2_2,upper_limit_q2,lower_limit_q2);
    
    /* choose the angles, first check if valid, then check which distance is the biggest*/
    if (q1_valid and q2_valid){
        
        if (q1_1_valid and not q1_2_valid and q2_1_valid and not q2_2_valid){ 
            condition_case = 1;
            if (diff[0] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_1;
            }
        }
        else if(q1_1_valid and not q1_2_valid and not q2_1_valid and q2_2_valid){
            condition_case = 2;
            if (diff[2] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_2;
            }
        }
        else if(not q1_1_valid and q1_2_valid and q2_1_valid and not q2_2_valid){
            condition_case = 3;
            if (diff[1] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_1;
            }
        }
        else if(not q1_1_valid and q1_2_valid and not q2_1_valid and q2_2_valid){
            condition_case = 4;
            if (diff[3] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_2;
            }
        }
        else if (q1_1_valid and q1_2_valid and not q2_1_valid and q2_2_valid){
            condition_case = 5;
            if (diff[3] > diff[2] and diff[3] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_2;
            }
            else if (diff[2] >= diff[3] and diff[2] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_2;
            }
        }  
        else if (q1_1_valid and q1_2_valid and q2_1_valid and not q2_2_valid){
            condition_case = 6;
            if (diff[1] > diff[0] and diff[1] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_1;
            }
            else if (diff[0] >= diff[1] and diff[0] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_1;
            }
        }
        else if (q1_1_valid and not q1_2_valid and q2_1_valid and q2_2_valid){
            condition_case = 7;
            if (diff[2] > diff[0] and diff[2] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_2;
            }
            else if (diff[0] >= diff[2] and diff[0] > 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_1;
            } 
        }      
        else if (not q1_1_valid and q1_2_valid and q2_1_valid and q2_2_valid){
            condition_case = 8;
            if (diff[3] > diff[1] and diff[3] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_2;
            }
            else if (diff[1] >= diff[3] and diff[1] > 0){
                q_soll[0] = q1_2;
                q_soll[1] = q2_1;
            }
        }        
        else if (q1_1_valid and q1_2_valid and q2_1_valid and q2_2_valid){
            condition_case = 9;
            if (max_diff[1] == 0){
                q_soll[0] = q1_1;
                q_soll[1] = q2_1;
            }
            else if (max_diff[1] == 1){
                q_soll[0] = q1_2;
                q_soll[1] = q2_1;
            }
            else if (max_diff[1] == 2){
                q_soll[0] = q1_1;
                q_soll[1] = q2_2;
            }
            else if (max_diff[1] == 3){
                q_soll[0] = q1_2;
                q_soll[1] = q2_2;
            }
        }
    }

    return q_soll;
}

float* ikp(float X, float Y, float L_A1, float L_A2, float L_B1, float L_B2, float D){
  float* q = new float[4];
  
  /* Achse q1 */
  float a1 = 2 * L_A1 * Y;
  float b1 = -2 * L_A1 * X;
  float c1 = X*X + Y*Y + L_A1*L_A1 - L_A2*L_A2;
  float sum_1 = a1*a1 + b1*b1 - c1*c1;
  float d1_plus = sqrt(sum_1);
  float d1_minus = -sqrt(sum_1);
        
  float q1_1 = degrees(atan2(c1, d1_plus) + atan2(b1,a1));
  float q1_2 = degrees(atan2(c1, d1_minus) + atan2(b1,a1));

  /* Achse q2 */
  float a2 = 2 * L_B1 * Y;
  float b2 = 2 * L_B1 * (D - X);
  float c2 = X*X + Y*Y + D*D + L_B1*L_B1 - L_B2*L_B2 - 2*D*X;
  float sum_2 = a2*a2 + b2*b2 - c2*c2;
  float d2_plus = sqrt(sum_2);
  float d2_minus = -sqrt(sum_2);
      
  float q2_1 = degrees(atan2(c2, d2_plus) + atan2(b2,a2));
  float q2_2 = degrees(atan2(c2, d2_minus) + atan2(b2,a2));

  q[0] = q1_1;
  q[1] = q1_2;
  q[2] = q2_1;
  q[3] = q2_2;

  return q;
}

float move_motor_to_angle(){

  // Write info to motors
  digitalWrite(pin_motor1_start, motor1_start); 
  digitalWrite(pin_motor1_dir, motor1_dir);

  digitalWrite(pin_motor2_start, motor2_start); 
  digitalWrite(pin_motor2_dir, motor2_dir);

  while (motor1_start || motor2_start){
    //startTime = millis();
    // read encoders
    encoder1_Pin_A_Now = digitalRead(pin_encoder1_A);
    encoder1_Pin_B_Now = digitalRead(pin_encoder1_B);

    encoder2_Pin_A_Now = digitalRead(pin_encoder2_A);
    encoder2_Pin_B_Now = digitalRead(pin_encoder2_B);
    //Serial.println(encoder2_Pin_B_Now);


    // Update Encoder 1
    if (motor1_start){
      if ((encoder1_Pin_A_Last == HIGH) && (encoder1_Pin_A_Now == LOW)) {
        if (encoder1_Pin_B_Now == HIGH) {
          encoder1_Pos++;
        } else {
          encoder1_Pos--;
        }
      }
      encoder1_Pin_A_Last = encoder1_Pin_A_Now;

      // Check if target angle was reached
      motor1_current_angle = encoder1_Pos*0.2;
      //Serial.println(motor1_current_angle);
      if (motor1_dir){
        if (motor1_current_angle > motor1_target_angle){
          motor1_start = false; // stop the motor as soon as its reached the target angle
          digitalWrite(pin_motor1_start, motor1_start);
          Serial.println("Stop M1");

        }
      } else {
        if (motor1_current_angle < motor1_target_angle){
          motor1_start = false; // stop the motor as soon as its reached the target angle
          digitalWrite(pin_motor1_start, motor1_start);
          Serial.println("Stop M1");
        }
      }
    }


    // Update Encoder 2
    if (motor2_start){
      if ((encoder2_Pin_A_Last == HIGH) && (encoder2_Pin_A_Now == LOW)) {
        if (encoder2_Pin_B_Now == HIGH) {
          encoder2_Pos++;
          //Serial.println("pups");
        } else {
          encoder2_Pos--;
        }
      }
      encoder2_Pin_A_Last = encoder2_Pin_A_Now;

      // Check if target angle was reached
      motor2_current_angle = encoder2_Pos*0.2;
      //Serial.println(motor2_current_angle);
      if (motor2_dir){
        if (motor2_current_angle > motor2_target_angle){
          motor2_start = false; // stop the motor as soon as its reached the target angle
          digitalWrite(pin_motor2_start, motor2_start);
          Serial.println("Stop M2");
        }
      } else {
        if (motor2_current_angle < motor2_target_angle){
          motor2_start = false; // stop the motor as soon as its reached the target angle
          digitalWrite(pin_motor2_start, motor2_start);
          Serial.println("Stop M2");
        }
      }
    }
    //endTime = millis();
    //Serial.println(endTime-startTime);
  }
}


void setup() {
  // Encoder  1
  pinMode (pin_encoder1_A, INPUT_PULLUP);
  pinMode (pin_encoder1_B, INPUT_PULLUP);

  // Encoder  2
  pinMode (pin_encoder2_A, INPUT_PULLUP);
  pinMode (pin_encoder2_B, INPUT_PULLUP);

  // Communication - Motor 1
  pinMode(pin_motor1_start, OUTPUT); 
  pinMode(pin_motor1_dir, OUTPUT);

  // Communication - Motor 2
  pinMode(pin_motor2_start, OUTPUT); 
  pinMode(pin_motor2_dir, OUTPUT);

  pinMode(pin_com, OUTPUT);
  digitalWrite(pin_motor1_dir,LOW);

  Serial.begin(9600); // Initialisiere die serielle Kommunikation
  delay(5000);
}


void loop() {
  

  // Choose Target Position
  if (circle == 1){
    X = -100;
    Y = 120;
  } 
  else if (circle == 2){
    X = 80;
    Y = 200;
  }
  else if (circle == 3){
    X = 0;
    Y = 150;
  }
  else if (circle == 4){
    X = 180;
    Y = 120;
  }
  

  
  // Solve IKP
  float* q = ikp(X,Y,L_A1, L_A2, L_B1, L_B2, D);

  float q1_1 = q[0];
  float q1_2 = q[1];
  float q2_1 = q[2];
  float q2_2 = q[3];

  Serial.println(q1_1);
  Serial.println(q1_2);
  Serial.println(q2_1);
  Serial.println(q2_2);

  float upper_limit_q1 = 180;
  float lower_limit_q1 = 45;
  float upper_limit_q2 = 135;
  float lower_limit_q2 = 0;

  float* q_soll = pick_angle(q1_1,q1_2,q2_1,q2_2,upper_limit_q1,lower_limit_q1,upper_limit_q2,lower_limit_q2,L_A1,L_B1,D);
  motor1_target_angle = q_soll[0];
  motor2_target_angle = q_soll[1]; 
  


  // Get the direction
  float motor1_angle_diff = motor1_target_angle - motor1_current_angle;

  if (motor1_angle_diff > 0) {
    digitalWrite(pin_motor1_dir,HIGH);
    motor1_dir = true;
  } else {
    digitalWrite(pin_motor1_dir,LOW);
    motor1_dir = false;
  }

  float motor2_angle_diff = motor2_target_angle - motor2_current_angle;
  if (motor2_angle_diff > 0) {
    digitalWrite(pin_motor2_dir,HIGH);
    motor2_dir = true;
  } else {
    digitalWrite(pin_motor2_dir,LOW);
    motor2_dir = false;
  }
  


  // Move the motors
  motor1_start = true;
  motor2_start = true;
  Serial.println(motor1_start);
  Serial.println(motor2_start);
  Serial.println(motor1_target_angle);
  Serial.println(motor2_target_angle);
  move_motor_to_angle();


  // Repeat
  circle = circle + 1;
  if (circle>4){
    circle = 0;
  }
  delete[] q;
  delete[] q_soll;

  delay(500);
}

