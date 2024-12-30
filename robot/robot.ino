#include <math.h>


#define stp_q1 2
#define dir_q1 3
#define stp_q2 4
#define dir_q2 5

#define MS1_q1 6
#define MS2_q1 7
#define MS1_q2 8
#define MS2_q2 9


int x;
int steps;
float q1_ist_grad = 90;
float q2_ist_grad = 90;
float X;
float Y;

bool MS1 = HIGH;
bool MS2 = HIGH;
float deg_per_step;

float L_A1 = 145;
float L_A2 = 115;
float L_B1 = 145;
float L_B2 = 115;
float D = 100;

int circle = 1;


bool check_angle_is_valid(int angle,int upper_limit,int lower_limit)
{
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


float* pick_angle(float q1_1,float q1_2,float q2_1,float q2_2,float upper_limit,float lower_limit,float L_A1,float L_B1,float D){

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
    bool q1_valid = check_angle_is_valid(q1_1,upper_limit,lower_limit) or check_angle_is_valid(q1_2,upper_limit,lower_limit);
    bool q2_valid = check_angle_is_valid(q2_1,upper_limit,lower_limit) or check_angle_is_valid(q2_2,upper_limit,lower_limit);
    
    bool q1_1_valid = check_angle_is_valid(q1_1,upper_limit,lower_limit);
    bool q1_2_valid = check_angle_is_valid(q1_2,upper_limit,lower_limit);
    bool q2_1_valid = check_angle_is_valid(q2_1,upper_limit,lower_limit);
    bool q2_2_valid = check_angle_is_valid(q2_2,upper_limit,lower_limit);
    
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


float* ikp(float X, float Y, float L_A1, float L_A2, float L_B1, float L_B2, float D)
{
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

void setup() {
  delay(2000);

  pinMode(stp_q1, OUTPUT);
  pinMode(dir_q1, OUTPUT);
  pinMode(stp_q2, OUTPUT);
  pinMode(dir_q2, OUTPUT);
  
  pinMode(MS1_q1, OUTPUT);
  pinMode(MS2_q1, OUTPUT);
  pinMode(MS1_q2, OUTPUT);
  pinMode(MS2_q2, OUTPUT);
  
  digitalWrite(MS1_q1,MS1);
  digitalWrite(MS2_q1,MS2);
  digitalWrite(MS1_q2,MS1);
  digitalWrite(MS2_q2,MS2);
  Serial.begin(9600);
  delay(10000);
}



void loop() {

  if (circle == 1){
    X = -50;
    Y = 180;
  } 
  else if (circle == 2){
    X = 100;
    Y = 200;
  }
  else if (circle == 3){
    X = 0;
    Y = 200;
  }
  else if (circle == 4){
    X = 150;
    Y = 180;
  }
  
  Serial.println(circle);

  float* q = ikp(X,Y,L_A1, L_A2, L_B1, L_B2, D);

  float q1_1 = q[0];
  float q1_2 = q[1];
  float q2_1 = q[2];
  float q2_2 = q[3];

  float upper_limit = 160;
  float lower_limit = 20;

  float* q_soll = pick_angle(q1_1,q1_2,q2_1,q2_2,upper_limit,lower_limit,L_A1,L_B1,D);
  float q1_soll_grad = q_soll[0];
  float q2_soll_grad = q_soll[1];

  bool direction_q1;
  bool direction_q2;
  
  /* Check MS1 and MS2 */
  if (MS1 == HIGH && MS2 == HIGH){
    deg_per_step = 1.8;
  } else if (MS1 == HIGH && MS2 == LOW){
    deg_per_step = 0.9;
  } else if (MS1 == LOW && MS2 == HIGH){
    deg_per_step = 0.45;
  } else if (MS1 == LOW && MS2 == LOW){
    deg_per_step = 0.225;
  }

  /* Achse q1 */
  float q1_diff_grad = q1_soll_grad - q1_ist_grad;
  
  /* dir = HIGH, rechts herum, oder minus */
  if (q1_diff_grad < 0) {
    digitalWrite(dir_q1,HIGH);
    direction_q1 = true;
  } else {
    digitalWrite(dir_q1,LOW);
    direction_q1 = false;
  }
  double q1_diff_step = abs(q1_diff_grad/deg_per_step);
  int q1_steps = round(q1_diff_step);


  /* Achse q2 */
  float q2_diff_grad = q2_soll_grad - q2_ist_grad;
  
  /* dir = HIGH, rechts herum, oder minus */
  if (q2_diff_grad < 0) {
    digitalWrite(dir_q2,HIGH);
    direction_q2 = true;
  } else {
    digitalWrite(dir_q2,LOW);
    direction_q2 = false;
  }
  
  double q2_diff_step = abs(q2_diff_grad/deg_per_step);
  int q2_steps = round(q2_diff_step);

  int steps = 0;
  if (abs(q1_steps) > abs(q2_steps)){
    steps = abs(q1_steps);
  } else {
    steps = abs(q2_steps);
  }
  
  Serial.println(q1_ist_grad);
  Serial.println(q1_diff_grad);
  Serial.println(q1_soll_grad);
  Serial.println(q2_ist_grad);
  Serial.println(q2_diff_grad);
  Serial.println(q2_soll_grad);

  int motordelay= 3;

  delay(30);
  for(x= 0; x<steps; x++){
    delay(motordelay);
    if (x<abs(q1_steps)){
      digitalWrite(stp_q1,HIGH); 
      delay(motordelay);
      digitalWrite(stp_q1,LOW); 
      delay(motordelay);
    }
    delay(motordelay);
    if (x<abs(q2_steps)){
      digitalWrite(stp_q2,HIGH); 
      delay(motordelay);
      digitalWrite(stp_q2,LOW); 
      delay(motordelay);
    }
  }

  q1_ist_grad = q1_soll_grad;
  q2_ist_grad = q2_soll_grad;
  circle = circle + 1;
  
  if (circle>4){
    circle = 0;
  }
  delete[] q;
  delete[] q_soll;
}
