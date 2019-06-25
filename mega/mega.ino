#include <Servo.h>
#include <math.h>
#define DEBUG

#define m_0_1_pin 2
#define m_0_2_pin 3
#define m_0_3_pin 4
#define m_1_1_pin 5
#define m_1_2_pin 6 
#define m_1_3_pin 7
#define m_2_1_pin 8
#define m_2_2_pin 9
#define m_2_3_pin 10
#define m_3_1_pin 11
#define m_3_2_pin 12
#define m_3_3_pin 13

#define _19_relay_pin 48
#define _12_relay_pin 52
#define _8_relay_pin 46
#define _5_relay_pin 50

// MOVEL <leg#> <px> <py> <pz> [speed (mm/s)]


Servo motor[12];
//int motor_pos[12] = { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0};
//
//int max_angle[12] = {80, 80, 85, 80, 80, 85, 80, 80, 85, 80, 80, 85};
////                  0_1  0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
//int min_angle[12] = {-80, -80, -85, -80, -80, -85, -80, -80, -85, -80, -80, -85};
////                   0_1  0_2  0_3  1_1  1_2  1_3  2_1  2_2  2_3  3_1  3_2  3_3
//int motor_dir[12] = {  1,  -1,  1,  1,  -1,  1,  1,  -1,  1,  1,  -1,  1 };
////                   0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
//int motor_offset[12] = {  6,  0,  -2,  -3,  5,  6,  2,  0,  -3,  0,  5,  -5 };
////                      0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3

float motor_angle[12] = { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0};

int motor_pos[12] = { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0};

int max_angle[12] = {86, 55, 83, 77, 45, 91, 82, 55, 82, 80, 55, 80};
//                  0_1  0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
int min_angle[12] = {-74, -55, -90, -83, -55, -79, -78, -50, -88, -80, -50, -90};
//                   0_1  0_2  0_3  1_1  1_2  1_3  2_1  2_2  2_3  3_1  3_2  3_3
int motor_dir[12] = {  1,  -1,  1,  1,  -1,  1,  1,  -1,  1,  1,  -1,  1 };
//                   0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
int motor_offset[12] = {  6,  0,  -2,  -3,  -5,  6,  2,  0,  -3,  0,  5,  -5 };
//                      0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3

void MOVEL();
void MOVEP();
void POS(int leg, int joint, float angle, float speed = 1);
void HOME();
void parseCmd();
void writeServo();
void xyz2degree(int legNum, float x,float y,float z, float& theta1, float& theta2, float& theta3);
void degree2xyz(int legNum, float theta1,float theta2,float theta3, float& x, float& y, float& z);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  motor[0].attach(m_0_1_pin);
  motor[1].attach(m_0_2_pin);
  motor[2].attach(m_0_3_pin);
  motor[3].attach(m_1_1_pin);
  motor[4].attach(m_1_2_pin);
  motor[5].attach(m_1_3_pin);
  motor[6].attach(m_2_1_pin);
  motor[7].attach(m_2_2_pin);
  motor[8].attach(m_2_3_pin);
  motor[9].attach(m_3_1_pin);
  motor[10].attach(m_3_2_pin);
  motor[11].attach(m_3_3_pin);
//  for(int i=0 ; i<12 ; ++i){
//    motor[i].write( (motor_pos[i] )*motor_dir[i] +90);
//  }
  HOME();
  writeServo();
  pinMode(_19_relay_pin, OUTPUT);
  pinMode(_12_relay_pin, OUTPUT);
  pinMode(_8_relay_pin, OUTPUT);
  pinMode(_5_relay_pin, OUTPUT);
  digitalWrite(_19_relay_pin, 0);
  digitalWrite(_12_relay_pin, 0);
  digitalWrite(_8_relay_pin, 0);
  digitalWrite(_5_relay_pin, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  while( Serial.available() ){
     Serial.println( "----------" );
     parseCmd();
     writeServo();
    }
    
  }

  
void MOVEL(){

}

// MOVEP <leg#> <px> <py> <pz>
void MOVEP(int legNum, float x,float y,float z){
  float theta1 = 0;
  float theta2 = 0;
  float theta3 = 0;
  float check_x = 0;
  float check_y = 0;
  float check_z = 0;
  xyz2degree(legNum, x, y, z, theta1, theta2, theta3);
  degree2xyz(legNum, theta1, theta2, theta3, check_x, check_y, check_z);

  if( abs(x-check_x) + abs(y-check_y) + abs(z-check_z) < 1){
    //valid pos
    motor_angle[legNum*3 + 0] = theta1;
    motor_angle[legNum*3 + 1] = theta2;
    motor_angle[legNum*3 + 2] = theta3;
  }
}

// POS <leg#> <joint#> <angle> [speed (rad/s)]
void POS(int leg, int joint, float angle, float speed = 1){
  float current_angle = motor_angle[leg*3+joint];
//  while(current_angle != angle){
//  
//  }
//  motor_angle[i] = new_angle[i];
}

// home pos
void HOME(){
  float new_angle[12] = {0, 90, -90, 0, 90, -90, 0, 90 , -90, 0, 90, -90};
  for(int i=0 ; i<12 ; ++i){
    motor_angle[i] = new_angle[i];
  } 
}

//cmds:
// ML
// MP
// P
// F
// B
// L
// R
void parseCmd(){
  char ch = Serial.read();
  Serial.print(ch);
  if(ch == 'M'){
    ch = Serial.read();
    if( ch == 'P'){
      if(Serial.available()){
        float legNum = Serial.parseFloat();
        float x = Serial.parseFloat();
        float y = Serial.parseFloat();
        float z = Serial.parseFloat();
        MOVEP(legNum, x, y, z);
      }
      
    }else if( ch == 'L'){

    }
  }else if (ch == 'P'){
    
  }else if (ch == 'F'){

  }else if (ch == 'B'){

  }else if (ch == 'L'){

  }else if (ch == 'R'){

  }
  else if (ch == 'R'){

  }
  else if (ch == 'H'){
    HOME();
  }
}


void writeServo(){
  for(int i=0 ; i<12 ; ++i){
    motor_pos[i] = int((motor_angle[i]+90)/180*float(max_angle[i]-min_angle[i])+min_angle[i]);
    if(motor_pos[i] > max_angle[i]){
      motor_pos[i] = max_angle[i];
    }else if(motor_pos[i] < min_angle[i]){
      motor_pos[i] = min_angle[i];
    }
    motor[i].write( (motor_pos[i]  )*motor_dir[i] +90 );
  }
}

void xyz2degree(int legNum, float x,float y,float z, float& theta1, float& theta2, float& theta3)
{
  float robotWidth = 150;
  float robotHeight = 49;
  float robotL1 = 50;
  float robotL2 = 149;
  float robotL3 = 198;

  float th1 = 0;
  float th2 = 0;
  float th3 = 0;
  float centerPointX = 0;
  float centerPointY = 0;  
  
  switch(legNum){
    case 0:
      centerPointX = robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 1:
      centerPointX = -1*robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 2:
      centerPointX = -1*robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    case 3:
      centerPointX = robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    default:
      break;  
  }

  float r = sqrt((x-centerPointX)*(x-centerPointX)+(y-centerPointY)*(y-centerPointY))- robotL1;
  float thr = atan((y-centerPointY)/(x-centerPointX));
  float _z = z - robotHeight;
  float L = sqrt(r*r + _z*_z);
  float thL = atan(_z/r)/PI*180;

  th3 = acos((robotL2*robotL2+robotL3*robotL3-L*L)/(2*robotL2*robotL3))/PI*180;
  th2 = acos((robotL2*robotL2+L*L-robotL3*robotL3)/(2*robotL2*L))/PI*180 + thL;
  th1 = thr - legNum*90;

  theta1 = th1;
  theta2 = th2;
  theta3 = th3;
}

void degree2xyz(int legNum, float theta1,float theta2,float theta3, float& x, float& y, float& z)
{
  float robotWidth = 150;
  float robotHeight = 49;
  float robotL1 = 50;
  float robotL2 = 149;
  float robotL3 = 198;
  
  float th1 = theta1;
  float th2 = theta2;
  float th3 = theta3+90;
  float centerPointX = 0;
  float centerPointY = 0;

  float r = robotL2*cos(th2/180*PI) + robotL3*sin((th2+th3-90)/180*PI) + robotL1;
  float _z = robotL2*sin(th2/180*PI) - robotL3*cos((th2+th3-90)/180*PI);

  switch(legNum){
    case 0:
      th1 = theta1;
      centerPointX = robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 1:
      th1 = theta1+90;
      centerPointX = -1*robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 2:
      th1 = theta1+180;
      centerPointX = -1*robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    case 3:
      th1 = theta1+270;
      centerPointX = robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    default:
      break;  
  }

  x = centerPointX + r * cos(th1/180*PI);
  y = centerPointY + r * sin(th1/180*PI);
  z = _z + robotHeight;
}
