#include <Servo.h>
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

void MOVEL(){

}

// MOVEL <leg#> <px> <py> <pz> [speed (rad/s)]
void MOVEP(){

}

// POS <leg#> <joint#> <angle> [speed (rad/s)]
void POS(int leg, int joint, float angle, float speed = 1){
  float current_angle = motor_angle[leg*3+joint];
  while(current_angle != angle){
  
  }
  motor_angle[i] = new_angle[i];
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

  
