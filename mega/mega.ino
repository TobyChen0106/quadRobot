#include <Servo.h>
#include <math.h>
#include <PS2X_lib.h>  //for v1.6
#define DEBUG

//controller
PS2X ps2x; // create PS2 Controller Class
int error = 0; 
byte type = 0;
byte vibrate = 0;

//#define m_0_1_pin 2
//#define m_0_2_pin 3
//#define m_0_3_pin 4
//#define m_1_1_pin 5
//#define m_1_2_pin 6 
//#define m_1_3_pin 7
//#define m_2_1_pin 8
//#define m_2_2_pin 9
//#define m_2_3_pin 10
//#define m_3_1_pin 11
//#define m_3_2_pin 12
//#define m_3_3_pin 13

#define m_1_1_pin 2
#define m_1_2_pin 3
#define m_1_3_pin 4
#define m_2_1_pin 5
#define m_2_2_pin 6 
#define m_2_3_pin 7
#define m_3_1_pin 8
#define m_3_2_pin 9
#define m_3_3_pin 10
#define m_0_1_pin 11
#define m_0_2_pin 12
#define m_0_3_pin 13

#define _19_relay_pin 40
#define _12_relay_pin 44
#define _8_relay_pin 38
#define _5_relay_pin 42

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

int max_angle[12] = {80, 55, 80, 86, 55, 83, 77, 45, 91, 82, 55, 82};
//                  0_1  0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
int min_angle[12] = {-80, -50, -90, -74, -55, -90, -83, -55, -79, -78, -50, -88};
//                   0_1  0_2  0_3  1_1  1_2  1_3  2_1  2_2  2_3  3_1  3_2  3_3
int motor_dir[12] = {  1,  -1,  1,  1,  -1,  1,  1,  -1,  1,  1,  -1,  1 };
//                   0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3
int motor_offset[12] = {0,  5,  -5 ,   6,  0,  -2,  -3,  -5,  6,  2,  0,  -3};
//                      0_1 0_2 0_3 1_1 1_2 1_3 2_1 2_2 2_3 3_1 3_2 3_3

float line[4][7] = {0};
unsigned long long updateLinearStart_time[4] = {0};
int updateLinearFlag[4] = {0};

void MOVEL();
void MOVEP();
void POS(int leg, int joint, float angle, float speed = 1);
void HOME();
void parseCmd();
void writeServo();
void xyz2degree(int legNum, float x,float y,float z, float& theta1, float& theta2, float& theta3);
void degree2xyz(int legNum, float theta1,float theta2,float theta3, float& x, float& y, float& z);
void forward(int steps=1);
void backward(int steps=1);
void leftward(int steps=1);
void rightward(int steps=1);
void stand(int height=50);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  error = ps2x.config_gamepad(52,51,53,50, true, true);
  if(error == 0){
    Serial.println("Controller OK!");
  }
  else if(error == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

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
  stand();
}

void loop() {
  // put your main code here, to run repeatedly:
  while( Serial.available() ){
    Serial.println( "----------" );
    parseCmd();
    writeServo();
  }
  ps2x.read_gamepad(false, vibrate);
  // Serial.println("-----");
  // Serial.println(ps2x.Analog(PSS_LY));
  // Serial.println(ps2x.Analog(PSS_LX)); 
  // Serial.println(ps2x.Analog(PSS_RY)); 
  // Serial.println(ps2x.Analog(PSS_RX)); 

  
  if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)){
    int height = (255-ps2x.Analog(PSS_RY))/255*80;
    stand(height);
    Serial.println(height);
  }else{
    int pos_x = ps2x.Analog(PSS_LX)-128;
    int pos_y = ps2x.Analog(PSS_LY)-127;
    
    if( abs(pos_x) > abs(pos_y)){
      if(pos_x >100){
        rightward(1);
      }else if (pos_x < -100){
        leftward(1);
      }
    }else{
      if(pos_y >100){
        backward(1);
      }else if (pos_y < -100){
        forward(1);
      }
    }
  }
  
  delay(50);
}
  
void forward(int steps){
  // for(int i ; i<steps ; ++i){
    
  // }
}
void backward(int steps){};
void leftward(int steps){};
void rightward(int steps){};
void stand(int height){
  MOVEP(0, 200,200,-1*height);
  MOVEP(1, -200,200,-1*height);
  MOVEP(2, -200,-200,-1*height);
  MOVEP(3, 200,-200,-1*height);
  writeServo();
}



// MOVEP <leg#> <px> <py> <pz>
void MOVEL(int legNum, float x,float y,float z, float speed){
  float current_x = 0;
  float current_y = 0;
  float current_z = 0;

  degree2xyz(legNum, motor_angle[legNum*3+0], motor_angle[legNum*3+1], motor_angle[legNum*3+2], current_x, current_y, current_z);

  line[legNum][0] = current_x;
  line[legNum][1] = current_y;
  line[legNum][2] = current_z;
  line[legNum][3] = x;
  line[legNum][4] = y;
  line[legNum][5] = z;
  line[legNum][6] = speed;
  // [current_x, current_y, current_z, x, y, z, speed];
  updateLinearStart_time[legNum] = millis();
  updateLinearFlag[legNum] = 1;
}

void updateLinear(){
  while(updateLinearFlag[0] != 0 || updateLinearFlag[1] != 0  || updateLinearFlag[2] != 0  || updateLinearFlag[3] != 0 ){
    for (int i=0; i < 4 ; ++i){
      if(updateLinearFlag[i] != 0){
        float distance = (millis()-updateLinearStart_time[i])/1000*line[i][6];
        float L = sqrt((line[i][0]-line[i][3])*(line[i][0]-line[i][3]) +(line[i][1]-line[i][4])*(line[i][1]-line[i][4])+(line[i][2]-line[i][5])*(line[i][2]-line[i][5]));
        if( distance >= L)
        {
          updateLinearFlag[i] = 0;
          continue;
        } 
        float distanceRatio = distance/L;
        float v_x = line[i][3] - line[i][0];
        float v_y = line[i][4] - line[i][1];
        float v_z = line[i][5] - line[i][2];
        MOVEP(i, line[i][0]+v_x*distanceRatio, line[i][1]+v_y*distanceRatio, line[i][2]+v_z*distanceRatio );
      }
      writeServo();
    }
  }
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
//  Serial.println("\n check number");
//  Serial.println(theta1);
//  Serial.println(theta2);
//  Serial.println(theta3);
//
//  Serial.println(check_x);
//  Serial.println(check_y);
//  Serial.println(check_z);
  
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
  if(ch == 'M'){
    if(Serial.available())
      ch = Serial.read();
    if( ch == 'P'){
      Serial.print("movep");
      
      int legNum = Serial.parseInt();
      float x = Serial.parseFloat();
      float y = Serial.parseFloat();
      float z = Serial.parseFloat();
      
      MOVEP(legNum, x, y, z);
      
      
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
  float robotWidth = 180;
  float robotHeight = 49;
  float robotL1 = 50;
  float robotL2 = 149;
  float robotL3 = 198;

  float th1 = 0;
  float th2 = 0;
  float th3 = 0;
  float centerPointX = 0;
  float centerPointY = 0;  
  float thr = 0;
  
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
  
  thr =atan((y-centerPointY)/(x-centerPointX))/PI*180 ;
  thr = (x-centerPointX)>0? thr: thr+180;
  
  float r = sqrt((x-centerPointX)*(x-centerPointX)+(y-centerPointY)*(y-centerPointY))- robotL1;
  
  float _z = z - robotHeight;
  float L = sqrt(r*r + _z*_z);
  float thL = atan(_z/r)/PI*180;
//  Serial.print(" thr: ");Serial.print(thr);
  th3 = acos((robotL2*robotL2+robotL3*robotL3-L*L)/(2*robotL2*robotL3))/PI*180;
  th2 = acos((robotL2*robotL2+L*L-robotL3*robotL3)/(2*robotL2*L))/PI*180 + thL;
  th1 = thr - 90*legNum;

  theta1 = th1-45;
  theta2 = th2;
  theta3 = th3-90;

  if(theta1>=360) theta1 = theta1-360;
  if(theta2>=360) theta2 = theta2-360;
  if(theta3>=360) theta3 = theta3-360;

  if(theta1<=-360) theta1 = theta1+360;
  if(theta2<=-360) theta2 = theta2+360;
  if(theta3<=-360) theta3 = theta3+360;
  
  return;
}

void degree2xyz(int legNum, float theta1,float theta2,float theta3, float& x, float& y, float& z)
{
  float robotWidth = 180;
  float robotHeight = 49;
  float robotL1 = 50;
  float robotL2 = 149;
  float robotL3 = 198;
  
  float th1 = theta1+45;
  float th2 = theta2;
  float th3 = theta3+90;
  float centerPointX = 0;
  float centerPointY = 0;

  float r = robotL2*cos(th2/180*PI) + robotL3*sin((th2+th3-90)/180*PI) + robotL1;
  float _z = robotL2*sin(th2/180*PI) - robotL3*cos((th2+th3-90)/180*PI);

  switch(legNum){
    case 0:
      th1 = th1;
      centerPointX = robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 1:
      th1 = th1+90;
      centerPointX = -1*robotWidth/2;
      centerPointY = robotWidth/2;
      break;
    case 2:
      th1 = th1+180;
      centerPointX = -1*robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    case 3:
      th1 = th1+270;
      centerPointX = robotWidth/2;
      centerPointY = -1*robotWidth/2;
      break;
    default:
      break;  
  }

  x = centerPointX + r * cos(th1/180*PI);
  y = centerPointY + r * sin(th1/180*PI);
  z = _z + robotHeight;
  return;
}
