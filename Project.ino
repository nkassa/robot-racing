#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;
ButtonA buttonA;
ButtonB buttonB;
Sonar sonar(4);

#define minOutput -100.0
#define maxOutput 100.0

//separate Kp and Kd values for the leader and follower robots
#define kp_following 40
#define kd_following 5
#define kp_line 0.25
#define kd_line 1

//separate base speeds for the leader and follower robots
#define baseSpeed 50.0 //leader is faster
#define baseSpeed_ 45.0 //follower is slower


int flag = 0;
PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_following(kp_following, kd_following, minOutput, maxOutput);



const float distFromWall=15.0; //Distance from wall
double wallDist;
int side=0;

//linefollower
int calibrationSpeed = 60;
int pos;
unsigned int lineSensorValues[5];

void calibrateSensors()
{
  delay(1000);
  for(int i = 0; i < 80; i++){
    if (i > 20 && i <= 60){
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    }
    else{
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void setup()
{
  Serial.begin(9600);
  servo.attach(5);
  //check right and left
  servo.write(0);
  delay(3000);
  wallDist = sonar.readDist();
  servo.write(175);
  delay(3000);
  if(wallDist<sonar.readDist()){ //if distance from the other robot is within range of sonar sensor
     servo.write(0);
     delay(2000);
     side=1;
  }

  calibrateSensors();
}

void loop()
{
  //ditermining the robots behavior depending on which button is pressed. 
  
  if(buttonA.isPressed())   
  {
    flag = 1;
  }
  else if(buttonB.isPressed()) 
  {
    flag = 2;
  }
  if(flag == 1)
  {
    linefollowing();   //A means the robot will be line following
  }
  else if(flag == 2)
  {
    robotfollowing();  //B means the robot will be robot following
  }
}

void linefollowing()
{
    pos = lineSensors.readLineBlack(lineSensorValues); //readtheblackline
    float PDout = pd_line.update(pos, 2000);
    motors.setSpeeds(int(baseSpeed+PDout), int(baseSpeed-PDout));
}

void robotfollowing()
{
  //delay(40);
  wallDist = sonar.readDist();
  float PDout = pd_following.update(wallDist,distFromWall);

  if(side==0)
    motors.setSpeeds(baseSpeed_ - PDout, baseSpeed_ + PDout);
  else
    motors.setSpeeds(baseSpeed_ + PDout, baseSpeed_ - PDout);

}


