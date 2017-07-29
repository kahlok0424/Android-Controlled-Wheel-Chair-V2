#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

ESP8266WebServer server(80);
// define pin
#define MOTOR_LEFT_STEP_PIN   4
#define MOTOR_LEFT_DIR_PIN    5
#define MOTOR_RIGHT_STEP_PIN  0
#define MOTOR_RIGHT_DIR_PIN   2
#define MOTOR_LEFT_FOWARD     1
#define MOTOR_LEFT_BACKWARD   0
#define MOTOR_RIGHT_FOWARD    0
#define MOTOR_RIGHT_BACKWARD  1

const char* ssid = "test123";
const char* password =  "testing12345";

typedef struct MotorInfo MotorInfo;
struct  MotorInfo {
  unsigned long stepPeriod[2];
  int motorControlPin;
  int pinState;
  int dirPin;
  int dir;  
  int steps;
  unsigned long prevTime;
  int updated;
  int index;
};

MotorInfo leftMotorInfo = {

};

MotorInfo rightMotorInfo = {

};

typedef struct AngleSpeed AngleSpeed;
struct  AngleSpeed {
  float Speed;
  float Angle;
  float previousAngle;
  float previousSpeed; 
};

AngleSpeed instruction ={
  .Speed = 0.0 ,
  .Angle = 90.00 ,
  .previousAngle = 270,
  .previousSpeed = 0,
};

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_LEFT_STEP_PIN, OUTPUT); 
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_STEP_PIN, OUTPUT); 
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  rightMotorInfo.stepPeriod[0] = -1,
  rightMotorInfo.motorControlPin = MOTOR_RIGHT_STEP_PIN,
  rightMotorInfo.pinState = LOW ,
  rightMotorInfo.dirPin = MOTOR_RIGHT_DIR_PIN,
  rightMotorInfo.dir = MOTOR_RIGHT_FOWARD,
  rightMotorInfo.steps = 0,
  rightMotorInfo.prevTime = 0,
  rightMotorInfo.updated =0,
  rightMotorInfo.index =0,

  leftMotorInfo.stepPeriod[0] = -1,
  leftMotorInfo.motorControlPin = MOTOR_LEFT_STEP_PIN,
  leftMotorInfo.pinState = LOW ,
  leftMotorInfo.dirPin = MOTOR_LEFT_DIR_PIN,
  leftMotorInfo.dir = MOTOR_LEFT_FOWARD,
  leftMotorInfo.steps = 0,
  leftMotorInfo.prevTime = 0,
  leftMotorInfo.updated =0,
  leftMotorInfo.index =0,

  WiFi.softAP(ssid, password,1,1);
  handling(&leftMotorInfo , &rightMotorInfo);
  handleUturn(&leftMotorInfo , &rightMotorInfo);
  server.begin();
  IPAddress myIP=WiFi.softAPIP();
  Serial.println(myIP);
//  delay(150);
}

int TimerExpired(unsigned long duration ,unsigned long previous )
{   
   unsigned long current = micros();
    if(duration != -1 && (current - previous >= duration))
    {
      return 1;
    }
    return 0;
}

void motorStep(MotorInfo *motorInfo)
{
  
  if(TimerExpired(motorInfo->stepPeriod[motorInfo->index], motorInfo->prevTime))
  {
    if(motorInfo->updated == 1)
    {
      motorInfo->index = (motorInfo->index + 1)&1;
      motorInfo->updated = 0;
    }
     digitalWrite(motorInfo->dirPin , motorInfo->dir);
     digitalWrite(motorInfo->motorControlPin,HIGH);
     digitalWrite(motorInfo->motorControlPin ,LOW);
     motorInfo->prevTime = micros();
     motorInfo-> steps ++ ;
  } 
}

void Uturn(MotorInfo *leftInfo , MotorInfo *rightInfo)
{
    int index = getStepPeriodIndex(leftInfo);
    int index2 = getStepPeriodIndex(rightInfo);
    leftInfo->dir = MOTOR_LEFT_FOWARD;
    leftInfo->stepPeriod[index] = 400;
    rightInfo->dir = MOTOR_RIGHT_BACKWARD;
    rightInfo  ->stepPeriod[index2] = 400; 
  while( (leftInfo->steps < 4000) && (rightInfo-> steps <4000) )
  {
      motorStep(leftInfo);
      motorStep(rightInfo);
  }
}

void ForceStop(MotorInfo *leftInfo , MotorInfo *rightInfo)
{
  int index = getStepPeriodIndex(leftInfo);
  int index2 = getStepPeriodIndex(rightInfo);
  leftInfo-> stepPeriod[index] = -1;
  rightInfo ->stepPeriod[index2] = -1;
}

int getStepPeriodIndex(MotorInfo *info)
{
  if(info->updated == 1)
  {
  info->index = (info->index +1) &1;
  info->updated =0;
  }
  return info->index;
  }

/*void Calculation(AngleSpeed *MainInfo , MotorInfo *leftMotor , MotorInfo *rightMotor)
{

   if(MainInfo->Speed == 0)
    {
    ForceStop(leftMotor ,rightMotor);
    return;
    }

   else if(MainInfo->previousSpeed != MainInfo->Speed || MainInfo->previousAngle != MainInfo->Angle)
    {
      MainInfo->previousSpeed = MainInfo->Speed;
      MainInfo->previousAngle = MainInfo->Angle;
      int max_period;
      
      if( MainInfo->Angle > 180)
       {
         MainInfo->Angle = 360 - MainInfo->Angle;
         leftMotor->dir = MOTOR_LEFT_BACKWARD;
         rightMotor->dir = MOTOR_RIGHT_BACKWARD;
       }
      max_period = (6000/MainInfo->Speed);
      leftMotor->delay = max_period * ( MainInfo->Angle/180);
      rightMotor->delay = max_period - leftMotor->delay;
   
      Serial.println(leftMotor -> delay);
      Serial.println(rightMotor -> delay);
    }
}*/

void handling(MotorInfo *leftMotor, MotorInfo *rightMotor){
  server.on("/body", [=](){   
     if (server.hasArg("plain")== false){ //Check if body received

      server.send(200, "text/plain", "Body not received");
      return;
    }
    String message = "Body received:\n";
       message += server.arg("plain");
       message += "\n";

    server.send(200, "text/plain", message);
    Serial.println(message);

    StaticJsonBuffer<200> jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(server.arg("plain"));

    if(!root.success()){
      Serial.println("parseObject() failed");
      return;
    }
    int whichmotor;
    unsigned long delays,dire;
    whichmotor = root["whichmotor"];
   
   if(whichmotor == 1 )
   {
      if(leftMotor->updated == 0)
      {
        leftMotor->stepPeriod[(leftMotor->index+1)&1] = root["delay"];
        leftMotor->dir = root["direction"];
        leftMotor->updated == 1;
      }
    }
    else
    {
       if(rightMotor->updated == 0)
       {
         rightMotor->stepPeriod[(rightMotor->index+1)&1] = root["delay"];
         rightMotor->dir = root["direction"];
         rightMotor->updated == 1;
       }
     }  
   //MainInfo->Speed = root["offset"];
   //MainInfo->Angle = root["degrees"];  
  });
}  

void handleUturn(MotorInfo *leftMotor , MotorInfo *rightMotor){
   server.on("/uturn", [=](){
     leftMotor->steps = 0;
     rightMotor->steps = 0;   
     Uturn(leftMotor ,rightMotor);
  });
  return;
}
  

// the loop function runs over and over again forever
void loop() {

    motorStep(&leftMotorInfo);
    yield();
    //Serial.println("got things to show");
    motorStep(&rightMotorInfo);
    yield();
    server.handleClient();
    yield();
 //   Calculation(&instruction , &leftMotorInfo ,&rightMotorInfo);
   // yield();
}
