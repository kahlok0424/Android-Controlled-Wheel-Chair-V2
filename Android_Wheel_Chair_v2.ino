#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

ESP8266WebServer server(80);
// define pin
#define MOTOR_ENABLE_PIN        10
#define MOTOR_LEFT_STEP_PIN     4
#define MOTOR_LEFT_DIR_PIN      5
#define MOTOR_RIGHT_STEP_PIN    0
#define MOTOR_RIGHT_DIR_PIN     2
#define MOTOR_LEFT_FOWARD       1
#define MOTOR_LEFT_BACKWARD     0
#define MOTOR_RIGHT_FOWARD      0
#define MOTOR_RIGHT_BACKWARD    1

#define ENABLE                  LOW
#define DISABLE                 HIGH

#define _1u                     80

const char* ssid = "test123";
const char* password =  "testing12345";
volatile unsigned long next;
volatile int previoustime =0;
volatile int motorDelay =0;

/*#define updateIndex(x)    do {                              \
                    if((x)->updated == 1)                   \
                     {                                      \
                       (x)->index  = ((x)->index +1)&1;     \
                       (x)->updated =0;                     \
                     }                                      \
} while(0)  */                                                \

typedef struct MotorInfo MotorInfo;
struct  MotorInfo {
  unsigned long stepPeriod[2];
  int motorControlPin;
  int dirPin;
  int dir;  
  int steps;
  unsigned long prevTime;
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
  .Speed = 0.0,
  .Angle = 90.00,  
  .previousAngle = 270,
  .previousSpeed = 0.0 ,
};

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_LEFT_STEP_PIN, OUTPUT); 
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_STEP_PIN, OUTPUT); 
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  rightMotorInfo.stepPeriod[0] = -1;
  rightMotorInfo.motorControlPin = MOTOR_RIGHT_STEP_PIN;
  rightMotorInfo.dirPin = MOTOR_RIGHT_DIR_PIN;
  rightMotorInfo.dir = MOTOR_RIGHT_FOWARD;
  rightMotorInfo.steps = 0;
  rightMotorInfo.prevTime = 0;
  leftMotorInfo.stepPeriod[0] = -1;
  leftMotorInfo.motorControlPin = MOTOR_LEFT_STEP_PIN;
  leftMotorInfo.dirPin = MOTOR_LEFT_DIR_PIN;
  leftMotorInfo.dir = MOTOR_LEFT_FOWARD;
  leftMotorInfo.steps = 0;
  leftMotorInfo.prevTime = 0;
  disableMotor();
  WiFi.softAP(ssid, password,1,1);
    handling(&instruction,&leftMotorInfo , &rightMotorInfo );
    handleUturn(&leftMotorInfo , &rightMotorInfo);
  //  handlingDebug(&leftMotorInfo , &rightMotorInfo);
  server.begin();
  IPAddress myIP=WiFi.softAPIP();
  enableMotor();
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(rightMotorStep_test);
  next=ESP.getCycleCount()+2000;
  timer0_write(next);
  interrupts();
//  Serial.println(myIP);
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
  next=next+400000;
  timer0_write(next);
  //unsigned long stepPeriod = getStepPeriod(motorInfo);
  //updateIndex(motorInfo);
  
  if(TimerExpired(motorInfo->stepPeriod[0], motorInfo->prevTime))
  {
     digitalWrite(motorInfo->dirPin , motorInfo->dir);
     digitalWrite(motorInfo->motorControlPin,HIGH);
     digitalWrite(motorInfo->motorControlPin ,LOW);
     Serial.println(motorInfo->stepPeriod[0]);
     motorInfo->prevTime = micros();
     motorInfo-> steps ++ ;
  } 
}

void rightMotorStep_test()
{
  //next= rightMotorInfo.stepPeriod[0];
  next=next+rightMotorInfo.stepPeriod[0];
  timer0_write(next);
  //unsigned long stepPeriod = getStepPeriod(motorInfo);
  //updateIndex(motorInfo);
 
   digitalWrite(rightMotorInfo.dirPin ,0);
   digitalWrite(rightMotorInfo.motorControlPin,HIGH);
   digitalWrite(rightMotorInfo.motorControlPin,LOW);
   //Serial.println(rightMotorInfo->stepPeriod[0]);
     
  Serial.println("hooray!!");
  
}


void Uturn(MotorInfo *leftInfo , MotorInfo *rightInfo)
{
    leftInfo->dir = MOTOR_LEFT_FOWARD;
    leftInfo->stepPeriod[0] = 400;
    rightInfo->dir = MOTOR_RIGHT_BACKWARD;
    rightInfo  ->stepPeriod[0] = 400; 
  while( (leftInfo->steps <= 4000) && (rightInfo-> steps <= 4000) )
  {
      motorStep(leftInfo);
      motorStep(rightInfo);
  }
}

int getStepPeriod(MotorInfo *info)
{
  //updateIndex(info);
  return info->stepPeriod[0];
}

void ForceStop(MotorInfo *leftMotor , MotorInfo *rightMotor)
{
  //leftMotor-> stepPeriod[0] = -1;
  //rightMotor ->stepPeriod[0] = -1;
  disableMotor();
  //Serial.println("Stopped");
}

void enableMotor(){
  digitalWrite(MOTOR_ENABLE_PIN ,ENABLE);
  //Serial.println("Enable motor");
}

void disableMotor(){
  digitalWrite(MOTOR_ENABLE_PIN ,DISABLE);
  //Serial.println("Disable motor");
}

void Calculation(AngleSpeed *MainInfo , MotorInfo *leftMotor , MotorInfo *rightMotor){

    if(MainInfo->previousSpeed != MainInfo->Speed || MainInfo->previousAngle != MainInfo->Angle)
    { 
     // Serial.println("new speed and angle");
      //Serial.println("new speed and angle");
      //Serial.println(MainInfo->Speed);
      //Serial.println(MainInfo->Angle);
        enableMotor();
      
      MainInfo->previousSpeed = MainInfo->Speed;
      MainInfo->previousAngle = MainInfo->Angle;
      int max_period;
      float tempAngle;
      
     if(MainInfo->Speed == 0.0)
      {
      ForceStop(leftMotor ,rightMotor);
      }
      else{
        enableMotor();
        }

      if( MainInfo->Angle >= 180)
       {
         tempAngle = 360 - MainInfo->Angle;
         leftMotor->dir = MOTOR_LEFT_BACKWARD;
         rightMotor->dir = MOTOR_RIGHT_BACKWARD;
       }
       else{
        tempAngle = MainInfo->Angle;
        leftMotor->dir = MOTOR_LEFT_FOWARD;
        rightMotor->dir = MOTOR_RIGHT_FOWARD;
       }
            
      max_period = (80000 / MainInfo->Speed);
      leftMotor->stepPeriod[0] = ( max_period * ( tempAngle/180) ) + 200 ;
      rightMotor->stepPeriod[0] = ( max_period - ( max_period * ( tempAngle/180) ) ) + 200 ;
      }      
    }

void handling(AngleSpeed *info , MotorInfo *leftMotor, MotorInfo *rightMotor){
  server.on("/body", [=](){   
     if (server.hasArg("plain")== false){ //Check if body received

      server.send(200, "text/plain", "Body not received");
      return;
    }
    String message = "Body received:\n";
       message += server.arg("plain");
       message += "\n";

    server.send(200, "text/plain", message);
 //   Serial.println(message);

    StaticJsonBuffer<200> jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(server.arg("plain"));

    if(!root.success()){
//      Serial.println("parseObject() failed");
        return;
       }   
       /*if ( root.containsKey("offset") && root.containsKey("degrees") ){
         info->Speed = root["offset"];
         info->Angle = root["degrees"];
       }*/

        if ( root.containsKey("whichmotor") && root.containsKey("delay") && root.containsKey("direction") ){
        unsigned long stepPeriod;
        int direc; 
        int whichMotor;
       // whichMotor = root["whichmotor"];
       // MotorInfo *Stepinfo;
   
      //  Stepinfo = whichMotor == 1? leftMotor:rightMotor;
        stepPeriod = root["delay"];
        stepPeriod *= _1u;
        direc = root["direction"];
        enableMotor();

        noInterrupts();
        rightMotorInfo.stepPeriod[0] = stepPeriod;
        rightMotorInfo.dir = direc;
        timer0_write(next);
        interrupts();
        }
  });
}  

void handleUturn(MotorInfo *leftMotor , MotorInfo *rightMotor){
   server.on("/uturn", [=](){
     Uturn(leftMotor ,rightMotor);
  });
  return;
}

/*void handlingDebug(MotorInfo *leftMotor, MotorInfo *rightMotor){
  server.on("/debug", [=](){   
     if (server.hasArg("plain")== false){ //Check if body received

      server.send(200, "text/plain", "Body not received");
      return;
    }
    String message = "Body received:\n";
       message += server.arg("plain");
       message += "\n";

    server.send(200, "text/plain", message);
 //   Serial.println(message);

    StaticJsonBuffer<200> jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(server.arg("plain"));

    if(!root.success()){
//      Serial.println("parseObject() failed");
        return;
       }   
      int whichMotor;
      whichMotor = root["whichmotor"];
      MotorInfo *info;
   
        info = whichMotor == 1? leftMotor:rightMotor;
        info->stepPeriod[0] = root["delay"];
        info->dir = root["direction"];
  });
}*/
  
// the loop function runs over and over again forever
void loop() {
   // motorStep(&leftMotorInfo);
    yield();
    //motorStep(&rightMotorInfo);
    yield();
    server.handleClient();
    yield();
   // Calculation(&instruction , &leftMotorInfo ,&rightMotorInfo);
    yield();
    
}
