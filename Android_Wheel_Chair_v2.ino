#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

ESP8266WebServer server(80);
// define pin
#define DEVELOP
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
#define IS_MOTOR_ACTIVE(x)      ((x).reloadPeriod != -1)
  
const char* ssid = "test123";
const char* password =  "testing12345";
volatile unsigned long next;
volatile int previoustime =0;
volatile int motorDelay =0;
volatile int isTimerOn = 0;
/*#define updateIndex(x)    do {                              \
                    if((x)->updated == 1)                   \
                     {                                      \
                       (x)->index  = ((x)->index +1)&1;     \
                       (x)->updated =0;                     \
                     }                                      \
} while(0)  */                                                


//void handleUturn(MotorInfo *leftMotor , MotorInfo *rightMotor);
typedef struct MotorInfo MotorInfo;
struct  MotorInfo {
  unsigned long stepPeriod[2];
  unsigned long reloadPeriod;
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
  rightMotorInfo.stepPeriod[0] = 0;
  rightMotorInfo.reloadPeriod = -1;
  rightMotorInfo.motorControlPin = MOTOR_RIGHT_STEP_PIN;
  rightMotorInfo.dirPin = MOTOR_RIGHT_DIR_PIN;
  rightMotorInfo.dir = MOTOR_RIGHT_FOWARD;
  rightMotorInfo.steps = 0;
  rightMotorInfo.prevTime = 0;
  leftMotorInfo.stepPeriod[0] = 0;
  leftMotorInfo.reloadPeriod = -1;
  leftMotorInfo.motorControlPin = MOTOR_LEFT_STEP_PIN;
  leftMotorInfo.dirPin = MOTOR_LEFT_DIR_PIN;
  leftMotorInfo.dir = MOTOR_LEFT_FOWARD;
  leftMotorInfo.steps = 0;
  leftMotorInfo.prevTime = 0;
  disableMotor();
  WiFi.softAP(ssid, password,1,1);
  handling(&instruction,&leftMotorInfo , &rightMotorInfo );
//  handleUturn(&leftMotorInfo , &rightMotorInfo);
  //handlingDebug(&leftMotorInfo , &rightMotorInfo);
  server.begin();
  IPAddress myIP=WiFi.softAPIP();
  enableMotor();
  noInterrupts();
  timer0_isr_init();
  
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


void pulsePin(int pin){
   digitalWrite(pin,HIGH);
   digitalWrite(pin,LOW);
}

void pulseMotorOnTimeout(MotorInfo *motor){
//   if( motor->stepPeriod[0] < 2000 && motor->reloadPeriod != 0){
//     pulsePin(motor->motorControlPin);
//     motor->stepPeriod[0] += motor->reloadPeriod; 
//   }
 if( motor->stepPeriod[0] < 2000 && motor->reloadPeriod != 0){
  if(motor == &leftMotorInfo)
    pulsePin(MOTOR_LEFT_STEP_PIN);
   else
    pulsePin(MOTOR_RIGHT_STEP_PIN);
   motor->stepPeriod[0] += motor->reloadPeriod; 
  }
}

void leftMotorStep_test()
{
  //unsigned long stepPeriod = getStepPeriod(motorInfo);
  //updateIndex(motorInfo);

   //Serial.println(lefttMotorInfo->stepPeriod[0]);
   
   /*if( leftMotorInfo.stepPeriod[0] < 2000){
     pulsePin(MOTOR_LEFT_STEP_PIN);
     leftMotorInfo.stepPeriod[0] += leftMotorInfo.reloadPeriod; 
   }
   if(rightMotorInfo.stepPeriod[0] < 2000){
     pulsePin(MOTOR_RIGHT_STEP_PIN);
     rightMotorInfo.stepPeriod[0] += rightMotorInfo.reloadPeriod;
   }*/

   //-------------------
   /*pulseMotorOnTimeout(&leftMotorInfo);
   pulseMotorOnTimeout(&rightMotorInfo);
   
   if(leftMotorInfo.stepPeriod[0] < rightMotorInfo.stepPeriod[0]){
    next=ESP.getCycleCount()+leftMotorInfo.stepPeriod[0];
    timer0_write(next);
    rightMotorInfo.stepPeriod[0] -= leftMotorInfo.stepPeriod[0];
    leftMotorInfo.stepPeriod[0] = 0;
   }
   else{
     next=ESP.getCycleCount()+rightMotorInfo.stepPeriod[0];
     timer0_write(next);
     leftMotorInfo.stepPeriod[0] -= rightMotorInfo.stepPeriod[0];
     rightMotorInfo.stepPeriod[0] = 0;
   }*/
  int numOfActive = 0;
  unsigned long timeout;
  pulseMotorOnTimeout(&leftMotorInfo);
  pulseMotorOnTimeout(&rightMotorInfo);
  timeout = leftMotorInfo.stepPeriod[0];
  timeout = timeout <= rightMotorInfo.stepPeriod[0] ? timeout : rightMotorInfo.stepPeriod[0];
//  printf("timeout = %d\n", timeout);
  if(IS_MOTOR_ACTIVE(leftMotorInfo)){
   leftMotorInfo.stepPeriod[0] -= timeout;
   numOfActive++;
  }
  if(IS_MOTOR_ACTIVE(rightMotorInfo)){
   rightMotorInfo.stepPeriod[0] -= timeout;
   numOfActive++;
  }
  if(numOfActive > 0)
    timer0_write(ESP.getCycleCount() + timeout);
   else{
    printf("detach called");
    timer0_detachInterrupt();
    isTimerOn = 0;
   }
   

  /*next=ESP.getCycleCount()+79000;
  timer0_write(next);
  pulsePin(MOTOR_RIGHT_STEP_PIN);*/
   

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
  digitalWrite(rightMotorInfo.motorControlPin,LOW);
  digitalWrite(leftMotorInfo.motorControlPin,LOW);
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
    Serial.println("handling!");

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
        #ifdef DEVELOP
        if ( root.containsKey("whichmotor") && root.containsKey("delay") && root.containsKey("direction") ){
        unsigned long stepPeriod;
        int direc; 
        int whichMotor;
        whichMotor = root["whichmotor"];
        MotorInfo *Stepinfo;
        Serial.println("Entered the wifi function");
        Stepinfo = whichMotor == 1? leftMotor:rightMotor;
        stepPeriod = root["delay"];
        //stepPeriod *= _1u;
        direc = root["direction"];
        enableMotor();
        if(stepPeriod < 2000){
          stepPeriod = 2000;
        }
        Stepinfo->reloadPeriod = stepPeriod;
        Stepinfo->dir = direc;
        Serial.print("right reload period ");
        Serial.println(rightMotor->reloadPeriod);
        Serial.print("left reload period ");
        Serial.println(leftMotor->reloadPeriod);
        Serial.print("left step period ");
        Serial.println(leftMotor->stepPeriod[0]);
        Serial.print("right step period ");
        Serial.println(rightMotor->stepPeriod[0]);

        if(!isTimerOn){
          timer0_attachInterrupt(leftMotorStep_test);
          Serial.println("timer on!");
          isTimerOn = 1;
          noInterrupts();
          rightMotor->stepPeriod[0] = 0;
          leftMotor->stepPeriod[0] = 0;
          next=ESP.getCycleCount() + 2000;
          timer0_write(next);
          interrupts(); 
        }
       }
       #endif
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
