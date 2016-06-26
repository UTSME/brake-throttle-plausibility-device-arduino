//CAN-BUS Shield
#include <mcp_can.h>
#include <SPI.h>

//Can Bus
unsigned char Flag_Recv = 0;
unsigned char len = 0;
INT32U canId = 0x000;
unsigned char buf[8];
char str[20];

//Can Bus Send
unsigned char motortemp[3] = {61, 73, 100};
unsigned char rpm[3] = {61, 48, 100};
unsigned char act_current[3] = {61, 32, 100};
unsigned char cur_throttle[3] = {144, 0, 0};

//Can Bus Receive
int CDL3id = 0x507;
float canbusSpeed = 0; //From CDL3

//Flags
unsigned char Flag_MotorTemp = 0;
unsigned char Flag_RPM = 0;
unsigned char Flag_Current = 0;

//Data from Canbus;
float bmsCurrent = 0;

//I/O Pins
int mcDisable = 8; //MC Disable
int sdDisable = 9; //Shutdown Circuit Disable


//Analog Pins
int throttle1 = 0; //Throttle 1 ADC
int throttle2 = 1; //Throttle 2 ADC
int brake = 2;     //Brake ADC 
int current_sen = 5;

//Calibration Values
float throttleMin = 200; //Throttle Min
float throttlePeak = 900; //Throttle Peak
float throttle5p = (throttlePeak - throttleMin) * 0.10 + throttleMin; //Throttle Value at 5%
float throttle25p = (throttlePeak - throttleMin) * 0.45 + throttleMin; //Throttle Value at 25 percent
bool implausibilityTog = HIGH; //Implausibility toggle, that waits till throttle is less then 5%
bool torqueDiffBool = HIGH;    //TorqueDiff Bool
bool failureBool1 = HIGH;
bool failureBool2 = HIGH;
bool failureBool3 = HIGH;
float braking = 450; //Brake actuated Threshold //450
float highValue = 690; //Value when ADC signal is pulled high/disconnected
float throttleDifference = 170; //Two Throttle Sensor intial difference

//Values to use at track
//Braking 450
//throttle 25p 5p


//Disable Loop Status
bool mcBool = LOW; //HIGH: Motor Shutdown | LOW: Motor Ready
bool sdBool = HIGH;  //LOW: S/D OPEN | S/D CLOSED
bool sdLock = HIGH;

void setup() {
  
  CAN.begin(CAN_1000KBPS);                       // init can bus : baudrate = 1000k
  attachInterrupt(0, MCP2515_ISR, FALLING);     // start interrupt
  
  Serial.begin(115200);
  
  pinMode(sdDisable, OUTPUT);   
  pinMode(mcDisable, OUTPUT); 
  
  //Initialise disable pins
  digitalWrite(mcDisable, mcBool);
  digitalWrite(sdDisable, sdBool);
  
  START_INIT:

    if(CAN_OK == CAN.begin(CAN_1000KBPS))                   // init can bus : baudrate = 1000k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
    
 
    //Motortemp Loop
    while(buf[0] != 73){
      CAN.readMsgBuf(&len, buf);
      Serial.println("Motortemp Loop");
      Serial.println(buf[0]);
      CAN.sendMsgBuf(0x210, 0, 3, motortemp);
      delay(100);                       // send data per 100ms
    }
    
    //RPM Loop
    while(buf[0] != 48){
      CAN.readMsgBuf(&len, buf);
      Serial.println("RPM Loop");
      CAN.sendMsgBuf(0x210, 0, 3, rpm);
      delay(100);                       // send data per 100ms
    }
    
    //Actual Current
    while(buf[0] != 32){
      CAN.readMsgBuf(&len, buf);
      Serial.println("Act Curent Loop");
      CAN.sendMsgBuf(0x210, 0, 3, act_current);
      delay(100);                       // send data per 100ms
    }
    
   //detachInterrupt(0);
}

void MCP2515_ISR()
{
    Flag_Recv = 1;
    
    //Flag_Recv = 0;  
}

void loop() {
  
  //Update Disable pins
  mcBool = implausibilityTog;

  if(sdLock == LOW){digitalWrite(sdDisable, LOW);}
  else{
    sdBool = failureBool1 && failureBool2 && failureBool3;
    if(failureBool1 && failureBool2 && failureBool3){}else{sdLock = LOW;}
    digitalWrite(sdDisable, sdBool);
  }

  //Serial Prints
  Serial.print("(MC:");
  Serial.print(mcBool);
  Serial.print("|SD:");
  Serial.print(sdBool);
  Serial.print("|LOCK:");
  Serial.print(sdLock);
  Serial.print(")||");
  

  torqueImplausibility();
  torqueBrakeCheck();
  brakeFailure();

  
  Flag_Recv = 0;                        // clear flag
  CAN.readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
  //CDL3 Can ID
  if(CAN.getCanId() == CDL3id && len == 8){
      canbusSpeed = ((buf[0] * 256) + buf[1])/10;
  }
  Serial.print("||CanSpeed:");
  Serial.print(canbusSpeed);
  Serial.println("");
}

void torqueImplausibility(){
  // If implausibility occurs between the values of two torque encoder
  // sensors the power to the motor has to be immediately shut down
  // completely. It is not necessary to completely deacticvate the Tractive
  // System.
  // 10% Difference
  
  //Read current ADC values of throttle pots
  float throttle1Read = analogRead(throttle1) + throttleDifference;
  //throttle1Read = throttleDifference;
  float throttle2Read = analogRead(throttle2);// - throttleDifference; //Add initlal throttle difference
  float brakeRead = analogRead(brake);
  if(throttle1Read > 700){throttle2Read = throttle1Read;}
  //Get Percentage difference
  float throttleDiffP = throttle2Read / throttle1Read;//((throttle1Read / throttle2Read) * 100);
  
  //Use throttle Difference percentage to open or close 
  if(0.7 < throttleDiffP){  
    if(throttleDiffP < 1.3){
      torqueDiffBool = LOW;
    }else{
      torqueDiffBool = HIGH;
    }
  }else{
    torqueDiffBool = HIGH;
  }
  
  Serial.print("(Thr1:");
  Serial.print(throttle1Read);
  Serial.print("|Thr2:");
  Serial.print(throttle2Read);
  Serial.print("|PDiff:");
  Serial.print(throttleDiffP);
  Serial.print("|Bool:");
  Serial.print(torqueDiffBool);

  
  //If speed is above 5kph
  if(canbusSpeed > 5){
    brakeRead = constrain(map(brakeRead,110,1000,0,10000),0,10000);
    throttle1Read = constrain(map(throttle1Read,230,900,0,16380),0,16380);
    throttle1Read = brakeRead-throttle1Read;
  }else{
    throttle1Read = constrain(map(throttle1Read,230,900,0,16380),0,16380);
    throttle1Read = 0 - throttle1Read;
  }
  
  
  cur_throttle[1] = (int)throttle1Read % 256;
  cur_throttle[2] = (int)(throttle1Read / 256);
  CAN.sendMsgBuf(0x210, 0, 3, cur_throttle);
  
  Serial.print("|Map:");
  Serial.print(throttle1Read);
  Serial.print(",");
  Serial.print(cur_throttle[1]);
  Serial.print(",");
  Serial.print(cur_throttle[2]);
  Serial.print(")||");
 }

//WORKING
void torqueBrakeCheck(){
  // Torque encoder is at more than 25% and brake is actuated
  // simultaneously. The motors have to shut down. The motor power shut
  // down has to remain active until the torque encoder signals less than
  // 5% pedal travel, no matter whether the brake pedal is still actuated or
  // not.
  // implausibilityTog
  // --HIGH : mcBool HIGH
  // --LOW  : mcBool LOW
  
  // Get current ADC values
  int throttleRead = analogRead(throttle1) + throttleDifference;;
  int brakeRead = analogRead(brake);
  
  
  if(implausibilityTog){
    if(throttleRead < throttle5p){
      implausibilityTog = LOW;
      //
    }
  }else{
    if(throttleRead > throttle25p){
      if(brakeRead > braking){
        implausibilityTog = HIGH;
        sdLock = LOW;
      }
    }
  }
  
  Serial.print("(PlausTog:");
  Serial.print(implausibilityTog);
  Serial.print("|BrakingVal:");
  Serial.print(braking);
  Serial.print("|Thr5p:");
  Serial.print(throttle5p);
  Serial.print("|Thr25p:");
  Serial.print(throttle25p);
  Serial.print("|LOCK:");
  Serial.print(sdLock);
  Serial.print("|Throttle:");
  Serial.print(throttleRead);
  Serial.print("|BrakeRaw:");
  Serial.print(brakeRead);
  Serial.print(")||");
}    

//WORKING
void brakeFailure(){
  // When an analogue signal is used, e.g. from a 5V sensor, the brake system
  // encoder sensors will be considered to have failed when they achieve an
  // open circuit or short circuit condition which generates a signal outside
  // of the normal operating range, for example <0.5V or >4.5V. The circuitry
  // used to evaluate the sensor wilsl use pull down or pull up resistors to
  // ensure that open circuit signals result in a failure being detected
  
  int brakeRead = analogRead(brake);
  int throttle1Read = analogRead(throttle1);
  int throttle2Read = analogRead(throttle2);
  
  if(brakeRead >= highValue){failureBool1 = LOW;
  }else{failureBool1 = HIGH;}
  
  if(throttle1Read == 0){failureBool2 = LOW;
  }else{failureBool2 = HIGH;}
  
  if(throttle2Read == 0){failureBool3 = LOW;
  }else{failureBool3 = HIGH;}
  
  Serial.print("(fail:");
  Serial.print(failureBool1);
  Serial.print(failureBool2);
  Serial.print(failureBool3);
  Serial.print(")");
}
