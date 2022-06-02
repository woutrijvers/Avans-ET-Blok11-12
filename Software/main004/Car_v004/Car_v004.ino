/*----------------------------------------------------------------------------*/
//ESPnow twoway communication example with nested structures

//Wout Rijvers, 14-03-2022
/*-Library-includes-----------------------------------------------------------*/
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <math.h> 
#include <PID_v1.h>



/*-Program-configuration------------------------------------------------------*/
//ESPnow configuration
const int modus = 1;  //Remote = 0, Car = 1

//Debug levels
#define DEBUG1 0
#define DEBUG2 1
#define DEBUG3 0 
#define noPaverageseInterrupts 1  

/*-Timer initialisations------------------------------------------------------*/
//Init global timer variable

//Timer 1
unsigned long T1_prevMs = 0; //millis
unsigned int T1_interval = 5; //millis

//Timer 2
unsigned long T2_prevMs = 0; //millis
unsigned int T2_interval = 10; //millis

//Broadcast-Error Watchdog
unsigned long W1_upload = 5000;

/*-I/O-pins-------------------------------------------------------------------*/
// Assignment of the sensor pins 
const int S2 = 16;
const int S3 = 15; 
const int sensorOut = 4; 

const int PWM_Forward_RV = 3;     // 
const int PWM_Back_RV    = 1;    // 
const int PWM_Forward_LV = 33;     // 
const int PWM_Back_LV    = 32;    //

const int PWM_Forward_RA = 25;//3;     // RX UART
const int PWM_Back_RA    = 26;//1;    // TX UART
const int PWM_Forward_LA = 22;     // 
const int PWM_Back_LA    = 23;    // 

const int ENC_LF = 27;
const int ENC_RF = 21;
const int ENC_LB = 14;
const int ENC_RB = 19;

/*-Global-variables-initialisation--------------------------------------------*/
esp_now_peer_info_t peerInfo;

//MAC Address of your receiver 
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Colorsensor variables
int operatingMode = 0;

//Errors
bool macError = false;
bool espnowError = false;
bool peerError = false;
bool transmitError = false;
bool receiverError = false;

//Communicationsymbols
unsigned long lastUpload = 5000;
unsigned long lastDownload = 5000;

/*Calibration values (must be updated before   updated before each use)*/ 
int redMin = 17857; int redMax = 47000; 
int greenMin = 13333; int greenMax = 33333; 
int blueMin = 18181; int blueMax = 41666;  

int redColor = 0; int greenColor = 0; int blueColor = 0;  
int redFrequency = 0; 
int redEdgeTime = 0; 
int greenFrequency = 0; 
int greenEdgeTime = 0; 
int blueFrequency = 0; int blueEdgeTime = 0;

//Motor control: setting PWM properties
const int freqPWM = 1000;
const int ledChannel_F_RV = 0;
const int ledChannel_B_RV = 1;
const int ledChannel_F_LV = 2;
const int ledChannel_B_LV = 3;
const int ledChannel_F_RA = 4;
const int ledChannel_B_RA = 5;
const int ledChannel_F_LA = 6;
const int ledChannel_B_LA = 7;
const int resolution = 8;

//Motor encoder timer: Auxiliary variables
unsigned long now = millis();
float perimeterF = 20000.0; //Hz
int perimeterWheel = 215; //mm
int maxSpeed = 600;

//Moving Average FIR
float mavg1_LF[5] = { 0 };
float mavg1_RF[5] = { 0 };
float mavg1_LB[5] = { 0 };
float mavg1_RB[5] = { 0 };
float mavg1_R[5] = { 0 };
float mavg1_G[5] = { 0 };
float mavg1_B[5] = { 0 };
float mavg_b[5] = { 0.2, 0.2, 0.2, 0.2, 0.2 };

//Encoder LF
unsigned long lastTrigger_LF = 0;
unsigned long encoderP_LF = 0;
unsigned long encoderT1_LF = 0;
unsigned long encoderT2_LF = 0;
unsigned long encoderF_LF = 0;  //mHz
int actualSpeed_LF = 0; //mm/s
bool encLF_runned = false;
//Encoder RF
unsigned long lastTrigger_RF = 0;
unsigned long encoderP_RF = 0;
unsigned long encoderT1_RF = 0;
unsigned long encoderT2_RF = 0;
unsigned long encoderF_RF = 0;  //mHz
int actualSpeed_RF = 0; //mm/s
bool encRF_runned = false;
//Encoder LB
unsigned long lastTrigger_LB = 0;
unsigned long encoderP_LB = 0;
unsigned long encoderT1_LB = 0;
unsigned long encoderT2_LB = 0;
unsigned long encoderF_LB = 0;  //mHz
int actualSpeed_LB = 0; //mm/s
bool encLB_runned = false;
//Encoder RB
unsigned long lastTrigger_RB = 0;
unsigned long encoderP_RB = 0;
unsigned long encoderT1_RB = 0;
unsigned long encoderT2_RB = 0;
unsigned long encoderF_RB = 0;  //mHz
int actualSpeed_RB = 0; //mm/s
bool encRB_runned = false;

//PID regulation
double Setpoint_LF, Input_LF, Output_LF;
double Setpoint_RF, Input_RF, Output_RF;
double Setpoint_LB, Input_LB, Output_LB;
double Setpoint_RB, Input_RB, Output_RB;

//PID tuning parameters
//double consKp=0.15, consKi=0.2, consKd=0.005;
double consKp=0.15, consKi=0.2, consKd=0.0008;
PID PID_LF(&Input_LF, &Output_LF, &Setpoint_LF, consKp, consKi, consKd, P_ON_E, DIRECT);
PID PID_RF(&Input_RF, &Output_RF, &Setpoint_RF, consKp, consKi, consKd, P_ON_E, DIRECT);
PID PID_LB(&Input_LB, &Output_LB, &Setpoint_LB, consKp, consKi, consKd, P_ON_E, DIRECT);
PID PID_RB(&Input_RB, &Output_RB, &Setpoint_RB, consKp, consKi, consKd, P_ON_E, DIRECT);

//Motor output variables
int Motor_LF = 0;
int Motor_RF = 0;
int Motor_LB = 0;
int Motor_RB = 0;

/*-Communication-structure----------------------------------------------------*/
//Status has to be sended from car to remote
//Control has to be sended from remote to car

typedef struct statusMotor_struct
{
  //LF = Left, Front
  bool errorLF;
  int speedLF;  //in mm/s
  //RF = Right, Front
  bool errorRF;
  int speedRF;  //in mm/s
  //LB = Left, Back
  bool errorLB;
  int speedLB;  //in mm/s
  //RB = Right, Back
  bool errorRB;
  int speedRB;  //in mm/s
}statusMotor_struct;

typedef struct statusColor_struct
{
  int red;
  int green;
  int blue;
}statusColor_struct;

typedef struct controlMotor_struct
{
  //LF = Left, Front
  int speedLF;  //in mm/s
  //RF = Right, Front
  int speedRF;  //in mm/s
  //LB = Left, Back
  int speedLB;  //in mm/s
  //RB = Right, Back
  int speedRB;  //in mm/s
}controlMotor_struct;

typedef struct controlColor_struct
{
  bool LEDs;  //False = Off, True = On
}controlColor_struct;

typedef struct Status_struct
{
  statusMotor_struct Motor;
  statusColor_struct Color;
}Status_struct;

typedef struct Control_struct
{
  controlMotor_struct Motor;
  controlColor_struct Color;
}Control_struct;

typedef struct Communication_struct
{
  Status_struct Status;
  Control_struct Control;
  //LF = Left, Front
  //int speedLF;  //in mm/s
}Communication_struct;

/*-Create-struct-Communication------------------------------------------------*/
Communication_struct Communication;
Communication_struct CommunicationBuffer;

/*-Functions------------------------------------------------------------------*/
//Intrupt function for encoder
void IRAM_ATTR intruptLF() 
{
  now = millis();
  if (now > lastTrigger_LF)
  {
    encoderT1_LF = now - lastTrigger_LF;
    encoderP_LF = encoderT1_LF + encoderT2_LF;
    lastTrigger_LF = now;
    encLF_runned = true;
    encoderT2_LF = encoderT1_LF;
  }
}

void IRAM_ATTR intruptRF() 
{
  now = millis();
  if (now > lastTrigger_RF)
  {
    encoderT1_RF = now - lastTrigger_RF;
    encoderP_RF = encoderT1_RF + encoderT2_RF;
    lastTrigger_RF = now;
    encRF_runned = true;
    encoderT2_RF = encoderT1_RF;
  }
}

void IRAM_ATTR intruptLB() 
{
  now = millis();
  if (now > lastTrigger_LB)
  {
    encoderT1_LB = now - lastTrigger_LB;
    encoderP_LB = encoderT1_LB + encoderT2_LB;
    lastTrigger_LB = now;
    encLB_runned = true;
    encoderT2_LB = encoderT1_LB;
  }
}

void IRAM_ATTR intruptRB() 
{
  now = millis();
  if (now > lastTrigger_RB)
  {
    encoderT1_RB = now - lastTrigger_RB;
    encoderP_RB = encoderT1_RB + encoderT2_RB;
    lastTrigger_RB = now;
    encRB_runned = true;
    encoderT2_RB = encoderT1_RB;
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG3)Serial.print("\r\nLast Packet Send Status:\t");
  if (DEBUG3)Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  return;
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&CommunicationBuffer, incomingData, sizeof(CommunicationBuffer));
  if (modus == 0)
  {
    Communication.Status = CommunicationBuffer.Status;
    lastDownload = millis();
  }
  else if (modus == 1)
  {
    Communication.Control = CommunicationBuffer.Control;
    lastDownload = millis();
  }
  if (DEBUG3) Serial.print("\r\nBytes received:\t");
  if (DEBUG3) Serial.println(len);
  return;
}

/*-Setup----------------------------------------------------------------------*/
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  uint8_t transmitAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  WiFi.macAddress(transmitAddress);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    espnowError = true;
  }

  if (modus == 0)
  {
    uint8_t receiverAddress[] = {0x10, 0x52, 0x1C, 0x5B, 0x16, 0x0C};
  }
  else 
  {
    uint8_t receiverAddress[] = {0xEC, 0x94, 0xCB, 0x4A, 0x4F, 0xD4};
  }

  Serial.print("Sender Adress:");
  Serial.println(transmitAddress[0]);
  Serial.print("Receiver Adress:");
  Serial.println(receiverAddress[0]);

  //Error: sendAdress is the same as broadcastAdress
  if ((transmitAddress[0] == receiverAddress[0]) and (transmitAddress[5] == receiverAddress[5]))
  {
    Serial.println("Error: transmitAdress is the same as receiverAdress");
    macError = true;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    peerError = true;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  /*definition of the sensor pins*/  
  //Scaling the output frequency     S0/S1     LOW/LOW=AUS, LOW/HIGH=2%,     HIGH/LOW=20%, HIGH/HIGH=100% 
  pinMode(S2, OUTPUT);   
  pinMode(S3, OUTPUT);   
  pinMode(sensorOut, INPUT);    

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_F_RV, freqPWM, resolution);
  ledcSetup(ledChannel_B_RV, freqPWM, resolution);
  ledcSetup(ledChannel_F_LV, freqPWM, resolution);
  ledcSetup(ledChannel_B_LV, freqPWM, resolution);
  ledcSetup(ledChannel_F_RA, freqPWM, resolution);
  ledcSetup(ledChannel_B_RA, freqPWM, resolution);
  ledcSetup(ledChannel_F_LA, freqPWM, resolution);
  ledcSetup(ledChannel_B_LA, freqPWM, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_Forward_RV, ledChannel_F_RV);
  ledcAttachPin(PWM_Back_RV, ledChannel_B_RV);
  ledcAttachPin(PWM_Forward_LV, ledChannel_F_LV);
  ledcAttachPin(PWM_Back_LV, ledChannel_B_LV);
  ledcAttachPin(PWM_Forward_RA, ledChannel_F_RA);
  ledcAttachPin(PWM_Back_RA, ledChannel_B_RA);
  ledcAttachPin(PWM_Forward_LA, ledChannel_F_LA);
  ledcAttachPin(PWM_Back_LA, ledChannel_B_LA);

  // Intrupt encoder INPUT_PULLUP
  pinMode(ENC_LF, INPUT);
  pinMode(ENC_RF, INPUT);
  pinMode(ENC_LB, INPUT);
  pinMode(ENC_RB, INPUT);
  // Set encoder pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(ENC_LF), intruptLF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RF), intruptRF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB), intruptLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB), intruptRB, CHANGE);

  //Turn PID regulators on
  PID_LF.SetMode(AUTOMATIC);
  PID_RF.SetMode(AUTOMATIC);
  PID_LB.SetMode(AUTOMATIC);
  PID_RB.SetMode(AUTOMATIC);

  PID_LF.SetSampleTime(1);
  PID_RF.SetSampleTime(1);
  PID_LB.SetSampleTime(1);
  PID_RB.SetSampleTime(1);
}

/*-Loop-----------------------------------------------------------------------*/
void loop() {

  //Timer 1
  if ((millis() - T1_prevMs) > T1_interval){
    T1_prevMs = millis();
    
    //Sent control struct
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &Communication, sizeof(Communication));
    if (result != ESP_OK) {
      Serial.println("Error sending the data");
      W1_upload = millis();
    }
    else if (result == ESP_OK){
      lastUpload = millis();
    }

    if ((millis() - W1_upload) < (T1_interval*20)){
      transmitError = true;
    }
    else{
      transmitError = false;
    }
  }

  if (encLF_runned) {
    encLF_runned = false;
    //Calculate frequentie and speed in mm/s
    encoderF_LF = (1000000 / encoderP_LF);         //mHz
    if (encoderF_LF < 60000) {
      int actualSpeedValue = round((float(encoderF_LF) / perimeterF) * perimeterWheel);  //mm/s

      //Moving average FIR
      mavg1_LF[0] = actualSpeedValue;
      float mavg1_y = mavg_b[0] * mavg1_LF[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_LF[i];
        mavg1_LF[i] = mavg1_LF[i-1];
      }
      actualSpeed_LF = mavg1_y;
    }
  }
  else if (millis() - lastTrigger_LF > 100){
    actualSpeed_LF = 0;
  }

  if (encRF_runned) {
    encRF_runned = false;
    //Calculate frequentie and speed in mm/s
    encoderF_RF = (1000000 / encoderP_RF);         //mHz
    if (encoderF_RF < 60000) {
      int actualSpeedValue = round((float(encoderF_RF) / perimeterF) * perimeterWheel);  //mm/s

      //Moving average FIR
      mavg1_RF[0] = actualSpeedValue;
      float mavg1_y = mavg_b[0] * mavg1_RF[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_RF[i];
        mavg1_RF[i] = mavg1_RF[i-1];
      }
      actualSpeed_RF = mavg1_y;
    }
  }
  else if (millis() - lastTrigger_RF > 100){
    actualSpeed_RF = 0;
  }

  if (encLB_runned) {
    encLB_runned = false;
    //Calculate frequentie and speed in mm/s
    encoderF_LB = (1000000 / encoderP_LB);         //mHz
    if (encoderF_LB < 60000) {
      int actualSpeedValue = round((float(encoderF_LB) / perimeterF) * perimeterWheel);  //mm/s

      //Moving average FIR
      mavg1_LB[0] = actualSpeedValue;
      float mavg1_y = mavg_b[0] * mavg1_LB[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_LB[i];
        mavg1_LB[i] = mavg1_LB[i-1];
      }
      actualSpeed_LB = mavg1_y;
    }
  }
  else if (millis() - lastTrigger_LB > 100){
    actualSpeed_LB = 0;
  }

  if (encRB_runned) {
    encRB_runned = false;
    //Calculate frequentie and speed in mm/s
    encoderF_RB = (1000000 / encoderP_RB);         //mHz
    if (encoderF_RB < 60000) {
      int actualSpeedValue = round((float(encoderF_RB) / perimeterF) * perimeterWheel);  //mm/s

      //Moving average FIR
      mavg1_RB[0] = actualSpeedValue;
      float mavg1_y = mavg_b[0] * mavg1_RB[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_RB[i];
        mavg1_RB[i] = mavg1_RB[i-1];
      }
      actualSpeed_RB = mavg1_y;
    }
  }
  else if (millis() - lastTrigger_RB > 100){
    actualSpeed_RB = 0;
  }

  //Feedback of speed to remote
  Communication.Status.Motor.speedLF = actualSpeed_LF;
  Communication.Status.Motor.speedRF = actualSpeed_RF;
  Communication.Status.Motor.speedLB = actualSpeed_LB;
  Communication.Status.Motor.speedRB = actualSpeed_RB;
  if (abs(Setpoint_LF - actualSpeed_LF) > 50){
    Communication.Status.Motor.errorLF = true;
  }
  else {
    Communication.Status.Motor.errorLF = false;
  }
  if (abs(Setpoint_RF - actualSpeed_RF) > 50){
    Communication.Status.Motor.errorRF = true;
  }
  else {
    Communication.Status.Motor.errorRF = false;
  }
  if (abs(Setpoint_LB - actualSpeed_LB) > 50){
    Communication.Status.Motor.errorLB = true;
  }
  else {
    Communication.Status.Motor.errorLB = false;
  }
  if (abs(Setpoint_RB - actualSpeed_RB) > 50){
    Communication.Status.Motor.errorRB = true;
  }
  else {
    Communication.Status.Motor.errorRB = false;
  }

  Setpoint_LF = map(abs(Communication.Control.Motor.speedLF), 0, 255, 0, maxSpeed);
  Setpoint_RF = map(abs(Communication.Control.Motor.speedRF), 0, 255, 0, maxSpeed);
  Setpoint_LB = map(abs(Communication.Control.Motor.speedLB), 0, 255, 0, maxSpeed);
  Setpoint_RB = map(abs(Communication.Control.Motor.speedRB), 0, 255, 0, maxSpeed);
  
  Input_LF = actualSpeed_LF;
  Input_RF = actualSpeed_RF;
  Input_LB = actualSpeed_LB;
  Input_RB = actualSpeed_RB;

  PID_LF.Compute();
  PID_RF.Compute();
  PID_LB.Compute();
  PID_RB.Compute();

  Motor_LF = Output_LF;
  Motor_RF = Output_RF;
  Motor_LB = Output_LB;
  Motor_RB = Output_RB;
  
  //Timer 2
  if ((millis() - T2_prevMs) > T2_interval){
    T2_prevMs = millis();

    if (DEBUG2){
      /*Serial.print("RB_SET:");
      Serial.print(Setpoint_RB);
      Serial.print(",");
      Serial.print("RB_IN:");
      Serial.print(Input_RB);
      Serial.print(",");
      Serial.print("RB_OUT:");
      Serial.print(Output_RB);
      Serial.print(",");
      Serial.print("RF_SET:");
      Serial.print(Setpoint_RF);
      Serial.print(",");
      Serial.print("RF_IN:");
      Serial.print(Input_RF);
      Serial.print(",");
      Serial.print("RF_OUT:");
      Serial.println(Output_RF);*/

      /*Serial.print("LB_SET:");
      Serial.print(Setpoint_LB);
      Serial.print(",");
      Serial.print("LB_IN:");
      Serial.print(Input_LB);
      Serial.print(",");
      Serial.print("LB_OUT:");
      Serial.println(Output_LB);
      Serial.print(",");*/
      Serial.print("LF_SET:");
      Serial.print(Setpoint_LF);
      Serial.print(",");
      Serial.print("LF_IN:");
      Serial.print(Input_LF);
      Serial.print(",");
      Serial.print("LF_OUT:");
      Serial.println(Output_LF);
    }
    
    //speedLF forward and backwards
    if (Communication.Control.Motor.speedLF > 0) {
      ledcWrite(ledChannel_F_LV, Motor_LF);
    }
    else{
      ledcWrite(ledChannel_F_LV, 0);
    }
    if (Communication.Control.Motor.speedLF < 0){
      ledcWrite(ledChannel_B_LV, Motor_LF);
    }
    else{
      ledcWrite(ledChannel_B_LV, 0);
    }
  
    //speedRF forward and backwards
    if (Communication.Control.Motor.speedRF > 0) {
      ledcWrite(ledChannel_F_RV, Motor_RF);
    }
    else{
      ledcWrite(ledChannel_F_RV, 0);
    }
    if (Communication.Control.Motor.speedRF < 0){
      ledcWrite(ledChannel_B_RV, Motor_RF);
    }
    else{
      ledcWrite(ledChannel_B_RV, 0);
    }
  
    //speedLB forward and backwards
    if (Communication.Control.Motor.speedLB > 0) {
      ledcWrite(ledChannel_F_LA, Motor_LB);
    }
    else{
      ledcWrite(ledChannel_F_LA, 0);
    }
    if (Communication.Control.Motor.speedLB < 0){
      ledcWrite(ledChannel_B_LA, Motor_LB);
    }
    else{
      ledcWrite(ledChannel_B_LA, 0);
    }
  
    //speedRB forward and backwards
    if (Communication.Control.Motor.speedRB > 0) {
      ledcWrite(ledChannel_F_RA, Motor_RB);
    }
    else{
      ledcWrite(ledChannel_F_RA, 0);
    }
    if (Communication.Control.Motor.speedRB < 0){
      ledcWrite(ledChannel_B_RA, Motor_RB);
    }
    else{
      ledcWrite(ledChannel_B_RA, 0);
    }
  }

  operatingMode = 8;

  switch (operatingMode){
    case 0:
    {    
      //Determination of the photodiode type during measurement       S2/S3       LOW/LOW=RED, LOW/HIGH=BLUE,       HIGH/HIGH=GREEN, HIGH/LOW=CLEAR     
      digitalWrite(S2, LOW);     
      digitalWrite(S3, LOW);       
      
      //Frequency measurement of the specified color and its as-signment to an RGB value between 0-255     
      float(redEdgeTime) = pulseIn(sensorOut, HIGH, 50000) + pulseIn(sensorOut, LOW, 50000);     float(redFrequency) = (1 / (redEdgeTime / 1000000));     
      redColor = map(redFrequency, redMax, redMin, 255, 0);     if (redColor > 255) {       redColor = 255;     }     if (redColor < 0) {       redColor = 0;     }     
      
      //Moving average FIR
      mavg1_R[0] = redColor;
      float mavg1_y = mavg_b[0] * mavg1_R[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_R[i];
        mavg1_R[i] = mavg1_R[i-1];
      }
      Communication.Status.Color.red = mavg1_y;

      //Output of frequency mapped to 0-255
      operatingMode+=1;
      Serial.print("R:");     
      Serial.print(Communication.Status.Color.red);
      Serial.print(",");
    }
    break;   

    case 1: 
    {
      digitalWrite(S2, HIGH);     
      digitalWrite(S3, HIGH);     
      
      //Frequency measurement of the specified color and its as-signment to an RGB value between 0-255
      float(greenEdgeTime) = pulseIn(sensorOut, HIGH, 50000) + pulseIn(sensorOut, LOW, 50000);     
      float(greenFrequency) = (1 / (greenEdgeTime / 1000000));     
      greenColor = map(greenFrequency, greenMax, greenMin, 255, 0);     if (greenColor > 255) {       greenColor = 255;     }     
      if (greenColor < 0) {       greenColor = 0;     }     
      
      //Moving average FIR
      mavg1_G[0] = greenColor;
      float mavg1_y = mavg_b[0] * mavg1_G[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_G[i];
        mavg1_G[i] = mavg1_G[i-1];
      }
      Communication.Status.Color.green = mavg1_y;

      //Output of frequency mapped to 0-255
      operatingMode+=1;
      Serial.print("G:");     
      Serial.print(Communication.Status.Color.green);
      Serial.print(",");
    }  
    break;

    case 2:
    {
      digitalWrite(S2, LOW);     
      digitalWrite(S3, HIGH);     
      
      //Frequency measurement of the specified color and its as-signment to an RGB value between 0-255  
      float(blueEdgeTime) = pulseIn(sensorOut, HIGH, 50000) + pulseIn(sensorOut, LOW, 50000);     
      float(blueFrequency) = (1 / (blueEdgeTime / 1000000));     blueColor = map(blueFrequency, blueMax, blueMin, 255, 0);     
      if (blueColor > 255) {       blueColor = 255;     }     if (blueColor < 0) {       blueColor = 0;     }     
      
      //Moving average FIR
      mavg1_B[0] = blueColor;
      float mavg1_y = mavg_b[0] * mavg1_B[0];
      
      for (int i= 5-1; i>0; i--){
        mavg1_y += mavg_b[i] * mavg1_B[i];
        mavg1_B[i] = mavg1_B[i-1];
      }
      Communication.Status.Color.blue = mavg1_y;

      //Output of frequency mapped to 0-255
      operatingMode+=1;
      Serial.print("B:");     
      Serial.print(Communication.Status.Color.blue);
      Serial.print(",");
    }
    break;      
  }

  if (DEBUG1){  
      Serial.print("R:");     
      Serial.print(redColor);
      Serial.print(",");
      Serial.print("G:");     
      Serial.print(greenColor);
      Serial.print(",");
      Serial.print("B:");     
      Serial.println(blueColor);
  }
  if (DEBUG3){  
      Serial.print("FR:");     
      Serial.print(redFrequency);
      Serial.print(",");
      Serial.print("FG:");     
      Serial.print(greenFrequency);
      Serial.print(",");
      Serial.print("FB:");     
      Serial.println(blueFrequency);
  }

  //Upload/download safety
   if ( ( (millis() - lastUpload) < 300) and ( (millis() - lastDownload) < 300) ){
    //Upload+Download sign
    receiverError = false;
   }
   else {
    //no communication: empty
    receiverError = true;
    //All motors off
    Communication.Control.Motor.speedLF = 0;
    Communication.Control.Motor.speedRF = 0;
    Communication.Control.Motor.speedLB = 0;
    Communication.Control.Motor.speedRB = 0;
   }
  
}
