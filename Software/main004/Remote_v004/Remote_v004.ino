/*----------------------------------------------------------------------------*/
//ESPnow twoway communication example with nested structures

//Wout Rijvers, 14-03-2022
/*-Library-includes-----------------------------------------------------------*/
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <math.h> 

//LCD library
#include <LiquidCrystal_I2C.h>

/*-Program-configuration------------------------------------------------------*/
//ESPnow configuration
const int modus = 0;  //Remote = 0, Car = 1

//Debug levels
#define DEBUG1 0
#define DEBUG2 0
#define DEBUG3 0


/*-Timer initialisations------------------------------------------------------*/
//Init global timer variable

//Timer 1
unsigned long T1_prevMs = 0; //millis
unsigned int T1_interval = 5; //millis

//Timer 1
unsigned long T2_prevMs = 0; //millis
unsigned int T2_interval = 25; //millis

//Broadcast-Error Watchdog
unsigned long W1_upload = 5000;

/*-I/O-pins-------------------------------------------------------------------*/
int JoyStick_X1 = 39; // Analog Pin  X
int JoyStick_Y1 = 36; // // Analog Pin  Y
int JoyStick_X2 = 35; // Analog Pin  X
int JoyStick_Y2 = 34; // // Analog Pin  Y
int JoyStick_X3 = 33; // Analog Pin  X
int JoyStick_Y3 = 32; // // Analog Pin  Y
int JoyStick_X4 = 26; // Analog Pin  X
int JoyStick_Y4 = 25; // // Analog Pin  Y

int Buzzer = 15; //Digital buzzer pin

int LED1 = 2;
int LED2 = 4;
int LED3 = 16;
int PZL1 = 27;
int PZL2 = 14;
int PZL3 = 12;


/*-Global-variables-initialisation--------------------------------------------*/
esp_now_peer_info_t peerInfo;

//MAC Address of your receiver 
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//LCD: set the LCD number of columns and rows
int lcdColumns = 20;
int lcdRows = 4;

// set LCD address, number of columns and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

//Puzzle variables
bool Puzzle1, Puzzle2, Puzzle3;
bool Level1, Level2, Level3;
int speedStraight;
int deadZone = 38;

//Questions
typedef struct Question_struct
{
  String question = "empty";
  bool answer = false;
  bool answered = false;
}Question_struct;
Question_struct questions[10];
byte actQuestion = 0;

//Colorsensor
int averageRGB = 0;
bool answeredTrue = false;
bool answeredFalse = false;
bool finished = false;

//Errors
bool macError = false;
bool espnowError = false;
bool peerError = false;
bool transmitError = false;
bool receiverError = false;

//Communicationsymbols
unsigned long lastUpload = 5000;
unsigned long lastDownload = 5000;

//Statemachine
enum STATE_enum {
  _INIT,
  _ERROR,
  _IDLE,
  _PUZZLE,
  _READY,
  _RUNNING
};
STATE_enum State = _INIT;

/*-Custom-characters----------------------------------------------------------*/
byte checkChar[] = {
  B00000,
  B00001,
  B00011,
  B10110,
  B11100,
  B01000,
  B00000,
  B00000
};
byte crossChar[] = {
  B00000,
  B01010,
  B00100,
  B01010,
  B00000,
  B00000,
  B00000,
  B00000
};
byte playChar[] = {
  B00000,
  B01000,
  B01100,
  B01110,
  B01100,
  B01000,
  B00000,
  B00000
};
byte stopChar[] = {
  B00000,
  B01110,
  B01110,
  B01110,
  B00000,
  B00000,
  B00000,
  B00000
};
byte updownloadChar[] = {
  B00100,
  B01110,
  B11111,
  B00000,
  B11111,
  B01110,
  B00100,
  B00000
};
byte uploadChar[] = {
  B00100,
  B01110,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};
byte downloadChar[] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B01110,
  B00100,
  B00000
};
byte arrowupChar[] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100
};

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
void tone(byte pin, int freq) {
  ledcSetup(0, 2000, 8); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG3)Serial.print("\r\nLast Packet Send Status:\t");
  if (DEBUG3)Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

  // initialize pins
  pinMode(JoyStick_X1, INPUT);
  pinMode(JoyStick_Y1, INPUT);
  pinMode(JoyStick_X2, INPUT);
  pinMode(JoyStick_Y2, INPUT);
  pinMode(JoyStick_X3, INPUT);
  pinMode(JoyStick_Y3, INPUT);
  pinMode(JoyStick_X4, INPUT);
  pinMode(JoyStick_Y4, INPUT);
  pinMode(Buzzer, OUTPUT); // Set Buzzer - pin 9 as an output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(PZL1, INPUT);
  pinMode(PZL2, INPUT);
  pinMode(PZL3, INPUT);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  // create custom characters
  lcd.createChar(0, checkChar);
  lcd.createChar(1, crossChar);
  lcd.createChar(2, playChar);
  lcd.createChar(3, stopChar);
  lcd.createChar(4, updownloadChar);
  lcd.createChar(5, uploadChar);
  lcd.createChar(6, downloadChar);
  lcd.createChar(7, arrowupChar);
  

  //LCD constant line
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1 Race-game");
  //ERROR sign
  lcd.setCursor(17, 0);
  lcd.write(1);
  //STOP sign
  lcd.setCursor(18, 0);
  lcd.write(3);
  //Upload sign
  lcd.setCursor(19, 0);
  lcd.write(5);
  //Startup text  
  lcd.setCursor(0, 1);
  lcd.print("Firmw vers: v0.0.3");
  lcd.setCursor(0, 2);
  lcd.print("Date: 11-05-2022");
  lcd.setCursor(0, 3);
  lcd.print("Auth:Stan,Kevin,Wout");

  //Startup Buzzer
  tone(Buzzer,2000);
  delay(500);
  tone(Buzzer,0);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1 Race-game");
  lcd.setCursor(17, 0);
  lcd.write(1);
  lcd.setCursor(18, 0);
  lcd.write(3);

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

  //Timer 2
  if ((millis() - T2_prevMs) > T2_interval) {
    T2_prevMs = millis();

    /*Serial.print("R:");
    Serial.print(Communication.Status.Color.red);
    Serial.print(",");
    Serial.print("G:");
    Serial.print(Communication.Status.Color.green);
    Serial.print(",");
    Serial.print("B:");
    Serial.println(Communication.Status.Color.blue);*/

    Serial.print("setpointLF:");
    Serial.print(Communication.Control.Motor.speedLF);
    Serial.print(",");
    Serial.print("speedLF:");
    Serial.print(Communication.Status.Motor.speedLF);
    Serial.print(",");
    Serial.print("errorLF:");
    Serial.print(Communication.Status.Motor.errorLF);
    Serial.print(",");
    Serial.print("setpointRF:");
    Serial.print(Communication.Control.Motor.speedRF);
    Serial.print(",");
    Serial.print("speedRF:");
    Serial.print(Communication.Status.Motor.speedRF);
    Serial.print(",");
    Serial.print("errorRF:");
    Serial.print(Communication.Status.Motor.errorRF);
    Serial.print(",");
    Serial.print("setpointLB:");
    Serial.print(Communication.Control.Motor.speedLB);
    Serial.print(",");
    Serial.print("speedLB:");
    Serial.print(Communication.Status.Motor.speedLB);
    Serial.print(",");
    Serial.print("errorLB:");
    Serial.print(Communication.Status.Motor.errorLB);
    Serial.print(",");
    Serial.print("setpointRB:");
    Serial.print(Communication.Control.Motor.speedRB);
    Serial.print(",");
    Serial.print("speedRB:");
    Serial.print(Communication.Status.Motor.speedRB);
    Serial.print(",");
    Serial.print("errorRB:");
    Serial.println(Communication.Status.Motor.errorRB);
    
    //State machine
    switch (State)
    {
      //INIT STATE
      case _INIT: {
        lcd.setCursor(0, 1);
        lcd.print("----Initializing----");
        //Clear screen
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");

        //Init questions (max 40 characters)
        questions[0].question = "(3 % 2) == 1";
        questions[0].answer = true;
        questions[0].answered = false;
      
        questions[1].question = "Een integer kan komma getallen bevatten";
        questions[1].answer = false;
        questions[1].answered = false;
      
        questions[2].question = "De auto wordt aangestuurd met een PLC";
        questions[2].answer = false;
        questions[2].answered = false;
        
        questions[3].question = "Bij kortsluiting gaan hoge stromen lopen";
        questions[3].answer = true;
        questions[3].answered = false;
      
        questions[4].question = "Het net heeft een frequentie van 60Hz";
        questions[4].answer = false;
        questions[4].answered = false;
        
        questions[5].question = "Het techlab zit op de 1e verdieping";
        questions[5].answer = true;
        questions[5].answered = false;
        
        questions[6].question = "Vanaf jaar 2 kies je een differientatie";
        questions[6].answer = true;
        questions[6].answered = false;
        
        questions[7].question = "Vraag nummer 7?";
        questions[7].answer = true;
        questions[7].answered = false;
        
        questions[8].question = "Vraag nummer 8?";
        questions[8].answer = true;
        questions[8].answered = false;
        
        questions[9].question = "Vraag nummer 9?";
        questions[9].answer = true;
        questions[9].answered = false;
        
        questions[10].question = "Vraag nummer 10?";
        questions[10].answer = true;
        questions[10].answered = false;
      
        //Startup random question
        actQuestion = byte(random(0, 11));

        //Save joystick values to motor control
        Communication.Control.Motor.speedLF = 0;
        Communication.Control.Motor.speedRF = 0;
        Communication.Control.Motor.speedLB = 0;
        Communication.Control.Motor.speedRB = 0;

        State = _ERROR;
      break;
      }

      //_ERROR STATE
      case _ERROR: {
        //Save joystick values to motor control
        Communication.Control.Motor.speedLF = 0;
        Communication.Control.Motor.speedRF = 0;
        Communication.Control.Motor.speedLB = 0;
        Communication.Control.Motor.speedRB = 0;

        if ((macError || transmitError || receiverError || espnowError || peerError) == false)
        {
          State = _IDLE;
        }
      break;
      }

      //_IDLE STATE
      case _IDLE: {
        lcd.setCursor(0, 1);
        lcd.print("--------IDLE--------");
        //Clear screen
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");

        //Save joystick values to motor control
        Communication.Control.Motor.speedLF = 0;
        Communication.Control.Motor.speedRF = 0;
        Communication.Control.Motor.speedLB = 0;
        Communication.Control.Motor.speedRB = 0;

        State = _PUZZLE;
      break;
      }

      //IDLE STATE
      case _PUZZLE: {
        lcd.setCursor(0, 1);
        lcd.print("---Fix the puzzle!--");
        //Clear screen
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");

        Puzzle1 = digitalRead(PZL1);
        Puzzle2 = digitalRead(PZL2);
        Puzzle3 = digitalRead(PZL3);

        if (Puzzle1) Level1 = true;
        else {
          Level1 = false;
          Level2 = false;
          Level3 = false;
        }
        
        if (Puzzle1 and Puzzle2) Level2 = true;
        else {
          Level2 = false;
          Level3 = false;
        }
        if (Puzzle1 and Puzzle2 and Puzzle3) Level3 = true;
        else Level3 = false;
        
        if(Puzzle1 == LOW){
            digitalWrite(LED1, HIGH);
        }
        else{
          digitalWrite(LED1, LOW);
        }
        if(Puzzle2 == LOW){
            digitalWrite(LED2, HIGH);
        }
        else{
          digitalWrite(LED2, LOW);
        }
        if(Puzzle3 == LOW){
            digitalWrite(LED3, HIGH);
        }
        else{
          digitalWrite(LED3, LOW);
        }

        //Save joystick values to motor control
        Communication.Control.Motor.speedLF = 0;
        Communication.Control.Motor.speedRF = 0;
        Communication.Control.Motor.speedLB = 0;
        Communication.Control.Motor.speedRB = 0;

        if ((Puzzle1 || Puzzle2 || Puzzle3) == 0)  //All puzzles correct
        {
          State = _READY;
        }
      break;
      }

      //Ready STATE
      case _READY: {
        lcd.setCursor(0, 1);
        lcd.print("-------Ready?-------");
        //Clear screen
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 3);
        lcd.print("                    ");
        
        Serial.print(Level1);
        Serial.print(Level2);
        Serial.println(Level3);        

        if (true)  //finishDetected
        {
          State = _RUNNING;
        }
      break;
      }

      //RUNNING STATE
      case _RUNNING: {
        lcd.setCursor(0, 1);
        lcd.print("-------Racing-------");

        //Question answered?
        if (answeredTrue or answeredFalse){
          questions[actQuestion].answered = true;
          //Boost or handicap
          if (questions[actQuestion].answer and answeredTrue){
            //Boost             
          }
          else if (questions[actQuestion].answer and answeredFalse){
            //Handicap             
          }
        }

        //New question
        while (questions[actQuestion].answered == true){ 
          actQuestion = byte(random(0, 11));
        }
        
        //Display question
        if (questions[actQuestion].question.length() <= 20){
          lcd.setCursor(0, 2);
          lcd.print(questions[0].question);
        }
        else if (questions[actQuestion].question.length() <= 40){
          lcd.setCursor(0, 2);
          lcd.print(questions[actQuestion].question.substring(0, 20));
          lcd.setCursor(0, 3);
          lcd.print(questions[actQuestion].question.substring(20, questions[actQuestion].question.length()));
        }
        else {
          lcd.setCursor(0, 2);
          lcd.print("Question to long...");
        }

        if (false)
        {
          State = _READY;
        }
      break;
      }
    }  
  }

  //RGB gemiddelde berekenen
  averageRGB = ( (Communication.Status.Color.red + Communication.Status.Color.green + Communication.Status.Color.blue) / 3);
  //Kleuren rood, groen en zwart detecteren
  if(abs(averageRGB - Communication.Status.Color.red) > 50){
    answeredTrue = false;
    answeredFalse = true;
    finished = false;
  }
  else if(abs(averageRGB - Communication.Status.Color.green) > 50){
    answeredTrue = true;
    answeredFalse = false;
    finished = false;
  }
  else if(abs(averageRGB) < 50){
    answeredTrue = false;
    answeredFalse = false;
    finished = true;
  }

  int x1, y1, x2, y2, x3, y3, x4, y4;
  x1 = analogRead(JoyStick_X1); //  X1
  y1 = analogRead(JoyStick_Y1); //  Y1
  x2 = analogRead(JoyStick_X2); //  X2
  y2 = analogRead(JoyStick_Y2); //  Y2
  x3 = analogRead(JoyStick_X3); //  X3
  y3 = analogRead(JoyStick_Y3); //  Y3
  x4 = analogRead(JoyStick_X4); //  X4
  y4 = analogRead(JoyStick_Y4); //  Y4
  
  //Map joystick values
  int X1 = map(x1, 0, 4095, -255, 255);
  int Y1 = map(y1, 0, 4095, -255, 255);
  int X2 = map(x2, 0, 4095, -255, 255);
  int Y2 = map(y2, 0, 4095, -255, 255);
  int X3 = map(x3, 0, 4095, -255, 255);
  int Y3 = map(y3, 0, 4095, -255, 255);
  int X4 = map(x4, 0, 4095, -255, 255);
  int Y4 = map(y4, 0, 4095, -255, 255);
  
  if (abs(X1) < deadZone) X1 = 0;
  if (abs(Y1) < deadZone) Y1 = 0;
  if (abs(X2) < deadZone) X2 = 0;
  if (abs(Y2) < deadZone) Y2 = 0;
  if (abs(X3) < deadZone) X3 = 0;
  if (abs(Y3) < deadZone) Y3 = 0;
  if (abs(X4) < deadZone) X4 = 0;
  if (abs(Y4) < deadZone) Y4 = 0;
  
  //Save joystick values to motor control
  if (State == _READY or State == _RUNNING){
  if (false){
    speedStraight = sqrt(Y1*Y1 + X1*X1);
  
    //forward right
    if ((Y1 > deadZone && X1 > deadZone)){
      Communication.Control.Motor.speedLF = speedStraight - (-1*X1);
      Communication.Control.Motor.speedLB = speedStraight - (-1*X1);
      Communication.Control.Motor.speedRF = speedStraight;
      Communication.Control.Motor.speedRB = speedStraight;
    }
    //forward left
    else if ((Y1 > deadZone && X1 < -deadZone)){
      Communication.Control.Motor.speedLF = speedStraight;
      Communication.Control.Motor.speedLB = speedStraight;
      Communication.Control.Motor.speedRF = speedStraight - X1;
      Communication.Control.Motor.speedRB = speedStraight - X1;
    }
    //forward
    else if ((Y1 > deadZone && X1 > -deadZone && X1 < deadZone)){
      Communication.Control.Motor.speedLF = speedStraight;
      Communication.Control.Motor.speedLB = speedStraight;
      Communication.Control.Motor.speedRF = speedStraight;
      Communication.Control.Motor.speedRB = speedStraight;
    }
    //back right
    else if ((Y1 < -deadZone && X1 > deadZone)){
      Communication.Control.Motor.speedLF = -1*(speedStraight - (-1*X1));
      Communication.Control.Motor.speedLB = -1*(speedStraight - (-1*X1));
      Communication.Control.Motor.speedRF = speedStraight;
      Communication.Control.Motor.speedRB = speedStraight;
    }
    //back left
    else if ((Y1 < -deadZone && X1 < -deadZone)){
      Communication.Control.Motor.speedLF = speedStraight;
      Communication.Control.Motor.speedLB = speedStraight;
      Communication.Control.Motor.speedRF = -1*(speedStraight - X1);
      Communication.Control.Motor.speedRB = -1*(speedStraight - X1);
    }
    //backwards
    else if ((Y1 < -deadZone && X1 > -deadZone && X1 < deadZone)){
      Communication.Control.Motor.speedLF = (-1)*speedStraight;
      Communication.Control.Motor.speedLB = (-1)*speedStraight;
      Communication.Control.Motor.speedRF = (-1)*speedStraight;
      Communication.Control.Motor.speedRB = (-1)*speedStraight;
    }
    //right
    else if ((abs(Y1) < deadZone && X1 > deadZone )){
      Communication.Control.Motor.speedLF = X1;
      Communication.Control.Motor.speedLB = X1;
      Communication.Control.Motor.speedRF = -X1;
      Communication.Control.Motor.speedRB = -X1;
    }
    //left
    else if ((abs(Y1) < deadZone && X1 < -deadZone )){
      Communication.Control.Motor.speedLF = -X1;
      Communication.Control.Motor.speedLB = -X1;
      Communication.Control.Motor.speedRF = X1;
      Communication.Control.Motor.speedRB = X1;
    }
    else {
      Communication.Control.Motor.speedLF = 0;
      Communication.Control.Motor.speedLB = 0;
      Communication.Control.Motor.speedRF = 0;
      Communication.Control.Motor.speedRB = 0;
    }
  }
    else if (true){
      Communication.Control.Motor.speedLF = Y1;
      Communication.Control.Motor.speedRF = Y2;
      Communication.Control.Motor.speedLB = Y1;
      Communication.Control.Motor.speedRB = Y2;
    }
    else if (Level1){
      Communication.Control.Motor.speedLF = Y1;
      Communication.Control.Motor.speedRF = Y2;
      Communication.Control.Motor.speedLB = Y3;
      Communication.Control.Motor.speedRB = Y4;
    }
    else{
      Communication.Control.Motor.speedLF = 0;
      Communication.Control.Motor.speedRF = 0;
      Communication.Control.Motor.speedLB = 0;
      Communication.Control.Motor.speedRB = 0;
    }
  }

 if( State == _RUNNING){
  //Play sign
  lcd.setCursor(18, 0);
  lcd.write(2);
 }
 else{
  //Stop sign
  lcd.setCursor(18, 0);
  lcd.write(3);
 }

 //Make sure lastDownload is always less or equal than current time
 if (lastDownload > millis() )
 {
  lastDownload = millis();
 }

 //Upload/download symbol handling
 if ( ( (millis() - lastUpload) < 300) and ( (millis() - lastDownload) < 300) ){
  //Upload+Download sign
  transmitError = false;
  receiverError = false;
  lcd.setCursor(19, 0);
  lcd.write(4);
 }
 else if ( (millis() - lastUpload) < 300){
  //Upload sign
  transmitError = false;
  receiverError = true;
  lcd.setCursor(19, 0);
  lcd.write(5);
 }
 else if ( (millis() - lastDownload) < 300) {
  //Download sign
  transmitError = true;
  receiverError = false;
  lcd.setCursor(19, 0);
  lcd.write(6);
 }
 else {
  //no communication: empty
  transmitError = true;
  receiverError = true;
  lcd.setCursor(19, 0);
  lcd.print(" ");
 }

 //Error message on display
 if (macError || transmitError || receiverError || espnowError || peerError) {
  lcd.setCursor(17, 0);
  lcd.write(1);
  //To _ERROR state
  State = _ERROR;
  if (macError) { lcd.setCursor(0, 1);           lcd.print("Mac error           ");}
  else if (transmitError) { lcd.setCursor(0, 1); lcd.print("Transmit error      ");}
  else if (receiverError) { lcd.setCursor(0, 1); lcd.print("Receiver error      ");}
  else if (espnowError) { lcd.setCursor(0, 1);   lcd.print("Espnow error        ");}
  else if (peerError) { lcd.setCursor(0, 1);     lcd.print("Peer error          ");}
  //Clear screen
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");
 }
 else if ((macError || transmitError || receiverError || espnowError || peerError) ==  false){
  lcd.setCursor(17, 0);
  lcd.write(0);
 }
}
