#include <TinyPinChange.h>
#include <SoftSerial.h>

// ########################## DEFINES ##########################
#define SERIAL_RX_TX_PIN 0   //RX not needed
#define START_FRAME 0xAC

#define LED_PIN 1

#define BUTTON_PIN 3    //Button press connects pin to gnd
#define BUTTON_DEBOUNCE_CNT 50    //debounce in ms

SoftSerial Serial1(SERIAL_RX_TX_PIN,SERIAL_RX_TX_PIN);  //(RX, TX)

uint16_t t = 0;
uint16_t t100ms = 0;
uint16_t t1000ms = 0;
uint8_t CntButton = 0; //Counting
uint8_t StEmergencyOff = 0;   // 0 = Encoder Off; 1 = Encoder On
uint8_t StEmergencyOffSwitched = 0;

typedef struct {
  uint8_t start;
  uint8_t StEmergencyOff;
} SerialCommand;
SerialCommand Command;

void setup() {                
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Serial1.begin(9600);
  Serial1.txMode();
}

void Task1ms(){
  uint8_t StButton = !digitalRead(BUTTON_PIN);

  if (CntButton == 0)
    StEmergencyOffSwitched = 0;
  
  if ((StButton == 0) && (CntButton > 0))
    CntButton--;
  else if ((StButton == 1) && (CntButton < BUTTON_DEBOUNCE_CNT))
    CntButton++;
  else if ((StButton == 1) && (CntButton >= BUTTON_DEBOUNCE_CNT) && (StEmergencyOffSwitched == 0))
  {
    StEmergencyOffSwitched = 1;
    StEmergencyOff = !StEmergencyOff;
    digitalWrite(LED_PIN, StEmergencyOff); 
  }
}

void Task100ms(){
  Command.start = (uint8_t)START_FRAME;
  Command.StEmergencyOff = StEmergencyOff;

  Serial1.write((uint8_t *)&Command, sizeof(Command));
}


void loop() {
  if ((uint16_t)millis() != t)  //1 ms task
  {
    t = (uint16_t)millis();
    Task1ms();
  }
  else if ((t - t100ms) >= 100)  //100 ms task
  {
    t100ms = t;
    Task100ms();
  }
  else if ((t - t1000ms) >= 1000)  
  {
    t1000ms = t;
    StEmergencyOff = !StEmergencyOff;       //Test-Case Toggle every Second
    digitalWrite(LED_PIN, StEmergencyOff);  //Test-Case Toggle every Second
  }
  
}