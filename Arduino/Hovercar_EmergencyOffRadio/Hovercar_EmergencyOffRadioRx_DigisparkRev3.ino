#include <TinyPinChange.h>
#include <SoftSerial.h>

// ########################## DEFINES ##########################
#define SERIAL_RX_TX_PIN 0   //TX not needed
#define START_FRAME 0xAC
#define SERIAL_TIMEOUT 300      //Timeout for last valid UART Receive in millis
#define SERIAL_NUM_UNPLAUSIBLE 2  //Number of unplausible messages in a row tolerated
#define SERIAL_BAUD 2400

#define LED_PIN 1

#define ENCSWITCH_PIN 2      //Pin switches Encoders on/off via npn transistor

SoftSerial Serial1(SERIAL_RX_TX_PIN,SERIAL_RX_TX_PIN);  //(RX, TX)

uint16_t t = 0;
uint16_t t10ms = 0;

typedef struct {
  uint8_t start;
  uint8_t StEmergencyOff = 0;   // 0 = Encoder Off; 1 = Encoder On
} SerialFeedback;
SerialFeedback Feedback; 
SerialFeedback NewFeedback;
uint16_t tMillisUART = 0;
uint8_t UART_qlf = 0;   //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t NumUartQlfUnplaus = 0;    //number of unplausible messages received in a row

uint8_t idx = 0;         // Index for new data pointer
  uint16_t bufStartFrame;  // Buffer Start Frame
  byte *p;                 // Pointer declaration for the new received data
  byte incomingByte;
  byte incomingBytePrev;

void setup() {    
  Feedback.StEmergencyOff = 0;            
  pinMode(ENCSWITCH_PIN, OUTPUT);
  digitalWrite(ENCSWITCH_PIN,Feedback.StEmergencyOff);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,Feedback.StEmergencyOff);
  
  Serial1.begin(SERIAL_BAUD);
  Serial1.rxMode();
}

void Task1ms(){

}

void ReceiveUART() {
  // Check for new data availability in the Serial buffer
  if (Serial1.available()) {
    incomingByte = Serial1.read();                                       // Read the incoming byte
    bufStartFrame = incomingByte;  // Construct the start frame
  } else {
    return;
  }

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingByte;
    idx = 1;
  } else if (idx >= 1 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      tMillisUART = (uint16_t)millis();
      UART_qlf = 1;
      NumUartQlfUnplaus = 0;

    } else {
      tMillisUART = (uint16_t)millis();
      UART_qlf = 0;
      NumUartQlfUnplaus++;
      if (NumUartQlfUnplaus > 254)
        NumUartQlfUnplaus = 254;
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  // Update previous states
  incomingBytePrev = incomingByte;
}

void ReceiveUARTPlaus() {
  if (((uint16_t)millis() - tMillisUART) > SERIAL_TIMEOUT)  //Trigger Timeout
  {
    tMillisUART = (uint16_t)millis() - SERIAL_TIMEOUT - 1;  //pull tMillisUART behind actual millis to avoid overflow/runover-effects

    UART_qlf = 2;

    //Set UART Feedback to 0
    Feedback.StEmergencyOff = 0;
  }
  else if ((UART_qlf == 0) && (NumUartQlfUnplaus > SERIAL_NUM_UNPLAUSIBLE)) //unplausible message
  {
    //Set UART Feedback to 0
    Feedback.StEmergencyOff = 0;
  }
}

void Task10ms(){
  ReceiveUARTPlaus();

  digitalWrite(ENCSWITCH_PIN,Feedback.StEmergencyOff);
  digitalWrite(LED_PIN,Feedback.StEmergencyOff);
}


void loop() {
  ReceiveUART();
  if ((uint16_t)millis() != t)  //1 ms task
  {
    t = (uint16_t)millis();
    Task1ms();
  }
  else if ((t - t10ms) >= 10)  //10 ms task
  {
    t10ms = t;
    Task10ms();
  }
    
}