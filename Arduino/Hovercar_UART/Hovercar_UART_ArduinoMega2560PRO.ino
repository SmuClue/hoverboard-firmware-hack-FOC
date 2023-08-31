
// ########################## DEFINES ##########################
// UART
#define HOVER_SERIAL_BAUD 115200  // [-] Baud rate for Serial3 (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200        // [-] Baud rate for USB Serial
#define BT_SERIAL_BAUD 38400        // [-] Baud rate Bluetooth Serial2 (used for SerialReport)
#define START_FRAME 0xABCD        // [-] Start frme definition for reliable serial communication
#define UART_GND_PIN 12           //GND for UART on Serial 3 (pins 15, 14)

// Accelerator ADC
#define ACCLRT_SUPPLY_PIN A0     //5V Supply for Accelerator
#define ACCLRT_SENS_PIN A2       //Sensor ADC for Accelerator
#define ACCLRT_GND_PIN A4        //GND Supply for Accelerator
#define ACCLRT_ADC_MIN 210       //MIN-Value of ADC over wich TrqRequest starts
#define ACCLRT_ADC_MIN_DIAG 170  //MIN-Threshold of ADC for Diagnosis
#define ACCLRT_ADC_MAX 810       //MIN-Value of ADC over wich TrqRequest starts
#define ACCLRT_ADC_MAX_DIAG 870  //MIN-Threshold of ADC for Diagnosis
#define ACCLRT_ADC_GRD_DIAG 300   //MAX-Absolute change of ADC over 1 Cycle for Diagnosis
#define ACCLRT_ERRCNTMAX   5          //max number of Error-Counter bevore Qlf is set to invalid
#define ACCLRT_TRQCMD_MAX 500   //Command @ ADC_MAX (Max = 1000)
#define ACCLRT_TRQCMD_MIN 0    //Command @ ADC_MIN

//RC Receiver PINS
#define RCRCV_SUPPLY_PIN 31      //5V Supply RC-Receiver
#define RCRCV_GND_PIN 30         //GND Supply RC-Receiver
#define RCRCV_CH2_PIN 20         //PWM In for RC-Receiver CH2 (Throttle)
#define RCRCV_CH1_PIN 21         //PWM In for RC-Receiver CH1 (Steering)
#define RCRCV_CH5_PIN 2         //PWM In for RC-Receiver CH5 (Drehknopf)
#define RCRCV_CH6_PIN 3         //PWM In for RC-Receiver CH6 (Drehknopf)
#define RCRCV_CH4_PIN 18         //PWM In for RC-Receiver CH4 (3-Way-Switch)
#define RCRCV_CH3_PIN 19        //PWM In for RC-Receiver CH4 (3-Way-Switch)

//DC-Relais PINS
#define DCRELAIS_PIN 8          //Digital out controlling DC-Relais

//CH2 RC Throttle (RcRcv_TrqCmd)
#define RCRCV_CH2_TD_MIN  900           //Min Duty-Time in micros 
#define RCRCV_CH2_TD_ZERO 1500          //Middle/Zero/RC-Off Duty-Time in micros
#define RCRCV_CH2_TD_MAX  2100          //Max Duty-Time in micros
#define RCRCV_CH2_TD_DEADBAND 40        //Deadband Duty-Time around TD_ZERO in micros
#define RCRCV_CH2_TD_MAX_DIAG  2200     //Max Duty-Time in micros plausible
#define RCRCV_CH2_TD_MIN_DIAG  800      //Min Duty-Time in micros plausible 
#define RCRCV_CH2_TD_GRD_DIAG  1000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_CH2_TIMEOUT     60        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH2_ERRCNTMAX   5          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_TRQCMD_MAX    500        //Command @ RCRCV_CH2_TD_MAX (Max = 1000)
#define RCRCV_TRQCMD_ZERO   0           //Command @ RCRCV_CH2_TD_ZERO +- RCRCV_CH2_TD_DEADBAND
#define RCRCV_TRQCMD_MIN    -500        //Command @ RCRCV_CH2_TD_MIN

//CH1 RC Steering
#define RCRCV_CH1_TD_MIN  1100           //Min Duty-Time in micros
#define RCRCV_CH1_TD_ZERO 1500          //Middle/Zero/RC-Off Duty-Time in micros
#define RCRCV_CH1_TD_MAX  1900          //Max Duty-Time in micros
#define RCRCV_CH1_TD_DEADBAND 50        //Deadband Duty-Time around TD_ZERO in micros
#define RCRCV_CH1_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH1_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH1_TD_GRD_DIAG  1000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_CH1_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH1_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_STRCMD_MAX    1         //Command @ RCRCV_CH1_TD_MAX (z.B. 500)
#define RCRCV_STRCMD_ZERO   0           //Command @ RCRCV_CH1_TD_ZERO +- RCRCV_CH1_TD_DEADBAND
#define RCRCV_STRCMD_MIN    -1        //Command @ RCRCV_CH1_TD_MIN (z.B. - 500)

//CH3 RC Pushbutton (Emergency off)
#define RCRCV_CH3_TD_OFF 1290           //Push Button Off Duty-Time in micros
#define RCRCV_CH3_TD_ON 1680             //Push Button On Duty-Time in micros
//#define RCRCV_CH3_TD_RCOFF 870         //RC turned Off Duty-Time in micros
#define RCRCV_CH3_TD_TOLERANCE 100        //Tolerance for each State Duty-Time in micros
#define RCRCV_CH3_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH3_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH3_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH3_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_EMERGOFFCNT_RELAIS  5           //after this number of cycles in emergency off the relais are opened (some time needed to reduce DC current first)

//CH4 RC 3-Way-Switch
#define RCRCV_CH4_TD_LEFT 1260           //3-Way-Switch left Duty-Time in micros
#define RCRCV_CH4_TD_MID 1510             //3-Way-Switch middle Duty-Time in micros
#define RCRCV_CH4_TD_RIGHT 1752            //3-Way-Switch right Duty-Time in micros
#define RCRCV_CH4_TD_RCOFF 870         //RC turned Off Duty-Time in micros
#define RCRCV_CH4_TD_TOLERANCE 100        //Tolerance for each State Duty-Time in micros
#define RCRCV_CH4_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH4_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH4_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH4_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_CTRLMOD_RC        0           //RC-Control-Mode (RC control only)
#define RCRCV_CTRLMOD_RCLIM     1           //RC-Limiting-Mode (RC limits Acclrt-Command: min(RC,Acclrt))
#define RCRCV_CTRLMOD_ACCLRT    2           //Accelerator-Control-Mode (Accelerator-Control only (Speed-Control still via RC))
#define RCRCV_CTRLMOD_SAFE  RCRCV_CTRLMOD_RCLIM           //CTRL_MODE to take for safe state

//CH5 RC Drehknopf  (RcRcv_SpdCmd)
#define RCRCV_CH5_TD_MIN  890           //Min Duty-Time in micros
#define RCRCV_CH5_TD_MAX  2100          //Max Duty-Time in micros
#define RCRCV_CH5_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH5_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH5_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH5_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_CH5_TD_GRD_DIAG  1000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_SPDCMD_MIN    0       //Command @ RCRCV_CH5_TD_MIN
#define RCRCV_SPDCMD_MAX    650    //Command @ RCRCV_CH5_TD_MAX
#define SPDCMD_REVERSE      100     //Speedlimit when driving reverse

//UART
#define UART_TIMEOUT      60      //Timeout for last valid UART Receive in millis

// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
// #define DEBUG_TX

// Global variables
uint8_t idx = 0;         // Index for new data pointer
uint16_t bufStartFrame;  // Buffer Start Frame
byte *p;                 // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

uint16_t t;
uint16_t t100ms;
uint16_t t10ms;

int16_t TrqCmd = 0;
int16_t SpdCmd = 0;

int16_t acclrt_adc = 0;     //raw ADC-Value
int16_t acclrt_adc_old = 0; //ADC-Value last cycle
int16_t acclrt_adc_lastvalid = 0; //last valid ADC-Value
uint8_t acclrt_qlf = 0;     //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t acclrt_errCnt = 0;      //Error cycle counter
int16_t acclrt_TrqCmd = 0;  //TrqCommand derived from acclrt [ACCLRT_TRQCMD_MIN; ACCLRT_TRQCMD_MAX]

//RC Receiver Ch2 Throttle
int16_t RcRcvCh2_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh2_TDuty_old = 0;   //last cycle
int16_t RcRcvCh2_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh2_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh2_errCnt = 0;      //Error cycle counter
int16_t RcRcv_TrqCmd = 0;       //TrqCommand derived from RcRcvCh2_TDuty

uint16_t tMicrosRcRcvCh2Pwm2 = 2;
uint16_t tMicrosRcRcvCh2Pwm1 = 1;
uint16_t tMilisRcRcvCh2Pwm = 0;
uint8_t RcRcvCh2NewData = 0;

//RC Receiver Ch1 Steering
int16_t RcRcvCh1_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh1_TDuty_old = 0;   //last cycle
int16_t RcRcvCh1_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh1_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh1_errCnt = 0;      //Error cycle counter
int16_t RcRcv_StrCmd = 0;      //Steering-Command (Tank-Steering) from RcRcvCh1_TDuty

uint16_t tMicrosRcRcvCh1Pwm2 = 2;
uint16_t tMicrosRcRcvCh1Pwm1 = 1;
uint16_t tMilisRcRcvCh1Pwm = 0;
uint8_t RcRcvCh1NewData = 0;

//RC Receiver Ch5 Drehknopf
int16_t RcRcvCh5_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh5_TDuty_old = 0;   //last cycle
int16_t RcRcvCh5_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh5_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh5_errCnt = 0;      //Error cycle counter
int16_t RcRcv_SpdCmd = 0;       //SpeedCommand derived from RcRcvCh5_TDuty

uint16_t tMicrosRcRcvCh5Pwm2 = 2;
uint16_t tMicrosRcRcvCh5Pwm1 = 1;
uint16_t tMilisRcRcvCh5Pwm = 0;
uint8_t RcRcvCh5NewData = 0;

//RC Receiver Ch4 3-way-switch
int16_t RcRcvCh4_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh4_TDuty_old = 0;   //last cycle
int16_t RcRcvCh4_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh4_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh4_errCnt = 0;      //Error cycle counter
uint8_t RcRcv_CtrlMod = RCRCV_CTRLMOD_SAFE;         //0 = RC-Control; 1 = min(RC, Acclrt); 2 = Acclrt-Control

uint16_t tMicrosRcRcvCh4Pwm2 = 2;
uint16_t tMicrosRcRcvCh4Pwm1 = 1;
uint16_t tMilisRcRcvCh4Pwm = 0;
uint8_t RcRcvCh4NewData = 0;

//RC Receiver Ch3 Pushbutton
int16_t RcRcvCh3_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh3_TDuty_old = 0;   //last cycle
int16_t RcRcvCh3_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh3_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh3_errCnt = 0;      //Error cycle counter
uint8_t RcRcv_EmergOff = 0;       //0 = normal operation; 1 = emergency off
uint8_t RcRcv_EmergOffCnt = 0;    //cycle counter

uint16_t tMicrosRcRcvCh3Pwm2 = 2;
uint16_t tMicrosRcRcvCh3Pwm1 = 1;
uint16_t tMilisRcRcvCh3Pwm = 0;
uint8_t RcRcvCh3NewData = 0;

// Speed
int16_t speedAvg_meas = 0;

//SerialReport
uint8_t SerialReportCounter = 0;

// UART Communication
uint16_t tMillisUART = 0;
uint8_t UART_qlf = 0;   //0 = unplausible; 1 = plausible; 2 = timeout

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t torque;
  int16_t nmax;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;   //*10
  int16_t boardTemp;    //*10
  //uint16_t cmdLed;
  int16_t dc_curr;      //*100
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() {
  //Serials
  Serial.begin(SERIAL_BAUD);  //USB Serial
  Serial.println("Hoverboard Serial v1.0");

  Serial2.begin(BT_SERIAL_BAUD);

  Serial3.begin(HOVER_SERIAL_BAUD);

  // PIN UART
  pinMode(UART_GND_PIN, OUTPUT);
  digitalWrite(UART_GND_PIN, LOW);  //GND-Supply

  //PINS ACCLRT
  pinMode(ACCLRT_SUPPLY_PIN, OUTPUT);
  digitalWrite(ACCLRT_SUPPLY_PIN, HIGH);  //5V-Supply
  pinMode(ACCLRT_GND_PIN, OUTPUT);
  digitalWrite(ACCLRT_GND_PIN, LOW);  //GND-Supply

  //PINS RCRCV
  pinMode(RCRCV_SUPPLY_PIN, OUTPUT);
  digitalWrite(RCRCV_SUPPLY_PIN, HIGH);  //5V-Supply
  pinMode(RCRCV_GND_PIN, OUTPUT);
  digitalWrite(RCRCV_GND_PIN, LOW);  //GND-Supply
  //CH2
  pinMode(RCRCV_CH2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCRCV_CH2_PIN), IsrRcRcvCh2, CHANGE);
  //CH1
  pinMode(RCRCV_CH1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCRCV_CH1_PIN), IsrRcRcvCh1, CHANGE);
  //CH5
  pinMode(RCRCV_CH5_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCRCV_CH5_PIN), IsrRcRcvCh5, CHANGE);
  //CH4
  pinMode(RCRCV_CH4_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCRCV_CH4_PIN), IsrRcRcvCh4, CHANGE);
  //CH3
  pinMode(RCRCV_CH3_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCRCV_CH3_PIN), IsrRcRcvCh3, CHANGE);

  // PINS DC-Relais
  pinMode(DCRELAIS_PIN, OUTPUT);
  digitalWrite(DCRELAIS_PIN, HIGH);   //HIGH = Relais not actuated

  //Initialize
  t = millis();
  t100ms = t;
  t10ms = t;

  acclrt_adc = analogRead(ACCLRT_SENS_PIN);
  acclrt_adc_old = acclrt_adc;
}

void IsrRcRcvCh2(){
  if ((digitalRead(RCRCV_CH2_PIN)) == 0)
  {
    tMicrosRcRcvCh2Pwm2 = (uint16_t)micros();
    tMilisRcRcvCh2Pwm = (uint16_t)millis();
    RcRcvCh2NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh2Pwm1 = (uint16_t)micros();
    RcRcvCh2NewData = 0;
  }    
}

void IsrRcRcvCh1(){
  if ((digitalRead(RCRCV_CH1_PIN)) == 0)
  {
    tMicrosRcRcvCh1Pwm2 = (uint16_t)micros();
    tMilisRcRcvCh1Pwm = (uint16_t)millis();
    RcRcvCh1NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh1Pwm1 = (uint16_t)micros();
    RcRcvCh1NewData = 0;
  }    
}

void IsrRcRcvCh5(){
  if ((digitalRead(RCRCV_CH5_PIN)) == 0)
  {
    tMicrosRcRcvCh5Pwm2 = (uint16_t)micros();
    tMilisRcRcvCh5Pwm = (uint16_t)millis();
    RcRcvCh5NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh5Pwm1 = (uint16_t)micros();
    RcRcvCh5NewData = 0;
  }    
}

void IsrRcRcvCh4(){
  if ((digitalRead(RCRCV_CH4_PIN)) == 0)
  {
    tMicrosRcRcvCh4Pwm2 = (uint16_t)micros();
    tMilisRcRcvCh4Pwm = (uint16_t)millis();
    RcRcvCh4NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh4Pwm1 = (uint16_t)micros();
    RcRcvCh4NewData = 0;
  }    
}

void IsrRcRcvCh3(){
  if ((digitalRead(RCRCV_CH3_PIN)) == 0)
  {
    tMicrosRcRcvCh3Pwm2 = (uint16_t)micros();
    tMilisRcRcvCh3Pwm = (uint16_t)millis();
    RcRcvCh3NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh3Pwm1 = (uint16_t)micros();
    RcRcvCh3NewData = 0;
  }    
}

void SendCommand(int16_t SteerCommand, int16_t TrqCommand, int16_t nmaxCommand) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)SteerCommand;
  Command.torque = (int16_t)TrqCommand;
  Command.nmax = (int16_t)nmaxCommand;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.torque ^ Command.nmax);

  // Write to Serial
  Serial3.write((uint8_t *)&Command, sizeof(Command));

  #ifdef DEBUG_TX
    Serial.write((uint8_t *)&Command, sizeof(Command));
  #endif
}

void SendCommandSafeState(){
  SendCommand(0, 0, 1000);  //Steer + TrqCommand 0 and no nmax-Controll (nmax 1000)
}

void ReceiveUART() {
  // Check for new data availability in the Serial buffer
  if (Serial3.available()) {
    incomingByte = Serial3.read();                                       // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
  } else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print("RX:");
    Serial.println(incomingByte);
    //return;
  #endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.dc_curr);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      tMillisUART = (uint16_t)millis();
      UART_qlf = 1;

      // Print data to built-in Serial
      // Serial.print("cS:");
      // Serial.print(Feedback.cmd1);
      // Serial.print(",cT:");
      // Serial.print(Feedback.cmd2);
      // Serial.print(",sR:");
      // Serial.print(Feedback.speedR_meas);
      // Serial.print(",sL:");
      // Serial.print(Feedback.speedL_meas);
      // Serial.print(",Ub:");
      // Serial.print(Feedback.batVoltage);
      // Serial.print(",Tb:");
      // Serial.print(Feedback.boardTemp);
      // Serial.print(",Idc:");
      // Serial.println(Feedback.dc_curr);
    } else {
      //Serial.println("CRC");
      tMillisUART = (uint16_t)millis();
      UART_qlf = 0;
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void RcRcvCh2ReadPlaus() {

  if (RcRcvCh2NewData == 1)
  {
    RcRcvCh2_TDuty_old = RcRcvCh2_TDuty;
    RcRcvCh2_TDuty = tMicrosRcRcvCh2Pwm2 - tMicrosRcRcvCh2Pwm1;
  }
  
  //Check min/max-grenzen und min/max gradient
  if ((RcRcvCh2_TDuty > RCRCV_CH2_TD_MAX_DIAG) || (RcRcvCh2_TDuty < RCRCV_CH2_TD_MIN_DIAG) || (abs(RcRcvCh2_TDuty - RcRcvCh2_TDuty_old) > RCRCV_CH2_TD_GRD_DIAG))
  {
    
    if (RcRcvCh2_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh2_TDuty_lastvalid = RcRcvCh2_TDuty_old;
      RcRcvCh2_errCnt++;
      RcRcvCh2_TDuty = RcRcvCh2_TDuty_lastvalid;
    }
    else if (RcRcvCh2_errCnt >= RCRCV_CH2_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh2_qlf = 0;
    else
    {
      RcRcvCh2_errCnt++;
      RcRcvCh2_TDuty = RcRcvCh2_TDuty_lastvalid;
    }
    
  }

  else if (((uint16_t)millis() - tMilisRcRcvCh2Pwm) > RCRCV_CH2_TIMEOUT)  //Trigger Timeout -> Set tMicros to init-Value so qlf can only get reset with new valid values
  {
    
    if (RcRcvCh2_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh2_TDuty_lastvalid = RcRcvCh2_TDuty_old;
      RcRcvCh2_errCnt++;
      RcRcvCh2_TDuty = RcRcvCh2_TDuty_lastvalid;
    }
    else if (RcRcvCh2_errCnt >= RCRCV_CH2_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
    {
      RcRcvCh2_qlf = 2;
      tMicrosRcRcvCh2Pwm2 = 2;
      tMicrosRcRcvCh2Pwm1 = 1;
    }
      
    else
    {
      RcRcvCh2_errCnt++;
      RcRcvCh2_TDuty = RcRcvCh2_TDuty_lastvalid;
    }

  }
  else if ((RcRcvCh2_TDuty < (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) && (RcRcvCh2_TDuty > (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)) && (RcRcvCh2_TDuty_old < (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) && (RcRcvCh2_TDuty_old > (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)))  //Re-Enable Plausi-Status only when CH2 is in zero position for 2 cycles
  {
    RcRcvCh2_qlf = 1;
    RcRcvCh2_errCnt = 0;
  }
    
}

void RcRcvCh1ReadPlaus() {
  if (RcRcvCh1NewData == 1)
  {
    RcRcvCh1_TDuty_old = RcRcvCh1_TDuty;
    RcRcvCh1_TDuty = tMicrosRcRcvCh1Pwm2 - tMicrosRcRcvCh1Pwm1;
  }

  //Check min/max-grenzen und min/max gradient
  if ((RcRcvCh1_TDuty > RCRCV_CH1_TD_MAX_DIAG) || (RcRcvCh1_TDuty < RCRCV_CH1_TD_MIN_DIAG) || (abs(RcRcvCh1_TDuty - RcRcvCh1_TDuty_old) > RCRCV_CH1_TD_GRD_DIAG))
  {
    
    if (RcRcvCh1_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh1_TDuty_lastvalid = RcRcvCh1_TDuty_old;
      RcRcvCh1_errCnt++;
      RcRcvCh1_TDuty = RcRcvCh1_TDuty_lastvalid;
    }
    else if (RcRcvCh1_errCnt >= RCRCV_CH1_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh1_qlf = 0;
    else
    {
      RcRcvCh1_errCnt++;
      RcRcvCh1_TDuty = RcRcvCh1_TDuty_lastvalid;
    }
    
  }

  else if (((uint16_t)millis() - tMilisRcRcvCh1Pwm) > RCRCV_CH1_TIMEOUT)  //Trigger Timeout -> Set tMicros to init-Value so qlf can only get reset with new valid values
  {
    
    if (RcRcvCh1_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh1_TDuty_lastvalid = RcRcvCh1_TDuty_old;
      RcRcvCh1_errCnt++;
      RcRcvCh1_TDuty = RcRcvCh1_TDuty_lastvalid;
    }
    else if (RcRcvCh1_errCnt >= RCRCV_CH1_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
    {
      RcRcvCh1_qlf = 2;
      tMicrosRcRcvCh1Pwm2 = 2;
      tMicrosRcRcvCh1Pwm1 = 1;
    }
      
    else
    {
      RcRcvCh1_errCnt++;
      RcRcvCh1_TDuty = RcRcvCh1_TDuty_lastvalid;
    }

  }
  else if ((RcRcvCh1_TDuty < (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) && (RcRcvCh1_TDuty > (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)) && (RcRcvCh1_TDuty_old < (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) && (RcRcvCh1_TDuty_old > (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)))  //Re-Enable Plausi-Status only when CH1 is in zero position for 2 cycles
  {
    RcRcvCh1_qlf = 1;
    RcRcvCh1_errCnt = 0;
  }
}

void RcRcvCh5ReadPlaus() {
  if (RcRcvCh5NewData == 1)
  {
    RcRcvCh5_TDuty_old = RcRcvCh5_TDuty;
    RcRcvCh5_TDuty = tMicrosRcRcvCh5Pwm2 - tMicrosRcRcvCh5Pwm1;
  }

  //Check min/max-grenzen und min/max gradient
  if ((RcRcvCh5_TDuty > RCRCV_CH5_TD_MAX_DIAG) || (RcRcvCh5_TDuty < RCRCV_CH5_TD_MIN_DIAG) || (abs(RcRcvCh5_TDuty - RcRcvCh5_TDuty_old) > RCRCV_CH5_TD_GRD_DIAG))
    
    if (RcRcvCh5_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh5_TDuty_lastvalid = RcRcvCh5_TDuty_old;
      RcRcvCh5_errCnt++;
      RcRcvCh5_TDuty = RcRcvCh5_TDuty_lastvalid;
    }
    else if (RcRcvCh5_errCnt >= RCRCV_CH5_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh5_qlf = 0;
    else
    {
      RcRcvCh5_errCnt++;
      RcRcvCh5_TDuty = RcRcvCh5_TDuty_lastvalid;
    }

  else if (((uint16_t)millis() - tMilisRcRcvCh5Pwm) > RCRCV_CH5_TIMEOUT)  //Trigger Timeout -> Set tMicros to init-Value so qlf can only get reset with new valid values
  {
    
    if (RcRcvCh5_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh5_TDuty_lastvalid = RcRcvCh5_TDuty_old;
      RcRcvCh5_errCnt++;
      RcRcvCh5_TDuty = RcRcvCh5_TDuty_lastvalid;
    }
    else if (RcRcvCh5_errCnt >= RCRCV_CH5_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
    {
      RcRcvCh5_qlf = 2;
      tMicrosRcRcvCh5Pwm2 = 2;
      tMicrosRcRcvCh5Pwm1 = 1;
    }
      
    else
    {
      RcRcvCh5_errCnt++;
      RcRcvCh5_TDuty = RcRcvCh5_TDuty_lastvalid;
    }

  }
  else //Re-Enable Plausi-Status if no error was found
  {
    RcRcvCh5_qlf = 1;
    RcRcvCh5_errCnt = 0;
  }
}

void RcRcvCh4ReadPlaus() {
  if (RcRcvCh4NewData == 1)
  {
    RcRcvCh4_TDuty_old = RcRcvCh4_TDuty;
    RcRcvCh4_TDuty = tMicrosRcRcvCh4Pwm2 - tMicrosRcRcvCh4Pwm1;
  }

  //Check min/max-grenzen und min/max gradient
  if ((RcRcvCh4_TDuty > RCRCV_CH4_TD_MAX_DIAG) || (RcRcvCh4_TDuty < RCRCV_CH4_TD_MIN_DIAG))
  {
    
    if (RcRcvCh4_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh4_TDuty_lastvalid = RcRcvCh4_TDuty_old;
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
    else if (RcRcvCh4_errCnt >= RCRCV_CH4_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh4_qlf = 0;
    else
    {
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
    
  }
  else if (((uint16_t)millis() - tMilisRcRcvCh4Pwm) > RCRCV_CH4_TIMEOUT)  //Trigger Timeout -> Set tMicros to init-Value so qlf can only get reset with new valid values
  {
    if (RcRcvCh4_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh4_TDuty_lastvalid = RcRcvCh4_TDuty_old;
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
    else if (RcRcvCh4_errCnt >= RCRCV_CH4_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
    {
      RcRcvCh4_qlf = 2;
      tMicrosRcRcvCh4Pwm2 = 2;
      tMicrosRcRcvCh4Pwm1 = 1;
    }
      
    else
    {
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
  }
  //check if outside tolerance bands around states
  else if (((RcRcvCh4_TDuty > (RCRCV_CH4_TD_LEFT + RCRCV_CH4_TD_TOLERANCE)) || (RcRcvCh4_TDuty < (RCRCV_CH4_TD_LEFT - RCRCV_CH4_TD_TOLERANCE))) && ((RcRcvCh4_TDuty > (RCRCV_CH4_TD_MID + RCRCV_CH4_TD_TOLERANCE)) || (RcRcvCh4_TDuty < (RCRCV_CH4_TD_MID - RCRCV_CH4_TD_TOLERANCE))) && ((RcRcvCh4_TDuty > (RCRCV_CH4_TD_RIGHT + RCRCV_CH4_TD_TOLERANCE)) || (RcRcvCh4_TDuty < (RCRCV_CH4_TD_RIGHT - RCRCV_CH4_TD_TOLERANCE))))
  {
    
    if (RcRcvCh4_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh4_TDuty_lastvalid = RcRcvCh4_TDuty_old;
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
    else if (RcRcvCh4_errCnt >= RCRCV_CH4_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh4_qlf = 0;
    else
    {
      RcRcvCh4_errCnt++;
      RcRcvCh4_TDuty = RcRcvCh4_TDuty_lastvalid;
    }
    
  }

  else //Re-Enable Plausi-Status if no error was found
  {
    RcRcvCh4_qlf = 1;
    RcRcvCh4_errCnt = 0;
  }
}

void RcRcvCh3ReadPlaus() {
  if (RcRcvCh3NewData == 1)
  {
    RcRcvCh3_TDuty_old = RcRcvCh3_TDuty;
    RcRcvCh3_TDuty = tMicrosRcRcvCh3Pwm2 - tMicrosRcRcvCh3Pwm1;
  }

  //Check min/max-grenzen und min/max gradient
  if ((RcRcvCh3_TDuty > RCRCV_CH3_TD_MAX_DIAG) || (RcRcvCh3_TDuty < RCRCV_CH3_TD_MIN_DIAG))
  {
    
    if (RcRcvCh3_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh3_TDuty_lastvalid = RcRcvCh3_TDuty_old;
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }
    else if (RcRcvCh3_errCnt >= RCRCV_CH3_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh3_qlf = 0;
    else
    {
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }
    
  }
  else if (((uint16_t)millis() - tMilisRcRcvCh3Pwm) > RCRCV_CH3_TIMEOUT)  //Trigger Timeout -> Set tMicros to init-Value so qlf can only get reset with new valid values
  {
    
    if (RcRcvCh3_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh3_TDuty_lastvalid = RcRcvCh3_TDuty_old;
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }
    else if (RcRcvCh3_errCnt >= RCRCV_CH3_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
    {
      RcRcvCh3_qlf = 2;
      tMicrosRcRcvCh3Pwm2 = 2;
      tMicrosRcRcvCh3Pwm1 = 1;
    }
      
    else
    {
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }

  }
  //check if outside tolerance bands around states
  else if (((RcRcvCh3_TDuty > (RCRCV_CH3_TD_ON + RCRCV_CH3_TD_TOLERANCE)) || (RcRcvCh3_TDuty < (RCRCV_CH3_TD_ON - RCRCV_CH3_TD_TOLERANCE))) && ((RcRcvCh3_TDuty > (RCRCV_CH3_TD_OFF + RCRCV_CH3_TD_TOLERANCE)) || (RcRcvCh3_TDuty < (RCRCV_CH3_TD_OFF - RCRCV_CH3_TD_TOLERANCE))))
  {
    
    if (RcRcvCh3_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      RcRcvCh3_TDuty_lastvalid = RcRcvCh3_TDuty_old;
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }
    else if (RcRcvCh3_errCnt >= RCRCV_CH3_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      RcRcvCh3_qlf = 0;
    else
    {
      RcRcvCh3_errCnt++;
      RcRcvCh3_TDuty = RcRcvCh3_TDuty_lastvalid;
    }
    
  }
  else //Re-Enable Plausi-Status if no error was found
  {
    RcRcvCh3_qlf = 1;
    RcRcvCh3_errCnt = 0;
  }
}

void AcclrtReadPlaus() {
  acclrt_adc_old = acclrt_adc;
  acclrt_adc = analogRead(ACCLRT_SENS_PIN);

  //Check min/max-grenzen und min/max gradient
  if ((acclrt_adc > ACCLRT_ADC_MAX_DIAG) || (acclrt_adc < ACCLRT_ADC_MIN_DIAG) || (abs(acclrt_adc - acclrt_adc_old) > ACCLRT_ADC_GRD_DIAG))
  {
    
    if (acclrt_errCnt==0) //Error just occurred -> Set lastvalid to old value
    {
      acclrt_adc_lastvalid = acclrt_adc_old;
      acclrt_errCnt++;
      acclrt_adc = acclrt_adc_lastvalid;
    }
    else if (acclrt_errCnt >= ACCLRT_ERRCNTMAX)  //Set Qlf invalid if ERRCNTMAX is reached
      acclrt_qlf = 0;
    else
    {
      acclrt_errCnt++;
      acclrt_adc = acclrt_adc_lastvalid;
    }
    
  }
  else if ((acclrt_adc < ACCLRT_ADC_MIN) && (acclrt_adc_old < ACCLRT_ADC_MIN))  //Re-Enable Plausi-Status only when acclrt not betätigt for 2 cycles
  {
    acclrt_qlf = 1;
    acclrt_errCnt = 0;
  }
    

  //TODO: Evtl. Feedback über Unplausibel-Status über Piepen oder LED
}

void AcclrtTrqCmd() {
  if (acclrt_qlf == 0) {
    acclrt_TrqCmd = ACCLRT_TRQCMD_MIN;
  } else {
    int16_t acclrt_lim = max(min(acclrt_adc, ACCLRT_ADC_MAX), ACCLRT_ADC_MIN);  //limit acclrt ADC Value to Min/Max-Values

    acclrt_TrqCmd = (int16_t)((((int32_t)acclrt_lim - ACCLRT_ADC_MIN) * (ACCLRT_TRQCMD_MAX - ACCLRT_TRQCMD_MIN)) / (ACCLRT_ADC_MAX - ACCLRT_ADC_MIN) + ACCLRT_TRQCMD_MIN);
  }
}

void RcRcvTrqCmd() {
  if (RcRcvCh2_qlf == 0) {
    RcRcv_TrqCmd = RCRCV_TRQCMD_ZERO;
  } else {
    //TD inside Deadband
    if ((RcRcvCh2_TDuty >= (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)) && (RcRcvCh2_TDuty <= (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)))
        RcRcv_TrqCmd = RCRCV_TRQCMD_ZERO;
    //TD on possitive TrqDemand
    else if (RcRcvCh2_TDuty > (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND))
    {
         int16_t RcRcvCh2_TDuty_lim = max(min(RcRcvCh2_TDuty, RCRCV_CH2_TD_MAX), (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND));  //limit to Min/Max-Values
         
         RcRcv_TrqCmd = (int16_t)((((int32_t)RcRcvCh2_TDuty_lim - (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) * (RCRCV_TRQCMD_MAX - RCRCV_TRQCMD_ZERO)) / (RCRCV_CH2_TD_MAX - (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) + RCRCV_TRQCMD_ZERO);
    }
    //TD on negative TrqDemand
    else if (RcRcvCh2_TDuty < (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND))
    {
        int16_t RcRcvCh2_TDuty_lim = max(min(RcRcvCh2_TDuty, (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)), RCRCV_CH2_TD_MIN);  //limit to Min/Max-Values

        RcRcv_TrqCmd = (int16_t)((((int32_t)RcRcvCh2_TDuty_lim - (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)) * (RCRCV_TRQCMD_ZERO - RCRCV_TRQCMD_MIN)) / ((RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND) - RCRCV_CH2_TD_MIN) + RCRCV_TRQCMD_ZERO);
    }
    else 
        RcRcv_TrqCmd = RCRCV_TRQCMD_ZERO; 
  }
}

void RcRcvStrCmd() {
  if (RcRcvCh1_qlf == 0) {
    RcRcv_StrCmd = RCRCV_STRCMD_ZERO;
  } else {
    //TD inside Deadband
    if ((RcRcvCh1_TDuty >= (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)) && (RcRcvCh1_TDuty <= (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)))
        RcRcv_StrCmd = RCRCV_TRQCMD_ZERO;
    //TD on possitive TrqDemand
    else if (RcRcvCh1_TDuty > (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND))
    {
         int16_t RcRcvCh1_TDuty_lim = max(min(RcRcvCh1_TDuty, RCRCV_CH1_TD_MAX), (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND));  //limit to Min/Max-Values
         
         RcRcv_StrCmd = (int16_t)((((int32_t)RcRcvCh1_TDuty_lim - (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) * (RCRCV_STRCMD_MAX - RCRCV_STRCMD_ZERO)) / (RCRCV_CH1_TD_MAX - (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) + RCRCV_STRCMD_ZERO);
    }
    //TD on negative TrqDemand
    else if (RcRcvCh1_TDuty < (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND))
    {
        int16_t RcRcvCh1_TDuty_lim = max(min(RcRcvCh1_TDuty, (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)), RCRCV_CH1_TD_MIN);  //limit to Min/Max-Values

        RcRcv_StrCmd = (int16_t)((((int32_t)RcRcvCh1_TDuty_lim - (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)) * (RCRCV_STRCMD_ZERO - RCRCV_STRCMD_MIN)) / ((RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND) - RCRCV_CH1_TD_MIN) + RCRCV_STRCMD_ZERO);
    }
    else 
        RcRcv_StrCmd = RCRCV_STRCMD_ZERO; 
  }
}

void RcRcvSpdCmd() {
  if (RcRcvCh5_qlf == 0) {
    RcRcv_SpdCmd = RCRCV_SPDCMD_MIN;
  } else {
    int16_t RcRcvCh5_TDuty_lim = max(min(RcRcvCh5_TDuty, RCRCV_CH5_TD_MAX), RCRCV_CH5_TD_MIN);  //limit to Min/Max-Values

    RcRcv_SpdCmd = (int16_t)((((int32_t)RcRcvCh5_TDuty_lim - RCRCV_CH5_TD_MIN) * (RCRCV_SPDCMD_MAX - RCRCV_SPDCMD_MIN)) / (RCRCV_CH5_TD_MAX - RCRCV_CH5_TD_MIN) + RCRCV_SPDCMD_MIN);
  }
}

void RcRcvCtrlMod() {
  if (RcRcvCh4_qlf == 0) {
    RcRcv_CtrlMod = RCRCV_CTRLMOD_SAFE;   //Safe-State is RC-Lim-Mode 
  } else {
    //Switch in left position -> RC-Control
    if ((RcRcvCh4_TDuty >= (RCRCV_CH4_TD_LEFT - RCRCV_CH4_TD_TOLERANCE)) && (RcRcvCh4_TDuty <= (RCRCV_CH4_TD_LEFT + RCRCV_CH4_TD_TOLERANCE)))
        RcRcv_CtrlMod = RCRCV_CTRLMOD_RC;
    //Switch on mid position -> RC-Lim-Mode
    else if ((RcRcvCh4_TDuty >= (RCRCV_CH4_TD_MID - RCRCV_CH4_TD_TOLERANCE)) && (RcRcvCh4_TDuty <= (RCRCV_CH4_TD_MID + RCRCV_CH4_TD_TOLERANCE)))
        RcRcv_CtrlMod = RCRCV_CTRLMOD_RCLIM;
    //Switch on right position -> Acclrt-Mode
    else if ((RcRcvCh4_TDuty >= (RCRCV_CH4_TD_RIGHT - RCRCV_CH4_TD_TOLERANCE)) && (RcRcvCh4_TDuty <= (RCRCV_CH4_TD_RIGHT + RCRCV_CH4_TD_TOLERANCE)))
        RcRcv_CtrlMod = RCRCV_CTRLMOD_ACCLRT;
    else 
        RcRcv_CtrlMod = RCRCV_CTRLMOD_SAFE; 
  }
}

void RcRcvEmergOff() {
  if (RcRcvCh3_qlf == 0) {
    RcRcv_EmergOff = 1;   //Emergency Off when Qlf not valid
    if (RcRcv_EmergOffCnt < 255)
      RcRcv_EmergOffCnt++;  
  } else {
    //Pushbutton off -> Emergency Off
    if ((RcRcvCh3_TDuty >= (RCRCV_CH3_TD_OFF - RCRCV_CH3_TD_TOLERANCE)) && (RcRcvCh3_TDuty <= (RCRCV_CH3_TD_OFF + RCRCV_CH3_TD_TOLERANCE)))
    {
      RcRcv_EmergOff = 1;
      if (RcRcv_EmergOffCnt < 255)
        RcRcv_EmergOffCnt++;      
    }
    //Pushbutton on -> normal operation
    else if ((RcRcvCh3_TDuty >= (RCRCV_CH3_TD_ON - RCRCV_CH3_TD_TOLERANCE)) && (RcRcvCh3_TDuty <= (RCRCV_CH3_TD_ON + RCRCV_CH3_TD_TOLERANCE)))
    {
      RcRcv_EmergOff = 0;
      RcRcv_EmergOffCnt = 0;
    }
    else 
    {
      RcRcv_EmergOff = 1;
      if (RcRcv_EmergOffCnt < 255)
        RcRcv_EmergOffCnt++;  
    }
  }
}

void ReceiveUARTPlaus() {
  if (((uint16_t)millis() - tMillisUART) > UART_TIMEOUT)  //Trigger Timeout
  {
    tMillisUART = (uint16_t)millis() - UART_TIMEOUT - 1;  //pull tMillisUART behind actual millis to avoid overflow/runover-effects

    UART_qlf = 2;

    //Set UART Feedback to 0
    speedAvg_meas = 0;
    Feedback.cmd1 = 0;
    Feedback.cmd2 = 0;
    Feedback.speedR_meas = 0;
    Feedback.speedL_meas = 0;
    Feedback.batVoltage = 0;
    Feedback.boardTemp = 0;
    Feedback.dc_curr = 0;
  }
  else if (UART_qlf == 0) //invalid CRC
  {
    //Set UART Feedback to 0
    speedAvg_meas = 0;
    Feedback.cmd1 = 0;
    Feedback.cmd2 = 0;
    Feedback.speedR_meas = 0;
    Feedback.speedL_meas = 0;
    Feedback.batVoltage = 0;
    Feedback.boardTemp = 0;
    Feedback.dc_curr = 0;
  }
  else
    speedAvg_meas = (Feedback.speedL_meas - Feedback.speedR_meas)/2;
}

void SerialReport(){
  if (Serial2.availableForWrite() >= 10)
  {
    switch(SerialReportCounter) {
      case 0: Serial2.print("a "); Serial2.println(acclrt_qlf); SerialReportCounter++;  break;
      case 1: Serial2.print("b "); Serial2.println(RcRcvCh1_qlf); SerialReportCounter++;  break;
      case 2: Serial2.print("c "); Serial2.println(RcRcvCh2_qlf); SerialReportCounter++;  break;
      case 3: Serial2.print("d "); Serial2.println(RcRcvCh3_qlf); SerialReportCounter++;  break;
      case 4: Serial2.print("e "); Serial2.println(RcRcvCh4_qlf); SerialReportCounter++;  break;
      case 5: Serial2.print("f "); Serial2.println(RcRcvCh5_qlf); SerialReportCounter++;  break;
      case 6: Serial2.print("g "); Serial2.println(UART_qlf); SerialReportCounter++;  break;
      case 7: Serial2.print("h "); Serial2.println(Feedback.batVoltage); SerialReportCounter++;  break;
      case 8: Serial2.print("i "); Serial2.println(Feedback.dc_curr); SerialReportCounter++;  break;
      case 9: Serial2.print("j "); Serial2.println(Feedback.boardTemp); SerialReportCounter++;  break;
      case 10: Serial2.print("k "); Serial2.println(speedAvg_meas); SerialReportCounter++;  break;
      case 11: Serial2.print("l "); Serial2.println(RcRcv_SpdCmd); SerialReportCounter++;  break;
      case 12: Serial2.print("m "); Serial2.println(RcRcv_StrCmd); SerialReportCounter++;  break;
      case 13: Serial2.print("n "); Serial2.println(RcRcv_CtrlMod); SerialReportCounter++;  break;
      case 14: Serial2.print("o "); Serial2.println(acclrt_TrqCmd); SerialReportCounter++;  break;
      case 15: Serial2.print("p "); Serial2.println(RcRcv_TrqCmd); SerialReportCounter++;  break;
      case 16: Serial2.print("q "); Serial2.println(TrqCmd); SerialReportCounter++;  break;
      case 17: Serial2.print("r "); Serial2.println(SpdCmd); SerialReportCounter++;  break;
      default: SerialReportCounter = 0; break;
    }
  }
}

void Task10ms() {
  AcclrtReadPlaus();
  // Serial.print("adc:");  Serial.print(acclrt_adc);
  // Serial.print(",acQlf:");  Serial.print(acclrt_qlf);
  // Serial.print(",acEC:");  Serial.print(acclrt_errCnt);

  RcRcvCh2ReadPlaus();
  // Serial.print(",TRc2:");  Serial.print(RcRcvCh2_TDuty);
  // Serial.print(",Rc2Qlf:");  Serial.print(RcRcvCh2_qlf);
  // Serial.print(",Rc2EC:");  Serial.print(RcRcvCh2_errCnt);

  RcRcvCh5ReadPlaus();
  //Serial.print(",TRc5:");  Serial.print(RcRcvCh5_TDuty);
  //Serial.print(",Rc5Qlf:");  Serial.print(RcRcvCh5_qlf);

  RcRcvCh4ReadPlaus();
  // Serial.print(",TRc4:");  Serial.print(RcRcvCh4_TDuty);
  // Serial.print(",Rc4Qlf:");  Serial.print(RcRcvCh4_qlf);

  RcRcvCh3ReadPlaus();
  // Serial.print(",TRc3:");  Serial.print(RcRcvCh3_TDuty);
  // Serial.print(",Rc3Qlf:");  Serial.print(RcRcvCh3_qlf);

  RcRcvCh1ReadPlaus();
  //Serial.print(",TRc1:");  Serial.print(RcRcvCh1_TDuty);
  //Serial.print(",Rc1Qlf:");  Serial.println(RcRcvCh1_qlf);

  ReceiveUARTPlaus();
  // Serial.print(",Savg:");  Serial.print(speedAvg_meas);

  //Check if all QLF ok
  if  (
      (RcRcvCh1_qlf == 1) 
      && (RcRcvCh2_qlf == 1) 
      && (RcRcvCh3_qlf == 1) 
      && (RcRcvCh4_qlf == 1) 
      && (RcRcvCh5_qlf == 1) 
      && (acclrt_qlf == 1)
      )
  {
    RcRcvEmergOff();
    // Serial.print(",EO:");  Serial.print(RcRcv_EmergOff);
    // Serial.print(",EOC:");  Serial.print(RcRcv_EmergOffCnt);

    //Emergency Off procedure
    if (RcRcv_EmergOff == 1)
    {
      SendCommandSafeState();
      if (RcRcv_EmergOffCnt > RCRCV_EMERGOFFCNT_RELAIS)
        digitalWrite(DCRELAIS_PIN, HIGH);  //Turn Off/Open DC-Relais
    }
    //No Emergency Off
    else 
    {
      digitalWrite(DCRELAIS_PIN, LOW);  //Turn On/Close DC-Relais

      RcRcvSpdCmd();
      // Serial.print(",SC:");  Serial.print(RcRcv_SpdCmd);

      RcRcvStrCmd();
      // Serial.print(",StC:");  Serial.print(RcRcv_StrCmd);

      RcRcvCtrlMod();
      // Serial.print(",CM:");  Serial.print(RcRcv_CtrlMod);

      AcclrtTrqCmd();
      //Serial.print(",TA:");  Serial.print(acclrt_TrqCmd);

      RcRcvTrqCmd();
      // Serial.print(",TR:");  Serial.println(RcRcv_TrqCmd);

      TrqCmd = 0;
      //TrqRequest according to CtrlMod
      if (RcRcv_CtrlMod == RCRCV_CTRLMOD_RC)
        TrqCmd = RcRcv_TrqCmd;
      else if (RcRcv_CtrlMod == RCRCV_CTRLMOD_ACCLRT)
        TrqCmd = acclrt_TrqCmd;
      else if (RcRcv_CtrlMod == RCRCV_CTRLMOD_RCLIM)
        TrqCmd = min(acclrt_TrqCmd, RcRcv_TrqCmd);

      // Serial.print(",TC:");  Serial.println(TrqCmd);
      
      // Speedcommand
      SpdCmd = 0;
      // reverse speedlim
      if (speedAvg_meas < -(SPDCMD_REVERSE - 20))
        SpdCmd = SPDCMD_REVERSE;
      else
        SpdCmd = RcRcv_SpdCmd;

      SendCommand(RcRcv_StrCmd, TrqCmd, SpdCmd);
    }
  }
  //at least one QLF not OK -> SAFE STATE and report QLF
  else
  {
    SendCommandSafeState();

    // Serial.print("R1:");  Serial.print(RcRcvCh1_qlf);
    // Serial.print(",R2:");  Serial.print(RcRcvCh2_qlf);
    // Serial.print(",R3:");  Serial.print(RcRcvCh3_qlf);
    // Serial.print(",R4:");  Serial.print(RcRcvCh4_qlf);
    // Serial.print(",R5:");  Serial.print(RcRcvCh5_qlf);
    // Serial.print(",A:");  Serial.println(acclrt_qlf);
  }

  //Send Heartbeat to communicate task was running
  Serial2.println("Z");
}

void loop(void) {

  if ((uint16_t)millis() != t)  //1 ms task
  {
    t = (uint16_t)millis();
    //Task1ms();
  } else if ((t - t10ms) >= 10)  //10 ms task
  {
    t10ms = t;
    Task10ms();
  } else if ((t - t100ms) >= 100)  //100 ms task
  {
    t100ms = t;
    //Task100ms();
  }

  SerialReport();

  // Check for new received data from hoverboard uart -> Must stay in main-loop
  ReceiveUART();
}
