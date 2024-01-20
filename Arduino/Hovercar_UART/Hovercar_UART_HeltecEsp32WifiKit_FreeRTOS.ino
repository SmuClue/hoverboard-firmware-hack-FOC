#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1
#endif

// ########################## includes ##########################
#include "BluetoothSerial.h"
#include "heltec.h"

// ########################## DEFINES ##########################
//HV Battery Voltages
#define UBATHV100P 42.0
#define UBATHV0P 36.0

//LV Battery Voltages
#define UBATLV100P 8.4
#define UBATLV0P 7.2

// UART
#define HOVER_SERIAL_BAUD 115200  // [-] Baud rate for Serial1 (used to communicate with the hoverboard)
#define HOVER_SERIAL_RX_PIN 5      //
#define HOVER_SERIAL_TX_PIN 17      //
#define HOVER_SERIAL_TIMEOUT 100      //Timeout for last valid UART Receive in millis
#define HOVER_SERIAL_NMAX_CMD_OPEN_MODE  9999             //Must match with #define SERIAL_NMAX_CMD_OPEN_MODE in hoverboard config  

#define SERIAL_BAUD 115200        // [-] Baud rate for USB Serial
//#define BT_SERIAL_BAUD 9600        // [-] Baud rate Bluetooth Serial2 (used for SerialReport)
#define START_FRAME 0xABCD        // [-] Start frme definition for reliable serial communication
//#define UART_GND_PIN            //GND for UART on Serial 3 (pins 15, 14) use GND-Port

//Over-All min/max trq command
#define TRQCMD_MAX  1000           //Max-Value in HoverFirmware is 1000
#define TRQCMD_MIN  -900           //Min-Value in HoverFirmware is -1000
#define TRQCMD_BRAKEOFFSET 200    //should match with Hoverboard ELECTRIC_BRAKE_THRES -> TRQ-Command above which actual acceleration torque is set

//Limit starting torque to avoid harsh motor vibrations
#define TRQSTART_ENABLED        //limits starting torque command
#define TRQSTART_TRQ  (500+TRQCMD_BRAKEOFFSET)       //TrqCmd at 0 RPM (Start Torque)
#define TRQSTART_K_RAMP 15      //Slope of TrqRamp over RPM (TrqCmd = TrqStart + speed*K_Ramp/10)

//SpeedLim
#define SPDLIM_ENABLED            //if defined Speedlim function is activated on arduino additionally to the one on hoverboard
#define SPDLIM_K_CTRL 45          //Controller-Stiffnes of SpeedLim (TrqLim = SpeedDif * K_CTRL / 10)
#define SPDLIM_TRQOFFSET TRQCMD_BRAKEOFFSET      //should match with Hoverboard ELECTRIC_BRAKE_THRES
#define SPDCMD_REVERSE      130     //Speedlimit when driving reverse

// Accelerator ADC
#define ACCLRT_SUPPLY_PIN 13     //3.3V Supply for Accelerator
#define ACCLRT_SENS_PIN 12       //Sensor ADC for Accelerator
#define ACCLRT_GND_PIN 14        //GND Supply for Accelerator
#define ACCLRT_ADC_MIN 1000       //MIN-Value of ADC over wich TrqRequest starts
#define ACCLRT_ADC_MIN_DIAG 800  //MIN-Threshold of ADC for Diagnosis
#define ACCLRT_ADC_MAX 2300       //MIN-Value of ADC over wich TrqRequest starts 2800 without spacer, 2300 with 4mm spacer
#define ACCLRT_ADC_MAX_DIAG 3000  //MIN-Threshold of ADC for Diagnosis
#define ACCLRT_ADC_GRD_DIAG 800   //MAX-Absolute change of ADC over 1 Cycle for Diagnosis
#define ACCLRT_ERRCNTMAX   10          //max number of Error-Counter bevore Qlf is set to invalid
#define ACCLRT_TRQCMD_MAX_DEFAULT 600   //Command @ ADC_MAX (Max = 1000)
#define ACCLRT_TRQCMD_MIN 0    //Command @ ADC_MIN

//RC Receiver PINS
//#define RCRCV_SUPPLY_PIN 31      //5V Supply RC-Receiver from 5V-Port
//#define RCRCV_GND_PIN 30         //GND Supply RC-Receiver from GND-Port
#define RCRCV_CH2_PIN 38         //PWM In for RC-Receiver CH2 (Throttle)
#define RCRCV_CH1_PIN 36         //PWM In for RC-Receiver CH1 (Steering)
//#define RCRCV_CH6_PIN         //PWM In for RC-Receiver CH6 (Drehknopf)
#define RCRCV_CH5_PIN 27         //PWM In for RC-Receiver CH5 (Drehknopf)
#define RCRCV_CH4_PIN 32         //PWM In for RC-Receiver CH4 (3-Way-Switch)
#define RCRCV_CH3_PIN 34        //PWM In for RC-Receiver CH4 (3-Way-Switch)

//EncoderSwitch PINS
#define ENCSWITCH_PIN 21           //Digital out controlling Transistor switching Encoder (Hoverboard Motorencoder) GND off/open

//CH2 RC Throttle (RcRcv_TrqCmd)
#define RCRCV_CH2_TD_MIN  1000           //Min Duty-Time in micros 
#define RCRCV_CH2_TD_ZERO 1500          //Middle/Zero/RC-Off Duty-Time in micros
#define RCRCV_CH2_TD_MAX  1950          //Max Duty-Time in micros
#define RCRCV_CH2_TD_DEADBAND 40        //Deadband Duty-Time around TD_ZERO in micros
#define RCRCV_CH2_TD_MAX_DIAG  2300     //Max Duty-Time in micros plausible
#define RCRCV_CH2_TD_MIN_DIAG  700      //Min Duty-Time in micros plausible 
#define RCRCV_CH2_TD_GRD_DIAG  1000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_CH2_TIMEOUT     80        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH2_ERRCNTMAX   8          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_TRQCMD_MAX    TRQCMD_MAX        //Command @ RCRCV_CH2_TD_MAX (Max = 1000)
#define RCRCV_TRQCMD_ZERO   0           //Command @ RCRCV_CH2_TD_ZERO +- RCRCV_CH2_TD_DEADBAND
#define RCRCV_TRQCMD_MIN    TRQCMD_MIN        //Command @ RCRCV_CH2_TD_MIN

//CH1 RC Steering
#define RCRCV_CH1_TD_MIN  1200           //Min Duty-Time in micros 
#define RCRCV_CH1_TD_ZERO 1500          //Middle/Zero/RC-Off Duty-Time in micros
#define RCRCV_CH1_TD_MAX  1800          //Max Duty-Time in micros
#define RCRCV_CH1_TD_DEADBAND 200        //Deadband Duty-Time around TD_ZERO in micros
#define RCRCV_CH1_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG+100     //Max Duty-Time in micros plausible
#define RCRCV_CH1_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG-100      //Min Duty-Time in micros plausible
#define RCRCV_CH1_TD_GRD_DIAG  2000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_CH1_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH1_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_STRCMD_MAX    1         //Command @ RCRCV_CH1_TD_MAX (z.B. 500)
#define RCRCV_STRCMD_ZERO   0           //Command @ RCRCV_CH1_TD_ZERO +- RCRCV_CH1_TD_DEADBAND
#define RCRCV_STRCMD_MIN    -1        //Command @ RCRCV_CH1_TD_MIN (z.B. - 500)
//CH1 acclrt_TrqCmdMax control
#define RCRCV_CH1_TD_TRQMAXDEC 1100   //below this TD acclrt_TrqCmdMax will be decreased
#define RCRCV_CH1_TD_TRQMAXINC 1900   //above this TD acclrt_TrqCmdMax will be increased
#define RCRCV_CH1_TRQMAX_CNT_THRS 10   //Loop-Counter für Entprellung
#define TRQMAX_STEP 100               //acclrt_TrqCmdMax will be in-/decreased by this step
//CH1 RC BT On/Off
#define RCRCV_CH1_TD_BT_OFF 1100      //below this TD BT is turned off
#define RCRCV_CH1_TD_BT_ON  1900      //above this TD BT is turned on
#define BT_CNT_THRS         50        //Loop-Counter für Entprellung BT On/Off Befehl

//CH3 RC Pushbutton (Emergency off)
#define RCRCV_CH3_TD_OFF 1290           //Push Button Off Duty-Time in micros
#define RCRCV_CH3_TD_ON 1670             //Push Button On Duty-Time in micros
//#define RCRCV_CH3_TD_RCOFF 1290         //RC turned Off Duty-Time in micros
#define RCRCV_CH3_TD_TOLERANCE 100        //Tolerance for each State Duty-Time in micros
#define RCRCV_CH3_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH3_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH3_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH3_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_EMERGOFFCNT_RELAIS  5           //after this number of cycles in emergency off the relais are opened (some time needed to reduce DC current first)

//CH4 RC 3-Way-Switch
#define RCRCV_CH4_TD_LEFT 1260           //3-Way-Switch left Duty-Time in micros
#define RCRCV_CH4_TD_MID 1470             //3-Way-Switch middle Duty-Time in micros
#define RCRCV_CH4_TD_RIGHT 1652            //3-Way-Switch right Duty-Time in micros
#define RCRCV_CH4_TD_RCOFF 870         //RC turned Off Duty-Time in micros
#define RCRCV_CH4_TD_TOLERANCE 80        //Tolerance for each State Duty-Time in micros
#define RCRCV_CH4_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH4_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH4_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH4_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_CTRLMOD_RC        0           //RC-Control-Mode (RC control only)
#define RCRCV_CTRLMOD_RCLIM     1           //RC-Limiting-Mode (RC limits Acclrt-Command: min(RC,Acclrt))
#define RCRCV_CTRLMOD_ACCLRT    2           //Accelerator-Control-Mode (Accelerator-Control only (Speed-Control still via RC))
#define RCRCV_CTRLMOD_SAFE  RCRCV_CTRLMOD_RCLIM           //CTRL_MODE to take for safe state

//CH5 RC Drehknopf  (RcRcv_SpdCmd)
#define RCRCV_CH5_TD_MIN  1000           //Min Duty-Time in micros
#define RCRCV_CH5_TD_MAX  1900          //Max Duty-Time in micros
#define RCRCV_CH5_TD_MAX_DIAG  RCRCV_CH2_TD_MAX_DIAG     //Max Duty-Time in micros plausible
#define RCRCV_CH5_TD_MIN_DIAG  RCRCV_CH2_TD_MIN_DIAG      //Min Duty-Time in micros plausible
#define RCRCV_CH5_TIMEOUT     RCRCV_CH2_TIMEOUT        //if last PWM Interrupt is longer ago than this -> Timeout. Time in milis and uint16_t (so max Value is 65535)
#define RCRCV_CH5_ERRCNTMAX   RCRCV_CH2_ERRCNTMAX          //max number of Error-Counter bevore Qlf is set to invalid
#define RCRCV_CH5_TD_GRD_DIAG  1000      //Max Gradient Duty-Time in micros plausible
#define RCRCV_SPDCMD_MIN    50       //Command @ RCRCV_CH5_TD_MIN
#define RCRCV_SPDCMD_MAX    800    //Command @ RCRCV_CH5_TD_MAX
#define SPDCMD_SAFE         300     //NMax-Command @SafeState -> Not 0 because NMax-Controller is very aggressive and defined brake torque shall be commanded via TrqCmd

// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
// #define DEBUG_TX

// Global variables
uint8_t idx = 0;         // Index for new data pointer
uint16_t bufStartFrame;  // Buffer Start Frame
byte *p;                 // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

int16_t TrqCmd = 0;
int16_t SpdCmd = 0;

uint8_t Fahrfreigabe = 0;

uint8_t StatusBtOn = 1;
uint8_t CntBTOnOff = 0;

uint8_t StTrqMaxCmdSet = 0;
uint8_t CntTrqMaxCmd = 0;

uint16_t acclrt_adc_raw[4] = {0,0,0,0};
uint16_t acclrt_adc = 0;     //filtered ADC-Value
uint16_t acclrt_adc_old = 0; //filtered ADC-Value last cycle
uint16_t acclrt_adc_lastvalid = 0; //last valid ADC-Value
uint8_t acclrt_qlf = 0;     //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t acclrt_errCnt = 0;      //Error cycle counter
int16_t acclrt_TrqCmd = 0;  //TrqCommand derived from acclrt [ACCLRT_TRQCMD_MIN; acclrt_TrqCmdMax]
int16_t acclrt_TrqCmdMax = ACCLRT_TRQCMD_MAX_DEFAULT;

//RC Receiver Ch2 Throttle
int16_t RcRcvCh2_TDuty = 0;       //PWM Dutycylce 
int16_t RcRcvCh2_TDuty_old = 0;   //last cycle
int16_t RcRcvCh2_TDuty_lastvalid = 0;   //last valid signal
int16_t RcRcvCh2_qlf = 0;         //0 = unplausible; 1 = plausible; 2 = timeout
uint8_t RcRcvCh2_errCnt = 0;      //Error cycle counter
int16_t RcRcv_TrqCmd = 0;       //TrqCommand derived from RcRcvCh2_TDuty

uint16_t tMicrosRcRcvCh2Pwm2 = 2;
uint16_t tMicrosRcRcvCh2Pwm1 = 1;
uint16_t tMicrosRcRcvCh2Pwm2old = 2;
uint16_t tMicrosRcRcvCh2Pwm1old = 1;
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

uint8_t FlagLastQlfNotOk = 0; //bit0 = RcvCh1Qlf; bit1 = RcvCh2Qlf; bit2 = RcvCh3Qlf; bit3 = RcvCh4Qlf; bit4 = RcvCh5Qlf; bit5 = RcvCh6Qlf; bit6 = Acclrt_Qlf; bit7 = UART_Qlf; 

// Speed
int16_t speedAvg_meas = 0;

uint8_t StTorqueControlRunning = 0;   //Flag to see if TorqeControl() is still running

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
  int16_t pwml;
  int16_t pwmr;
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

BluetoothSerial SerialBT;

void TaskPrioHigh_10ms(void *pvParameters);
void TaskPrioLow_10ms(void *pvParameters);
void TaskPrioLow_1ms(void *pvParameters);
void TaskPrioLow_500ms(void *pvParameters);

UBaseType_t uxHighWaterMark_TaskPrioHigh_10ms;
UBaseType_t uxHighWaterMark_TaskPrioLow_10ms;
UBaseType_t uxHighWaterMark_TaskPrioLow_500ms;
UBaseType_t uxHighWaterMark_TaskPrioLow_1ms;


// ########################## ISRs ##########################
void IRAM_ATTR IsrRcRcvCh2(){
  uint16_t tMicros = (uint16_t)micros();
  if ((digitalRead(RCRCV_CH2_PIN)) == 0)
  {
    tMicrosRcRcvCh2Pwm2 = tMicros;
    tMilisRcRcvCh2Pwm = (uint16_t)millis();
    RcRcvCh2NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh2Pwm1 = tMicros;
  }    
}

void IRAM_ATTR IsrRcRcvCh1(){
  uint16_t tMicros = (uint16_t)micros();
  if ((digitalRead(RCRCV_CH1_PIN)) == 0)
  {
    tMicrosRcRcvCh1Pwm2 = tMicros;
    tMilisRcRcvCh1Pwm = (uint16_t)millis();
    RcRcvCh1NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh1Pwm1 = tMicros;
  }    
}

void IRAM_ATTR IsrRcRcvCh5(){
  uint16_t tMicros = (uint16_t)micros();
  if ((digitalRead(RCRCV_CH5_PIN)) == 0)
  {
    tMicrosRcRcvCh5Pwm2 = tMicros;
    tMilisRcRcvCh5Pwm = (uint16_t)millis();
    RcRcvCh5NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh5Pwm1 = tMicros;
  }    
}

void IRAM_ATTR IsrRcRcvCh4(){
  uint16_t tMicros = (uint16_t)micros();
  if ((digitalRead(RCRCV_CH4_PIN)) == 0)
  {
    tMicrosRcRcvCh4Pwm2 = tMicros;
    tMilisRcRcvCh4Pwm = (uint16_t)millis();
    RcRcvCh4NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh4Pwm1 = tMicros;
  }    
}

void IRAM_ATTR IsrRcRcvCh3(){
  uint16_t tMicros = (uint16_t)micros();
  if ((digitalRead(RCRCV_CH3_PIN)) == 0)
  {
    tMicrosRcRcvCh3Pwm2 = tMicros;
    tMilisRcRcvCh3Pwm = (uint16_t)millis();
    RcRcvCh3NewData = 1;
  }
  else
  {
    tMicrosRcRcvCh3Pwm1 = tMicros;
  }    
}

// void IRAM_ATTR IsrRcRcvCh6(){
//   uint16_t tMicros = (uint16_t)micros();
//   if ((digitalRead(RCRCV_CH6_PIN)) == 0)
//   {
//     tMicrosRcRcvCh6Pwm2 = tMicros;
//     tMilisRcRcvCh6Pwm = (uint16_t)millis();
//     RcRcvCh6NewData = 1;
//   }
//   else
//   {
//     tMicrosRcRcvCh6Pwm1 = tMicros;
//   }    
// }

// ########################## SETUP ##########################
void setup() {
  //Heltec(Display)
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->drawString(0,0,"Setup...");
  Heltec.display->display();
  
  //Serials
  Serial.begin(SERIAL_BAUD);  //USB Serial
  Serial.println("Hoverboard Serial v1.0");  

  SerialBT.begin("HovercarBT");

  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_SERIAL_RX_PIN, HOVER_SERIAL_TX_PIN);

  //PINS ACCLRT
  pinMode(ACCLRT_SENS_PIN,INPUT_PULLDOWN);
  pinMode(ACCLRT_SUPPLY_PIN, OUTPUT);
  digitalWrite(ACCLRT_SUPPLY_PIN, HIGH);  //5V-Supply
  pinMode(ACCLRT_GND_PIN, OUTPUT);
  digitalWrite(ACCLRT_GND_PIN, LOW);  //GND-Supply

  //PINS RCRCV
  //CH2
  pinMode(RCRCV_CH2_PIN, INPUT_PULLDOWN);
  //CH1
  pinMode(RCRCV_CH1_PIN, INPUT_PULLDOWN);
  //CH5
  pinMode(RCRCV_CH5_PIN, INPUT_PULLDOWN);
  //CH4
  pinMode(RCRCV_CH4_PIN, INPUT_PULLDOWN);
  //CH3
  pinMode(RCRCV_CH3_PIN, INPUT_PULLDOWN);

  attatchAllInterrupts();
  
  // PINS EncoderSwitch
  pinMode(ENCSWITCH_PIN, OUTPUT);
  digitalWrite(ENCSWITCH_PIN, LOW);   // LOW = Encoders off

  //Initialize
  acclrt_adc = analogRead(ACCLRT_SENS_PIN);
  acclrt_adc_old = acclrt_adc;


  //Create RTOS-Tasks
  xTaskCreatePinnedToCore(
    TaskPrioHigh_10ms
    ,  "TaskPrioHigh_10ms"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    TaskPrioLow_10ms
    ,  "TaskPrioLow_10ms"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    TaskPrioLow_1ms
    ,  "TaskPrioLow_1ms"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

    xTaskCreatePinnedToCore(
    TaskPrioLow_500ms
    ,  "TaskPrioLow_500ms"   // A name just for humans
    ,  2048  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
}

void SendCommand(int16_t SteerCommand, int16_t TrqCommand, int16_t nmaxCommand) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)SteerCommand;
  Command.torque = (int16_t)TrqCommand;
  Command.nmax = (int16_t)nmaxCommand;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.torque ^ Command.nmax);

  // Write to Serial
  Serial1.write((uint8_t *)&Command, sizeof(Command));

  #ifdef DEBUG_TX
    Serial.write((uint8_t *)&Command, sizeof(Command));
  #endif
}

void SendCommandSafeState(){
  SendCommand(0, 0, SPDCMD_SAFE);  //Steer + TrqCommand 0 + Set NMax-Control to defined value (not 0)
}

void ReceiveUART() {
  // Check for new data availability in the Serial buffer
  if (Serial1.available()) {
    incomingByte = Serial1.read();                                       // Read the incoming byte
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
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.pwml ^ NewFeedback.pwmr ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.dc_curr);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      tMillisUART = (uint16_t)millis();
      UART_qlf = 1;

      // Print data to built-in Serial
      // Serial.print("cS:");
      // Serial.print(Feedback.pwml);
      // Serial.print(",cT:");
      // Serial.print(Feedback.pwmr);
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

void attatchAllInterrupts(){
  attachInterrupt(RCRCV_CH2_PIN, IsrRcRcvCh2, CHANGE);
  attachInterrupt(RCRCV_CH1_PIN, IsrRcRcvCh1, CHANGE);
  attachInterrupt(RCRCV_CH5_PIN, IsrRcRcvCh5, CHANGE);
  attachInterrupt(RCRCV_CH4_PIN, IsrRcRcvCh4, CHANGE);
  attachInterrupt(RCRCV_CH3_PIN, IsrRcRcvCh3, CHANGE);
}

void detatchAllInterrupts(){
  detachInterrupt(RCRCV_CH2_PIN);
  detachInterrupt(RCRCV_CH1_PIN);
  detachInterrupt(RCRCV_CH5_PIN);
  detachInterrupt(RCRCV_CH4_PIN);
  detachInterrupt(RCRCV_CH3_PIN);
}

void RcRcvCh2ReadPlaus() {

  if (RcRcvCh2NewData == 1)
  {
    RcRcvCh2_TDuty_old = RcRcvCh2_TDuty;
    RcRcvCh2_TDuty = tMicrosRcRcvCh2Pwm2 - tMicrosRcRcvCh2Pwm1;

    RcRcvCh2NewData = 0;

    // Serial.print(",TRc2:");  Serial.print(RcRcvCh2_TDuty);
    // Serial.print(",TRc2_D2:");  Serial.print((uint16_t)(tMicrosRcRcvCh2Pwm2 - tMicrosRcRcvCh2Pwm2old));
    // Serial.print(",TRc2_D1:");  Serial.print((uint16_t)(tMicrosRcRcvCh2Pwm1 - tMicrosRcRcvCh2Pwm1old));
    // Serial.print(",T2Rc2:");  Serial.print(tMicrosRcRcvCh2Pwm2);
    // Serial.print(",T1Rc2:");  Serial.print(tMicrosRcRcvCh2Pwm1);

    tMicrosRcRcvCh2Pwm2old = tMicrosRcRcvCh2Pwm2;
    tMicrosRcRcvCh2Pwm1old = tMicrosRcRcvCh2Pwm1;
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
  else //Re-Enable Plausi-Status if no error was found
  {
    RcRcvCh2_qlf = 1;
    if (RcRcvCh2_errCnt > 0)
      RcRcvCh2_errCnt--;
  }
  // Serial.print(",TRc2Lv:");  Serial.print(RcRcvCh2_TDuty_lastvalid);  
}

void RcRcvCh1ReadPlaus() {
  if (RcRcvCh1NewData == 1)
  {
    RcRcvCh1_TDuty_old = RcRcvCh1_TDuty;
    RcRcvCh1_TDuty = tMicrosRcRcvCh1Pwm2 - tMicrosRcRcvCh1Pwm1;
    
    RcRcvCh1NewData = 0;
  }
  // Serial.print(",TRc1:");  Serial.print(RcRcvCh1_TDuty);
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
  else  //Re-Enable Plausi-Status if no error was found
  {
    RcRcvCh1_qlf = 1;
    if (RcRcvCh1_errCnt > 0)
      RcRcvCh1_errCnt--;
  }
}

void RcRcvCh5ReadPlaus() {
  if (RcRcvCh5NewData == 1)
  {
    RcRcvCh5_TDuty_old = RcRcvCh5_TDuty;
    RcRcvCh5_TDuty = tMicrosRcRcvCh5Pwm2 - tMicrosRcRcvCh5Pwm1;
    
    RcRcvCh5NewData = 0;
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
    if (RcRcvCh5_errCnt > 0)
      RcRcvCh5_errCnt--;
  }
}

void RcRcvCh4ReadPlaus() {
  if (RcRcvCh4NewData == 1)
  {
    RcRcvCh4_TDuty_old = RcRcvCh4_TDuty;
    RcRcvCh4_TDuty = tMicrosRcRcvCh4Pwm2 - tMicrosRcRcvCh4Pwm1;
    
    RcRcvCh4NewData = 0;
  }

  //Check min/max-grenzen
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
    if (RcRcvCh4_errCnt > 0)
      RcRcvCh4_errCnt--;
  }
}

void RcRcvCh3ReadPlaus() {
  if (RcRcvCh3NewData == 1)
  {
    RcRcvCh3_TDuty_old = RcRcvCh3_TDuty;
    RcRcvCh3_TDuty = tMicrosRcRcvCh3Pwm2 - tMicrosRcRcvCh3Pwm1;
    
    RcRcvCh3NewData = 0;
  }

  //Check min/max-grenzen
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
    if (RcRcvCh3_errCnt > 0)
      RcRcvCh3_errCnt--;
  }
}

void AcclrtReadPlaus() {
  acclrt_adc_old = acclrt_adc;
  for (uint8_t i = 0; i<3; i++)
  {
    acclrt_adc_raw[i] = acclrt_adc_raw[i+1];
  }
  detatchAllInterrupts();     //Work-Around für ESP32 Bug, der bei ADC-Read/Wifi/BT benutzung Interrupts in Pin 36 u. 39 auslösen kann -> Führt zu plötzlichem Rauschen auf RC-RCV TDuty -> Alle Interrupts deaktivieren während ADC-Read
  acclrt_adc_raw[3] = analogRead(ACCLRT_SENS_PIN);
  attatchAllInterrupts();

  acclrt_adc = (acclrt_adc_raw[0] + acclrt_adc_raw[1] + acclrt_adc_raw[2] + acclrt_adc_raw[3])/4;

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
  else if ((acclrt_adc < ACCLRT_ADC_MIN) && (acclrt_adc_old < ACCLRT_ADC_MIN))  //Re-Enable Plausi-Status only when acclrt not pressed for 2 cycles
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
    uint16_t acclrt_lim = max(min(acclrt_adc, (uint16_t)ACCLRT_ADC_MAX), (uint16_t)ACCLRT_ADC_MIN);  //limit acclrt ADC Value to Min/Max-Values

    acclrt_TrqCmd = (int16_t)((((int32_t)acclrt_lim - ACCLRT_ADC_MIN) * (acclrt_TrqCmdMax - ACCLRT_TRQCMD_MIN)) / (ACCLRT_ADC_MAX - ACCLRT_ADC_MIN) + ACCLRT_TRQCMD_MIN);
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
         int16_t RcRcvCh2_TDuty_lim = max(min(RcRcvCh2_TDuty, (int16_t)RCRCV_CH2_TD_MAX), (int16_t)(RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND));  //limit to Min/Max-Values
         
         RcRcv_TrqCmd = (int16_t)((((int32_t)RcRcvCh2_TDuty_lim - (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) * (RCRCV_TRQCMD_MAX - RCRCV_TRQCMD_ZERO)) / (RCRCV_CH2_TD_MAX - (RCRCV_CH2_TD_ZERO + RCRCV_CH2_TD_DEADBAND)) + RCRCV_TRQCMD_ZERO);
    }
    //TD on negative TrqDemand
    else if (RcRcvCh2_TDuty < (RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND))
    {
        int16_t RcRcvCh2_TDuty_lim = max(min(RcRcvCh2_TDuty, (int16_t)(RCRCV_CH2_TD_ZERO - RCRCV_CH2_TD_DEADBAND)), (int16_t)RCRCV_CH2_TD_MIN);  //limit to Min/Max-Values

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
         int16_t RcRcvCh1_TDuty_lim = max(min(RcRcvCh1_TDuty, (int16_t)RCRCV_CH1_TD_MAX), (int16_t)(RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND));  //limit to Min/Max-Values
         
         RcRcv_StrCmd = (int16_t)((((int32_t)RcRcvCh1_TDuty_lim - (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) * (RCRCV_STRCMD_MAX - RCRCV_STRCMD_ZERO)) / (RCRCV_CH1_TD_MAX - (RCRCV_CH1_TD_ZERO + RCRCV_CH1_TD_DEADBAND)) + RCRCV_STRCMD_ZERO);
    }
    //TD on negative TrqDemand
    else if (RcRcvCh1_TDuty < (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND))
    {
        int16_t RcRcvCh1_TDuty_lim = max(min(RcRcvCh1_TDuty, (int16_t)(RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)), (int16_t)RCRCV_CH1_TD_MIN);  //limit to Min/Max-Values

        RcRcv_StrCmd = (int16_t)((((int32_t)RcRcvCh1_TDuty_lim - (RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND)) * (RCRCV_STRCMD_ZERO - RCRCV_STRCMD_MIN)) / ((RCRCV_CH1_TD_ZERO - RCRCV_CH1_TD_DEADBAND) - RCRCV_CH1_TD_MIN) + RCRCV_STRCMD_ZERO);
    }
    else 
        RcRcv_StrCmd = RCRCV_STRCMD_ZERO; 
  }
}

void RcRcvCh1BtCmd() {
  if (RcRcvCh1_qlf == 1) {
    if (RcRcvCh1_TDuty >= RCRCV_CH1_TD_BT_ON)
    {
      //BT On      
      if (CntBTOnOff >= BT_CNT_THRS)
      {
        SerialBT.begin("HovercarBT");
        StatusBtOn = 1;
      }
      else
        CntBTOnOff++;
      
    }        
    else if (RcRcvCh1_TDuty <= RCRCV_CH1_TD_BT_OFF)
    {
      //BT On      
      if (CntBTOnOff >= BT_CNT_THRS)
      {
        SerialBT.end();
        StatusBtOn = 0;
      }
      else
        CntBTOnOff++;
    }  
    else if (CntBTOnOff > 0)
      CntBTOnOff--;
  }
}

void RcRcvCh1TrqMaxCmd() {
  if (RcRcvCh1_qlf == 1) {
    if (RcRcvCh1_TDuty >= RCRCV_CH1_TD_TRQMAXINC)
    {    
      if ((CntTrqMaxCmd >= RCRCV_CH1_TRQMAX_CNT_THRS) && (StTrqMaxCmdSet == 0))
      {
        acclrt_TrqCmdMax = acclrt_TrqCmdMax + TRQMAX_STEP;
        acclrt_TrqCmdMax = min(acclrt_TrqCmdMax,(int16_t)TRQCMD_MAX);
        StTrqMaxCmdSet = 1;
      }
      else
        CntTrqMaxCmd++;
      
    }        
    else if (RcRcvCh1_TDuty <= RCRCV_CH1_TD_TRQMAXDEC)
    {   
      if ((CntTrqMaxCmd >= RCRCV_CH1_TRQMAX_CNT_THRS) && (StTrqMaxCmdSet == 0))
      {
        acclrt_TrqCmdMax = acclrt_TrqCmdMax - TRQMAX_STEP;
        acclrt_TrqCmdMax = max(acclrt_TrqCmdMax,(int16_t)TRQCMD_BRAKEOFFSET);
        StTrqMaxCmdSet = 1;
      }
      else
        CntTrqMaxCmd++;
    }  
    else if (CntTrqMaxCmd > 0)
      CntTrqMaxCmd--;
    else if (CntTrqMaxCmd == 0)
      StTrqMaxCmdSet = 0;
  }
}

void RcRcvSpdCmd() {
  if (RcRcvCh5_qlf == 0) {
    RcRcv_SpdCmd = RCRCV_SPDCMD_MIN;
  } else {
    int16_t RcRcvCh5_TDuty_lim = max(min(RcRcvCh5_TDuty, (int16_t)RCRCV_CH5_TD_MAX), (int16_t)RCRCV_CH5_TD_MIN);  //limit to Min/Max-Values

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

uint8_t set_bit(uint8_t x, uint8_t offset, bool value){
    return (value)
        ? x | (1 << offset)
        : x & ~(1 << offset);
}

void SetFlagLastQlfNotOk(){
  //bit0 = RcvCh1Qlf; bit1 = RcvCh2Qlf; bit2 = RcvCh3Qlf; bit3 = RcvCh4Qlf; bit4 = RcvCh5Qlf; bit5 = RcvCh6Qlf; bit6 = Acclrt_Qlf; bit7 = UART_Qlf; 
  
  if ((RcRcvCh1_qlf!=1)
    ||(RcRcvCh2_qlf!=1)
    ||(RcRcvCh3_qlf!=1)
    ||(RcRcvCh4_qlf!=1)
    ||(RcRcvCh5_qlf!=1)
    //||(RcRcvCh6_qlf!=1)
    ||(acclrt_qlf!=1)
    ||(UART_qlf!=1))
  {
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 0, (RcRcvCh1_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 1, (RcRcvCh2_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 2, (RcRcvCh3_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 3, (RcRcvCh4_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 4, (RcRcvCh5_qlf!=1));
    //FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 5, (RcRcvCh6_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 6, (acclrt_qlf!=1));
    FlagLastQlfNotOk = set_bit(FlagLastQlfNotOk, 7, (UART_qlf!=1));
  }

}

void ReceiveUARTPlaus() {
  //Serial.print("timeUARTRcvMs: "); Serial.println((uint16_t)millis() - tMillisUART);
  if (((uint16_t)millis() - tMillisUART) > HOVER_SERIAL_TIMEOUT)  //Trigger Timeout
  {
    tMillisUART = (uint16_t)millis() - HOVER_SERIAL_TIMEOUT - 1;  //pull tMillisUART behind actual millis to avoid overflow/runover-effects

    UART_qlf = 2;

    //Set UART Feedback to 0
    speedAvg_meas = 0;
    Feedback.pwml = 0;
    Feedback.pwmr = 0;
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
    Feedback.pwml = 0;
    Feedback.pwmr = 0;
    Feedback.speedR_meas = 0;
    Feedback.speedL_meas = 0;
    Feedback.batVoltage = 0;
    Feedback.boardTemp = 0;
    Feedback.dc_curr = 0;
  }
  else
    speedAvg_meas = (Feedback.speedL_meas - Feedback.speedR_meas)/2;
}

int16_t SpeedLimPos(int16_t speed_max, int16_t speed_meas, uint8_t K_Ctrl)
{

int16_t SpeedDif = speed_max - speed_meas;
int16_t TrqCmdSpeedlim = SPDLIM_TRQOFFSET + SpeedDif * K_Ctrl / (int8_t)10;
TrqCmdSpeedlim = min(TrqCmdSpeedlim,(int16_t)TRQCMD_MAX);
TrqCmdSpeedlim = max(TrqCmdSpeedlim,(int16_t)0);

return TrqCmdSpeedlim;
}

int16_t SpeedLimNeg(int16_t speed_min, int16_t speed_meas, uint8_t K_Ctrl)
{
int16_t SpeedDif = speed_min - speed_meas;
int16_t TrqCmdSpeedlim = -SPDLIM_TRQOFFSET + SpeedDif * K_Ctrl / (int8_t)10;
TrqCmdSpeedlim = min(TrqCmdSpeedlim,(int16_t)0);
TrqCmdSpeedlim = max(TrqCmdSpeedlim,(int16_t)TRQCMD_MIN);

return TrqCmdSpeedlim;
}

int16_t TrqLimStart(int16_t speed_meas, uint8_t K_Ctrl)
{
int16_t TrqCmdSpeedlim = TRQSTART_TRQ + abs(speed_meas) * K_Ctrl / (int8_t)10;
TrqCmdSpeedlim = min(TrqCmdSpeedlim,max((int16_t)TRQCMD_MAX,(int16_t)TRQCMD_MIN));
TrqCmdSpeedlim = max(TrqCmdSpeedlim,(int16_t)TRQSTART_TRQ);

return TrqCmdSpeedlim;
}


void SerialReport(){
  // Serial.print("BTClnt: "); Serial.println(SerialBT.hasClient());
  if ((StatusBtOn) && (SerialBT.hasClient() > 0))
  {
    SerialBT.print("a "); SerialBT.println(acclrt_qlf);
    SerialBT.print("b "); SerialBT.println(RcRcvCh1_qlf);
    SerialBT.print("c "); SerialBT.println(RcRcvCh2_qlf);
    SerialBT.print("d "); SerialBT.println(RcRcvCh3_qlf);
    SerialBT.print("e "); SerialBT.println(RcRcvCh4_qlf);
    SerialBT.print("f "); SerialBT.println(RcRcvCh5_qlf);
    SerialBT.print("g "); SerialBT.println(UART_qlf);
    SerialBT.print("h "); SerialBT.println(Feedback.batVoltage);
    SerialBT.print("i "); SerialBT.println(Feedback.dc_curr);
    SerialBT.print("j "); SerialBT.println(Feedback.boardTemp);
    SerialBT.print("k "); SerialBT.println(speedAvg_meas);
    SerialBT.print("l "); SerialBT.println(RcRcv_SpdCmd);
    SerialBT.print("m "); SerialBT.println(RcRcv_StrCmd);
    SerialBT.print("n "); SerialBT.println(RcRcv_CtrlMod);
    SerialBT.print("o "); SerialBT.println(acclrt_TrqCmd);
    SerialBT.print("p "); SerialBT.println(RcRcv_TrqCmd);
    SerialBT.print("q "); SerialBT.println(TrqCmd);
    SerialBT.print("r "); SerialBT.println(SpdCmd);
    SerialBT.print("s "); SerialBT.println(Fahrfreigabe);
    SerialBT.print("t "); SerialBT.println(Feedback.pwml);
    SerialBT.print("u "); SerialBT.println(Feedback.pwmr);  
    SerialBT.print("v "); SerialBT.println(acclrt_adc);
    SerialBT.print("w "); SerialBT.println(FlagLastQlfNotOk);
    SerialBT.print("x "); SerialBT.println(RcRcvCh2_TDuty);
    SerialBT.print("y "); SerialBT.println(Feedback.speedR_meas);
    SerialBT.print("z "); SerialBT.println(Feedback.speedL_meas);
    SerialBT.print("A "); SerialBT.println((uint8_t)millis());
    SerialBT.print("B "); SerialBT.println(acclrt_TrqCmdMax);

    if (StTorqueControlRunning)
    {
      StTorqueControlRunning = 0;
      SerialBT.println("Z");
    }
  }  
}

void DisplayReport(){
  Heltec.display->clear();

  //Heltec.display->drawString(10, 10, "t = " + String(pdTICKS_TO_MS(xLastWakeTime)%10000) + " ms");

  
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);

  //Line 1: display Qlf Acclrt + UART
  Heltec.display->drawString(0,0,"AcclrtQlf=" + String(acclrt_qlf));  Heltec.display->drawString(64,0,"UARTQlf=" + String(UART_qlf));

  //Line 2: display Qlf RcRcv
  if (RcRcvCh1_qlf != 1)
    Heltec.display->drawString(0,10,"RcCh1Qlf=" + String(RcRcvCh1_qlf));
  else if (RcRcvCh2_qlf != 1)
    Heltec.display->drawString(0,10,"RcCh2Qlf=" + String(RcRcvCh2_qlf));
  else if (RcRcvCh3_qlf != 1)
    Heltec.display->drawString(0,10,"RcCh3Qlf=" + String(RcRcvCh3_qlf));
  else if (RcRcvCh4_qlf != 1)
    Heltec.display->drawString(0,10,"RcCh4Qlf=" + String(RcRcvCh4_qlf));
  else if (RcRcvCh5_qlf != 1)
    Heltec.display->drawString(0,10,"RcCh5Qlf=" + String(RcRcvCh5_qlf));
  else
    Heltec.display->drawString(0,10,"RcCh1-5Qlf=" + String(RcRcvCh5_qlf));
  
  Heltec.display->drawString(64,10,"AcclrtTrq=" + String(min((int16_t)999,acclrt_TrqCmdMax)));

  // Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  // if (StatusBtOn)
  //   Heltec.display->drawString(128,10,"BT");
  // Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);


  //Line 3 -4: display HVbat status
  float UBatPerc = ((float)Feedback.batVoltage/(float)100.0 - (float)UBATHV0P)/((float)UBATHV100P - (float)UBATHV0P)*(float)100.0;
  UBatPerc = max(min((float)100.0, UBatPerc),(float)0.0);
  Heltec.display->drawProgressBar(2, 22, 120, 10, round(UBatPerc));
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  Heltec.display->drawString(64,32,"UBatHV = " + String((float)Feedback.batVoltage/(float)100.0) + " V");

  //Line 5 - 6: display LVbat status
  UBatPerc = ((float)740/(float)100.0 - (float)UBATLV0P)/((float)UBATLV100P - (float)UBATLV0P)*(float)100.0;
  UBatPerc = max(min((float)100.0, UBatPerc),(float)0.0);
  Heltec.display->drawProgressBar(2, 44, 120, 10, round(UBatPerc));
  Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
  Heltec.display->drawString(64,54,"UBatLV = " + String((float)740/(float)100.0) + " V");


  Heltec.display->display();
}

void TorqueControl() {
  AcclrtReadPlaus();
  // Serial.print("adc:");  Serial.println(acclrt_adc);  
  // Serial.print("adc0:");  Serial.println(acclrt_adc_raw[0]);
  // Serial.print("adc1:");  Serial.println(acclrt_adc_raw[1]);
  // Serial.print("adc2:");  Serial.println(acclrt_adc_raw[2]);
  // Serial.print("adc3:");  Serial.println(acclrt_adc_raw[3]);
  // Serial.print(",acQlf:");  Serial.print(acclrt_qlf);
  // Serial.print(",acEC:");  Serial.print(acclrt_errCnt);

  RcRcvCh2ReadPlaus();
  // Serial.print(",TRc2:");  Serial.print(RcRcvCh2_TDuty);
   //Serial.print(",Rc2Qlf:");  Serial.print(RcRcvCh2_qlf);
   //Serial.print(",Rc2EC:");  Serial.println(RcRcvCh2_errCnt);

  RcRcvCh5ReadPlaus();
  //Serial.print(",TRc5:");  Serial.print(RcRcvCh5_TDuty);
  //Serial.print(",Rc5Qlf:");  Serial.print(RcRcvCh5_qlf);

  RcRcvCh4ReadPlaus();
  //Serial.print(",TRc4:");  Serial.print(RcRcvCh4_TDuty);
  // Serial.print(",Rc4Qlf:");  Serial.print(RcRcvCh4_qlf);

  RcRcvCh3ReadPlaus();
  //Serial.print(",TRc3:");  Serial.print(RcRcvCh3_TDuty);
  // Serial.print(",Rc3Qlf:");  Serial.print(RcRcvCh3_qlf);

  RcRcvCh1ReadPlaus();
  // Serial.print(",TRc1:");  Serial.println(RcRcvCh1_TDuty);
  //Serial.print(",Rc1Qlf:");  Serial.println(RcRcvCh1_qlf);

  ReceiveUARTPlaus();
  // Serial.print(",Savg:");  Serial.print(speedAvg_meas);

  //RcRcvCh1BtCmd();
  // Serial.print("StatusBtOn:");  Serial.println(StatusBtOn);
  // Serial.print("BtCnt:");  Serial.println(CntBTOnOff);

  RcRcvCh1TrqMaxCmd();


  //Check if all QLF ok
  if  (
      (RcRcvCh1_qlf == 1) 
      && (RcRcvCh2_qlf == 1) 
      && (RcRcvCh3_qlf == 1) 
      && (RcRcvCh4_qlf == 1) 
      && (RcRcvCh5_qlf == 1) 
      //&& (acclrt_qlf == 1)  //-> acclrt_TrqCmd is set to ACCLRT_TRQCMD_MIN when qlf is not okay inside AcclrtTrqCmd(). Qlf only goes to "ok" when acclrt is not pressed. RC-Control-Mode shall still be available at Acclrt failure.
      )
  {
    RcRcvEmergOff();
    // Serial.print(",EO:");  Serial.print(RcRcv_EmergOff);
    // Serial.print(",EOC:");  Serial.print(RcRcv_EmergOffCnt);

    //Emergency Off procedure
    if (RcRcv_EmergOff == 1)
    {
      Fahrfreigabe = 0;
      SendCommand(0, 0, HOVER_SERIAL_NMAX_CMD_OPEN_MODE);  //Steer + TrqCommand 0 + Set motors into open mode
      if (RcRcv_EmergOffCnt > RCRCV_EMERGOFFCNT_RELAIS)
        digitalWrite(ENCSWITCH_PIN, LOW);  //Turn Off Encoders
    }
    //No Emergency Off
    else 
    {
      digitalWrite(ENCSWITCH_PIN, HIGH);  //Turn On Encoders

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
      int16_t SpdCmdReverse = min((int16_t)SPDCMD_REVERSE,RcRcv_SpdCmd);
      // reverse speedlim
      if (speedAvg_meas < -(SpdCmdReverse - 20))
        SpdCmd = SpdCmdReverse;
      else
        SpdCmd = RcRcv_SpdCmd;
      
      //TrqRequest with StartinTrqLimitation
      #if defined(TRQSTART_ENABLED)
        TrqCmd = min(TrqCmd,TrqLimStart(speedAvg_meas, (uint8_t)TRQSTART_K_RAMP));
        TrqCmd = max(TrqCmd,(int16_t)-TrqLimStart(speedAvg_meas, (uint8_t)TRQSTART_K_RAMP));
      #endif

      //TrqRequest with SpeedLimitation
      #if defined(SPDLIM_ENABLED)
        TrqCmd = min(TrqCmd,SpeedLimPos(RcRcv_SpdCmd, speedAvg_meas, (uint8_t)SPDLIM_K_CTRL));
        TrqCmd = max(TrqCmd,SpeedLimNeg(-SpdCmdReverse, speedAvg_meas, (uint8_t)SPDLIM_K_CTRL));
      #endif
      

      //Fahrfreigabe only when Trq-Request = 0
      if ((UART_qlf==1) && (abs(TrqCmd) < 5))
        Fahrfreigabe = 1;
      else if (UART_qlf!=1)
        Fahrfreigabe = 0;
      
      // Send Trq/Speed/Steer-Command only if Fahrfreigabe==1
      if (Fahrfreigabe == 1)
        SendCommand(RcRcv_StrCmd, TrqCmd, SpdCmd);
      else
        SendCommandSafeState();
    }
  }
  //at least one QLF not OK -> SAFE STATE, Fahrfreigabe = 0 and report QLF
  else
  {
    Fahrfreigabe = 0;
    SendCommandSafeState();

    //if RC Qlf not OK open Relais
    if ((RcRcvCh1_qlf != 1) 
      || (RcRcvCh2_qlf != 1) 
      || (RcRcvCh3_qlf != 1) 
      || (RcRcvCh4_qlf != 1) 
      || (RcRcvCh5_qlf != 1))
    digitalWrite(ENCSWITCH_PIN, LOW);  //Turn Off Encoders
  }

  SetFlagLastQlfNotOk();

  //Send Heartbeat to communicate task was running
  StTorqueControlRunning = 1;
  Serial.print("Z"); Serial.println((uint8_t)millis());
}

void TaskPrioLow_10ms(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  //10 ms Task-Cycle

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark_TaskPrioLow_10ms = uxTaskGetStackHighWaterMark( NULL );

  for( ;; )
  {
      // Wait for the next cycle.
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      // Perform action here.
      SerialReport();

      uxHighWaterMark_TaskPrioLow_10ms = uxTaskGetStackHighWaterMark( NULL );
  }  
}

void TaskPrioLow_500ms(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);  //500 ms Task-Cycle

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark_TaskPrioLow_500ms = uxTaskGetStackHighWaterMark( NULL );

  for( ;; )
  {
      // Wait for the next cycle.
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      // Perform action here.
      //Report Stack HighWaterMark of all tasks (= Stack Bytes that are not in use (higher = better))
      Serial.print("StackHi10ms:"); Serial.println(uxHighWaterMark_TaskPrioHigh_10ms);
      Serial.print("StackLo10ms:"); Serial.println(uxHighWaterMark_TaskPrioLow_10ms);
      Serial.print("StackLo500ms:"); Serial.println(uxHighWaterMark_TaskPrioLow_500ms);
      Serial.print("StackHi1ms:"); Serial.println(uxHighWaterMark_TaskPrioLow_1ms);

      //Display
      DisplayReport();

      uxHighWaterMark_TaskPrioLow_10ms = uxTaskGetStackHighWaterMark( NULL );
  }  
}

void TaskPrioHigh_10ms(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  //10 ms Task-Cycle  
  

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark_TaskPrioHigh_10ms = uxTaskGetStackHighWaterMark( NULL );

  for( ;; )
  {
      // Wait for the next cycle.
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      // Perform action here.
      TorqueControl();

      uxHighWaterMark_TaskPrioHigh_10ms = uxTaskGetStackHighWaterMark( NULL );
  }  
}

void TaskPrioLow_1ms(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1);  //1 ms Task-Cycle  
  

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark_TaskPrioLow_1ms = uxTaskGetStackHighWaterMark( NULL );

  for( ;; )
  {
      // Wait for the next cycle.
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      // Perform action here.
      //ReceiveUART several times to make sure all Bytes are received
      for (uint8_t i = 1; i<4; i++)
      {
        ReceiveUART();
      }
      

      uxHighWaterMark_TaskPrioLow_1ms = uxTaskGetStackHighWaterMark( NULL );
  }  
}

void loop(void) {
  //must be empty
}
