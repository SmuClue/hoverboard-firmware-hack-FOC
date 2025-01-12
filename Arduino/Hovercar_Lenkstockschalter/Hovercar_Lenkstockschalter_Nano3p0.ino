
// ########################## DEFINES ##########################
#define PIN_GND 3       //D3 = GND for lenkstockschalter
#define PIN_BLINK_LEFT_LENKSTOCK 4 //D4 = Eingang Blinker links Lenkstock
#define PIN_BLINK_RIGHT_LENKSTOCK 5 //D5 = Eingang Blinker rechts Lenkstock
#define PIN_BLINK_LEFT_REAR_OUT 12 //D12 = Ausgang Blinker links
#define PIN_BLINK_RIGHT_REAR_OUT 11 //D11 = Ausgang Blinker rechts

#define COUNTER_LENKSTOCK_DEBOUNCE_ON 8
#define COUNTER_LENKSTOCK_DEBOUNCE_OFF 1

#define COUNTER_BLINKER_ON 10
#define COUNTER_BLINKER_OFF 10

uint16_t t = 0;
uint16_t t10ms = 0;
uint16_t t100ms = 0;

uint8_t CounterBlinkLeftIn = 0; //Counter for debouncing
uint8_t CounterBlinkRightIn = 0; //Counter for debouncing

uint8_t StBlinkerLeftIn = 0;       //debounced status
uint8_t StBlinkerRightIn = 0;       //debounced status

uint8_t CounterBlink = 0;       //Counter for Blinker
uint8_t StBlinker = 0;          //Status Blinker on or off


void setup() {   
    //Lenkstock 
    pinMode(PIN_GND, OUTPUT);
    digitalWrite(PIN_GND,LOW);         
    pinMode(PIN_BLINK_LEFT_LENKSTOCK, INPUT_PULLUP);
    pinMode(PIN_BLINK_RIGHT_LENKSTOCK, INPUT_PULLUP);

    //Blinker
    pinMode(PIN_BLINK_LEFT_REAR_OUT, OUTPUT);
    digitalWrite(PIN_BLINK_LEFT_REAR_OUT,LOW);
    pinMode(PIN_BLINK_RIGHT_REAR_OUT, OUTPUT);
    digitalWrite(PIN_BLINK_RIGHT_REAR_OUT,LOW);

    Serial.begin(115200);
    Serial.println("Hovercar_Lenkstockschalter");
}

void ReadInputs(){
    //Blinker Left
    if (digitalRead(PIN_BLINK_LEFT_LENKSTOCK))
    {
        CounterBlinkLeftIn++;
        if (CounterBlinkLeftIn > COUNTER_LENKSTOCK_DEBOUNCE_ON)
        {
            CounterBlinkLeftIn = COUNTER_LENKSTOCK_DEBOUNCE_ON;
            StBlinkerLeftIn = 1;
        }
    }else{
        CounterBlinkLeftIn--;
        if (CounterBlinkLeftIn < COUNTER_LENKSTOCK_DEBOUNCE_OFF)
        {
            CounterBlinkLeftIn = COUNTER_LENKSTOCK_DEBOUNCE_OFF;
            StBlinkerLeftIn = 0;
        }
    }

    //Blinker Right
    if (digitalRead(PIN_BLINK_RIGHT_LENKSTOCK))
    {
        CounterBlinkRightIn++;
        if (CounterBlinkRightIn > COUNTER_LENKSTOCK_DEBOUNCE_ON)
        {
            CounterBlinkRightIn = COUNTER_LENKSTOCK_DEBOUNCE_ON;
            StBlinkerRightIn = 1;
        }
    }else{
        CounterBlinkRightIn--;
        if (CounterBlinkRightIn < COUNTER_LENKSTOCK_DEBOUNCE_OFF)
        {
            CounterBlinkRightIn = COUNTER_LENKSTOCK_DEBOUNCE_OFF;
            StBlinkerRightIn = 0;
        }
    }
}

void ControlBlinker(){
    if (StBlinkerLeftIn || StBlinkerRightIn)
    {
        CounterBlink++;
        if (CounterBlink < COUNTER_BLINKER_ON)
            StBlinker = 1;
        else if (CounterBlink < (COUNTER_BLINKER_ON+COUNTER_BLINKER_OFF))
            StBlinker = 0;
        else
            CounterBlink = 0;
    }
    else{
        StBlinker = 0;
        CounterBlink = 0;
    }
    //Serial.print("StBlinker: ");Serial.println(StBlinker);
    //Serial.print("DigWrite: ");Serial.println((StBlinkerLeftIn && StBlinker));
    digitalWrite(PIN_BLINK_LEFT_REAR_OUT,(StBlinkerLeftIn && StBlinker));
    digitalWrite(PIN_BLINK_RIGHT_REAR_OUT,(StBlinkerRightIn && StBlinker));
}

void Task1ms(){  
}

void Task10ms(){
    ReadInputs();
    
}

void Task100ms(){
    StBlinkerLeftIn = 1;
    StBlinkerRightIn = 1;
    ControlBlinker();
}

void loop() {
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
  else if ((t - t100ms) >= 100)  //100 ms task
  {
    t100ms = t;
    Task100ms();
  }  
}