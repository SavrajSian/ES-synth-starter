#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

const int32_t stepSizes [] = { 
  51076063,54113191,57330941,60740013,64351807,68178349,72232447,76527610,81078186,85899345,91007194,96418755
};
const int32_t notefreqs[] = {261, 277, 293, 311, 329, 349, 369, 392, 415, 440, 466, 493};
volatile int32_t currentStepSize;
volatile uint8_t keyArray[7];
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t CAN_TX_Semaphore; //to check when can place message in outgoing queue
QueueHandle_t msgInQ; //queue for incoming messages
QueueHandle_t msgOutQ; //queue for outgoing messages
uint8_t RX_Message[8]={0}; //array to store incoming messages

bool send = false; //if true, it sends, if false, it receives

struct knobrotations{
  int8_t knob0;
  int8_t knob1;
  int8_t knob2;
  int8_t knob3;

  knobrotations(){
    knob0 = 0;
    knob1 = 0;
    knob2 = 4;
    knob3 = 0;
  }
};

knobrotations knobrots;

class knob{

  public:
    
  
    knob(){ //if initial state not specified
      state = 0b00;
      prevstate = 0b00;
      rotation = 0;
      upperlimit = 8;
      lowerlimit = 0;
    }

    knob(int8_t kstate, int8_t kprevstate, int8_t krotation, int8_t kupperlimit, int8_t klowerlimit){ //specify initial state
      state = kstate;
      prevstate = kprevstate;
      rotation = krotation;
      upperlimit = kupperlimit;
      lowerlimit = klowerlimit;
    }

    void updaterotation(bool a, bool b, int knobno){  //pass in these values from key matrix, alternatively redo passing in whole keyarray 

      uint8_t state = b << 1 | a;
      int32_t localknob_rotation = rotation;

      /*switch(knobno){ //set local rotation value to global struct value
        case(0):
          localknob_rotation = knobrots.knob0;
        case(1):
          localknob_rotation = knobrots.knob1;
        case(2):
          localknob_rotation = knobrots.knob2;
        case(3):
          localknob_rotation = knobrots.knob3;
      }*/


      if(prevstate == 0b00 && state == 0b01){
        localknob_rotation++;
      }
      else if(prevstate == 0b01 && state == 0b00){
        localknob_rotation--;
      }
      else if(prevstate == 0b10 && state == 0b11){
        localknob_rotation--;
      }
      else if(prevstate == 0b11 && state == 0b10){
        localknob_rotation++;
      }
      //TODO: impossible transition interpretations

      if(localknob_rotation > upperlimit){
        localknob_rotation = upperlimit;
      }
      else if(localknob_rotation < lowerlimit){
        localknob_rotation = lowerlimit;
      }

      prevstate = state;

      /*switch(knobno){ //store local rotation value back into global struct
        case(0):
          __atomic_store_n(&knobrots.knob0, localknob_rotation, __ATOMIC_RELAXED);
        case(1):
          __atomic_store_n(&knobrots.knob1, localknob_rotation, __ATOMIC_RELAXED);
        case(2):
          __atomic_store_n(&knobrots.knob2, localknob_rotation, __ATOMIC_RELAXED);
        case(3):
          __atomic_store_n(&knobrots.knob3, localknob_rotation, __ATOMIC_RELAXED);           
      
      }*/
      __atomic_store_n(&rotation, localknob_rotation, __ATOMIC_RELAXED); 
    }

    void setlimits(int8_t kupperlimit, int8_t klowerlimit){
      upperlimit = kupperlimit;
      lowerlimit = klowerlimit;
    }

    int8_t getrotation(){
      return rotation;
    }

  private: //add knob presses as well, just need to be bool. add joystick too, can define 0=none, 1=west, 2=east, 3 = press. use knob presses to change send/receive mode maybe
    int8_t state;
    int8_t prevstate;
    int8_t rotation;
    int8_t upperlimit;
    int8_t lowerlimit;

};

knob knob3; //volume
knob knob2(0b00, 0b00, 4, 7, 1); //octave
knob knob1;//effects
knob knob0(0b00, 0b00, 0, 10, 1);//effect control

// Define constants for delay effect
const int DELAY_BUFFER_SIZE = 8000;
const float DELAY_MIX = 0.5;
// Create delay buffer
int16_t delayBuffer[DELAY_BUFFER_SIZE] = {0};
int delayBufferIndex = 0;


void sampleISR(){

  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize; // times/divide currentstepsize by 2 to go up/down octave
  
  int32_t Vout = (phaseAcc >> 24) - 128; // offset of 0

  if(knob0.getrotation() == 3){
    //triangle wave
  }
  
  if(knob1.getrotation() == 1){ //simple distortion
    Vout = Vout * Vout >> 4;
  }

  else if(knob1.getrotation() == 2){ //delay
    float delay_feedback = knob0.getrotation() / 18.0; //adjust delay with knob0
    int16_t delayedSample = delayBuffer[delayBufferIndex];
    delayBuffer[delayBufferIndex] = Vout + delayedSample * delay_feedback;
    delayBufferIndex = (delayBufferIndex + 1) % DELAY_BUFFER_SIZE;
    Vout = Vout * (1 - DELAY_MIX) + delayedSample * DELAY_MIX;
  }

  else if(knob1.getrotation() == 3){
    
  }

  Vout = Vout >> (8 - knob3.getrotation()); //volume control

  analogWrite(OUTR_PIN, Vout + 128); //put offset back to median voltage - add offset in final step to make calculations easier
}



u_int8_t readCols() {

  bool a = digitalRead(C0_PIN);
  bool b = digitalRead(C1_PIN);
  bool c = digitalRead(C2_PIN);
  bool d = digitalRead(C3_PIN);

  u_int8_t keys =  d<<3 | c<<2 | b<<1 | a;
  return keys;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(RA0_PIN, rowIdx & 0b001);
  digitalWrite(RA1_PIN, rowIdx & 0b010);
  digitalWrite(RA2_PIN, rowIdx & 0b100);

}

 

void scanKeysTask(void * pvParameters){

  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};

  while(1){ //infinitely run this (RTOS task)

    vTaskDelayUntil( &xLastWakeTime, xFrequency ); //wait for next tick

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    //read key matrix
    for(int i = 0; i < 8; i++){
        setRow(i);
        delayMicroseconds(4);
        keyArray[i] = readCols();
      }
    
    bool pressed = false;
    int32_t localCurrentStepSize = 0;

    u_int16_t keys = keyArray[2]<<8 | keyArray[1] << 4 | keyArray[0]; 

    bool knob3a = keyArray[3] & 0b0001;
    bool knob3b = (keyArray[3] >> 1) & 0b0001;
    bool knob2a = (keyArray[3] >> 2) & 0b0001;
    bool knob2b = (keyArray[3] >> 3) & 0b0001;
    bool knob1a = keyArray[4] & 0b0001;
    bool knob1b = (keyArray[4] >> 1) & 0b0001;
    bool knob0a = (keyArray[4] >> 2) & 0b0001;
    bool knob0b = (keyArray[4] >> 3) & 0b0001;
    //stick other knob var reads in here
    xSemaphoreGive(keyArrayMutex);

    knob3.updaterotation(knob3a, knob3b, 3);
    knob2.updaterotation(knob2a, knob2b, 2);
    knob1.updaterotation(knob1a, knob1b, 1);
    knob0.updaterotation(knob0a, knob0b, 0);
  

    //update currentstepsize
    for(int i=0; i<12; i++){
      bool bit = keys & (1<<i);
      if(bit == 0){
        localCurrentStepSize = stepSizes[i];
        pressed = true;
        int8_t octave = knob2.getrotation();
        TX_Message[0] = 0x50; //note pressed (starts from 0 for C)
        TX_Message[1] = octave; //octave
        TX_Message[2] = i; //note number
        if (octave > 4){
          localCurrentStepSize = localCurrentStepSize << abs(4 - octave);
        }
        else{
          localCurrentStepSize = localCurrentStepSize >> (4 - octave);
        }
        break;
      }
    }
    if(!pressed){
      currentStepSize = 0;
      TX_Message[0] = 0x52; //note released
    }

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED); //comment out to use CAN message instead for playing notes, need other one uncommented obvs
    if(send){ //only send message if set to send mode
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY); //place item in outgoing queue
    }

    //tremolo = sin(2*3.14*0.5*millis())/2;

  }
}

const std::string keyNames [] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

std::string getkey(){
  xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
  u_int8_t l = keyArray[0];
  u_int8_t m = keyArray[1];
  u_int8_t h = keyArray[2];
  xSemaphoreGive(keyArrayMutex);
  u_int32_t keys = h <<8 | m << 4 | l;

  switch (keys){
    case 0b111111111110:
      return keyNames[0];
    case 0b111111111101:
      return keyNames[1];
    case 0b111111111011:
      return keyNames[2];
    case 0b111111110111:
      return keyNames[3];
    case 0b111111101111:
      return keyNames[4];
    case 0b111111011111:
      return keyNames[5];
    case 0b111110111111:
      return keyNames[6];
    case 0b111101111111:
      return keyNames[7];
    case 0b111011111111:
      return keyNames[8];
    case 0b110111111111:
      return keyNames[9];
    case 0b101111111111:
      return keyNames[10];
    case 0b011111111111:
      return keyNames[11];
    default:
      return "X";
  }
}

void displayUpdateTask(void * pvParameters){

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); //wait for next tick

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Keyboard Synth");  // write something to the internal memory

    std::string key = getkey();
    if (key != "X") {
      u8g2.drawStr(2, 20, key.c_str());
    }

    u8g2.setCursor(122,30);
    u8g2.print(knob3.getrotation());
    u8g2.setCursor(85,30);
    u8g2.print(knob2.getrotation());
    u8g2.setCursor(40,30);
    u8g2.print(knob1.getrotation());
    u8g2.setCursor(2,30);
    u8g2.print(knob0.getrotation());

   

    u8g2.setCursor(100,10);
    if(send){
      u8g2.print("send");
    }
    else{
      u8g2.print("recv");
    }

    u8g2.sendBuffer();          // transfer internal memory to the display
    digitalToggle(LED_BUILTIN);
  }

}

void CAN_RX_ISR (void) { //put incoming messages in queue
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
  if(!send){ //only put messages in recv queue if set to recv mode
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  }
}

void decodeTask(void *pvParameters){

  while(1){
    if(!send){ //only decode messages if set to recv mode, might be redundant with cond in CAN_RX_ISR but just in case lol
      xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
      bool press;
      int8_t octave = RX_Message[1];
      if(RX_Message[0] == 0x50){
        press = true;
      }
      else if(RX_Message[0] == 0x52){
        press = false;
      }
      int32_t locCurrentStepSize = 0;
      if(press){
        locCurrentStepSize = stepSizes[RX_Message[2]];
      }
      if (octave > 4){ //adjust step size for octave
        locCurrentStepSize = locCurrentStepSize << abs(4 - octave);
      }
      else{
        locCurrentStepSize = locCurrentStepSize >> (4 - octave);
      }

      //__atomic_store_n(&currentStepSize, locCurrentStepSize, __ATOMIC_RELAXED); //store to global var //comment out if not using TX/RX to play notes, other one in scankeys needs to be uncommented too
      
    }
  }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Initialise CAN bus
  CAN_Init(true); //loopback mode: true means receive own messages, false means receive messages from other keyboards
  setCANFilter(0x123,0x7ff); //only looks for messages with ID 0x123, mask is 0x7ff (every bit of the ID must match the filter for the message to be accepted)
  if(!send){ //only receive if send is false
  CAN_RegisterRX_ISR(CAN_RX_ISR); //call ISR when message received
  }
  if(send){ //only send if send is true
  CAN_RegisterTX_ISR(CAN_TX_ISR); //call ISR when message sent
  }
  CAN_Start();

  //timer to trigger interrupt
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  keyArrayMutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3); //3 slots for outgoing messages, start with 3 slots available. Max count = 3 so a 4th attempt is blocked

  //setup threading for scanning keys
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );  /* Pointer to store the task handle */

  //setup threading for updating display
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );  /* Pointer to store the task handle */

  msgInQ = xQueueCreate(36,8); //create queue for messages from other keyboards
  msgOutQ = xQueueCreate(36,8); //create queue for transmitted messages

  //setup threading for decoding messages
  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  4,			/* Task priority */
  &decodeTaskHandle);  /* Pointer to store the task handle */

  //setup threading for sending messages
  TaskHandle_t CAN_TX_TaskHandle = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "CAN_TX",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &CAN_TX_TaskHandle);  /* Pointer to store the task handle */

  vTaskStartScheduler(); //start RTOS scheduler
}




void loop() {}
