#include <Arduino.h>
#include <U8g2lib.h>

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

const int32_t stepSizes [] = { 
  51076063,54113191,57330941,60740013,64351807,68178349,72232447,76527610,81078186,85899345,91007194,96418755
};

volatile int32_t currentStepSize;

const std::string keyNames [] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};



std::string getkey(uint8_t keyArray[7]){
  u_int8_t l = keyArray[0];
  u_int8_t m = keyArray[1];
  u_int8_t h = keyArray[2];
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

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory

    //Read key matrix
    uint8_t keyArray[7];
    for(int i = 0; i < 3; i++){
      setRow(i);
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }

    u8g2.setCursor(2,20);
    u_int8_t keys = keyArray[0]<<2 | keyArray[1] << 1 | keyArray[2];
    for(int k = 0; k < 8; k++){
      if (keyArray[k] == 0){
        currentStepSize = stepSizes[k];
      }
    }

    u8g2.print(keys, HEX); 
    std::string key = getkey(keyArray);
    if (key != "X") {
      u8g2.drawStr(2, 30, key.c_str());
    }
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

  }
}