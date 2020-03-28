#include <TSM12.h>

#define I2C_EN_PIN 3
#define INT_PIN    4

TSM12 keypad = TSM12(I2C_EN_PIN);
E_TSM12_Output_t outputs[TSM12_CHANNEL_COUNT_MAX];
int lastIntPinState = HIGH;

void setup() {
  Serial.begin(9600);
  Serial.println("TSM12 test with INT pin by polling");

  // initialize the INT pin as an input:
  pinMode(INT_PIN, INPUT);
  // initialize the TSM12
  keypad.begin();
}

void loop() {
  int intPinState = digitalRead(INT_PIN);
  if ((intPinState != lastIntPinState) &&
      (intPinState == LOW)
     ) { // Just got a Falling Edge on INT pin
    keypad.getAllChannelOutputs((uint8_t *)&outputs, sizeof(outputs));
    for (int i = 0; i < TSM12_CHANNEL_COUNT_MAX; i++) {
      Serial.print("Channel ");
      Serial.print(i);
      Serial.print(" Output ");
      switch (outputs[i]) {
        case TSM12_OUTPUT_NONE:
          Serial.println("None");
          break;
        case TSM12_OUTPUT_LOW:
          Serial.println("Low");
          break;
        case TSM12_OUTPUT_MIDDLE:
          Serial.println("Middle");
          break;
        case TSM12_OUTPUT_HIGH:
          Serial.println("High");
          break;
        default:
          Serial.println("Unknown");
          break;
      }
    }
  }
}