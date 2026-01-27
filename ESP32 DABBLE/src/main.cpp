#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h> 

#define ESP_TX 17  
#define ESP_RX 16  
#define SERIAL_BAUD 9600  

void setup() {
  Serial.begin(115200);      
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, ESP_RX, ESP_TX); 

  Dabble.begin("Rover_ESP32");
  Serial.println("Dabble ESP32 ready!");
}

void loop() {
  Dabble.processInput(); 
  float yValue = GamePad.getYaxisData();  

  int speed = map((int)yValue, -7, 7, -1000, 1000);
  
  
  if (abs(yValue) < 1) {  
    Serial2.println("STOP");
    Serial.println("Sent: STOP");
  } else {
    Serial2.println(speed);
    Serial.print("Sent Speed: ");
    Serial.println(speed);
  }
  
  // Handle button presses
  if (GamePad.isTrianglePressed()) {
    Serial2.println("TRIANGLE");
  }
  if (GamePad.isCirclePressed()) {
    Serial2.println("CIRCLE");
  }
  if (GamePad.isSquarePressed()) {
    Serial2.println("SQUARE");
  }
  if (GamePad.isCrossPressed()) {
    Serial2.println("CROSS");
  }

  delay(50); 
}
