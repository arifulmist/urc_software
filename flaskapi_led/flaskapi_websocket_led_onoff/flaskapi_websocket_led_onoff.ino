#include<WiFi.h>
#include<WebSocketsClient.h>
#define led 2
const char* sid="hack_me";
const char* pass="arafatragib";
WebSocketsClient web;
void webSocketEvent(WStype_t type, uint8_t*payload,size_t len)
{
  if(type==WStype_TEXT)
  {
    String msg=String ((char*)payload);
    Serial.println("Received from server "+msg);
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led,OUTPUT);
  WiFi.begin(sid,pass);
  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
   Serial.print("\n connected to wifi ");

   web.begin("192.168.31.72",5000,"/");
   web.onEvent(webSocketEvent);
   web.setReconnectInterval(5000);


}

void loop() {
  // put your main code here, to run repeatedly:
  web.loop();
  int val=random(1000,2000);
  String payload = String(val);       // FIX: store in a variable
  web.sendTXT(payload);         //  send the variable
  Serial.println("\nSent to Server :-> "+ String(val));

  if(val>1500)
  {
    digitalWrite(led,HIGH);
    Serial.print("LEd ON means Arik bhai treat dibe");

  }
  else
  {
    digitalWrite(led,LOW);
    Serial.print("Led OFF means Labiba Treat dibe");
  }
  delay(3000);


}
