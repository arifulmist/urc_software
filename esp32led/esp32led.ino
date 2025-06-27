#include<WiFi.h>
const char* sid="hack_me";
const char*pass="arafatragib";
IPAddress local_IP(192, 168, 1, 189);   // Set the static IP
IPAddress gateway(192, 168, 1, 1);      // Set the gateway IP
IPAddress subnet(255, 255, 255, 0);     // Set subnet mask

void setup() {
  Serial.begin(115200); 
  WiFi.config(local_IP,gateway,subnet);        
  WiFi.begin(sid,pass);      // Start Wi-Fi connection
  Serial.println("Connecting wifi.hi....");
  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
   Serial.println();
  Serial.println("✅ Connected to WiFi!");
  Serial.print("📶 IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
            
}
