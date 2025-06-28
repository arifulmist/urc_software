#include<WiFi.h>
#include<HTTPClient.h>
# define  led 2
const char* sid="hack_me";
const char*pass="arafatragib";
const char* server="http://192.168.31.72:5000/led";
// IPAddress local_IP(192, 168, 1, 189);   // Set the static IP
// IPAddress gateway(192, 168, 1, 1);      // Set the gateway IP
// IPAddress subnet(255, 255, 255, 0);     // Set subnet mask

void setup() {
  Serial.begin(115200); 
  pinMode(led,OUTPUT);
 // WiFi.config(local_IP,gateway,subnet);   //for static ip     
  WiFi.begin(sid,pass);      // Start Wi-Fi connection
  Serial.println("Connecting wifi.hi....");
  while(WiFi.status()!=WL_CONNECTED)
  {
    delay(400);
    Serial.print(".");
  }
   Serial.println();
  Serial.println("✅ Connected");

  Serial.print("📶 IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
         if(WiFi.status()==WL_CONNECTED)
         {
              HTTPClient hp;//Creates a new HTTP client.
              hp.begin(server);//Connects to your Flask server (/led endpoint).
             // hp.addHeader ("Content-Type","application/json");//Sets the Content-Type to application/json
               int hpcode=hp.GET();
            
            //int hpres=hp.POST(payload);//Sends the POST request with the JSON data.return 200  if ok
            if(hpcode>0)
            {
              String response=hp.getString();
              Serial.println(" server Response: "+response);
            
              if(response.indexOf("ON")>0)
              {
                digitalWrite(led,HIGH);
                Serial.println("Led Is on ....");
              }
              else 
              {
                digitalWrite(led,LOW);
                Serial.println("Led Is off");

              }
            }
            else
            {
                  Serial.println("Error in HTTP request");
            }
             hp.end();
          }
          delay(5000);

}
