#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>

// https://gist.github.com/santolucito/4016405f54850f7a216e9e453fe81803

#define PIN_LED 2

#define PIN_PWM1 32
#define PIN_DIR1_A 33
#define PIN_DIR1_B 25

#define PIN_PWM2 14
#define PIN_DIR2_A 26
#define PIN_DIR2_B 27

#define PWM_RESOLUTION 8
#define PWM_FREQ 100
#define PIN_PWM_CH1 14
#define PIN_PWM_CH2 15

// AsyncWebServer on port 80
//AsyncWebServer server(80);

// Websocket
//AsyncWebSocket ws("/ws");

// UDP
WiFiUDP Udp;

unsigned int localUdpPort = 4210;
char incomingPacket[255];

// nastaveni ovladacu
uint8_t control_led;
int16_t control_left;
int16_t control_right;

IPAddress controller_ip;
uint8_t controller_ip_have;

unsigned long back_link_last_ms;

void initWiFi();
void initPin();
void initServer();
void controlToAction();

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);

  controller_ip_have = 0;
  back_link_last_ms = 0;

  control_led = 0;
  control_left = 0;
  control_right = 0;

  initPin();
  initWiFi();
  initServer();

  ledcWrite(PIN_PWM_CH1, 0);
  ledcWrite(PIN_PWM_CH2, 0);
  digitalWrite(PIN_DIR1_A, HIGH);
  digitalWrite(PIN_DIR1_B, HIGH);
  digitalWrite(PIN_DIR2_A, HIGH);
  digitalWrite(PIN_DIR2_B, HIGH);
}

/**
 * Ovladace na real akci
 */
void controlToAction(){

  // LEDka
  if(control_led == 1){
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  // Levej pas
  if(control_left == 0){
    digitalWrite(PIN_DIR1_A, HIGH);
    digitalWrite(PIN_DIR1_B, HIGH);
    ledcWrite(PIN_PWM_CH1, 0);
  }
  if(control_left > 0){
    digitalWrite(PIN_DIR1_A, HIGH);
    digitalWrite(PIN_DIR1_B, LOW);
    ledcWrite(PIN_PWM_CH1, control_left);
  }
  if(control_left < 0){
    digitalWrite(PIN_DIR1_A, LOW);
    digitalWrite(PIN_DIR1_B, HIGH);
    ledcWrite(PIN_PWM_CH1, -control_left);
  }

  // Pravej pas
  if(control_right == 0){
    digitalWrite(PIN_DIR2_A, HIGH);
    digitalWrite(PIN_DIR2_B, HIGH);    
    ledcWrite(PIN_PWM_CH2, 0);
  }
  if(control_right > 0){
    digitalWrite(PIN_DIR2_A, HIGH);
    digitalWrite(PIN_DIR2_B, LOW);
    ledcWrite(PIN_PWM_CH2, control_right);
  }
  if(control_right < 0){
    digitalWrite(PIN_DIR2_A, LOW);
    digitalWrite(PIN_DIR2_B, HIGH);
    ledcWrite(PIN_PWM_CH2, -control_right);
  }  

}

/**
 * Hlavni
 */
void loop() {
    uint8_t i;
    char *ch_s;
    char *ch_pt1;
    char *ch_pt2;
    char str[20];
    char buff[100];
    unsigned long ms;

    ms = millis();

    int packetSize = Udp.parsePacket();

    if (packetSize){
      // receive incoming UDP packets
      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());

      controller_ip = Udp.remoteIP();
      controller_ip_have = 1;

      int len = Udp.read(incomingPacket, 255);
      if (len > 0){
        incomingPacket[len] = '\0';

        if(incomingPacket[0] == 'D'){
          ch_s = incomingPacket;
          for(i = 0; i < 3; i++){
            ch_pt1 = strchr(ch_s, '|');
            ch_pt2 = strchr(ch_pt1 + 1, '|');
            memcpy(str, ch_pt1 + 1, ch_pt2 - ch_pt1 - 1);
            str[ch_pt2 - ch_pt1 - 1] = '\0';
            //Serial.println(str);
            ch_s = ch_pt2;

            if(i == 0) control_left = -atoi(str);
            if(i == 1) control_right = -atoi(str);
            if(i == 2) control_led = atoi(str);
          }
        }

        Serial.printf("UDP packet contents: %s\n", incomingPacket);
        
        sprintf(buff, "Left: %d Right: %d LED: %u", control_left, control_right, control_led);
        Serial.println(buff);
      }
      
    } 

    if(ms - back_link_last_ms > 1000 && controller_ip_have == 1){
      Udp.beginPacket(controller_ip, 8889);
      Udp.print("Ahoj z vozidla!");
      Udp.endPacket();
      back_link_last_ms = ms;
    }

    controlToAction();

  /*
  uint8_t pwm;

  digitalWrite(PIN_LED, HIGH);
  for(pwm = 0; pwm < 255; pwm++){
    ledcWrite(PIN_PWM_CH1, pwm);
    delay(60);
  }

  digitalWrite(PIN_LED, LOW);
  for(pwm = 255; pwm > 0; pwm--){
    ledcWrite(PIN_PWM_CH1, pwm);
    delay(60);
  }
  */
}

/**
 * Init serveru
 */
void initServer() {
  
  // Index
  //server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  //  request->send_P(200, "text/html", "Ahoj");
  //});

  // Start server
  //server.begin();

  // UDP
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}

/**
 * Init pinu
 */
void initPin() {
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_DIR1_A, OUTPUT);
  pinMode(PIN_DIR1_B, OUTPUT);
  pinMode(PIN_DIR2_A, OUTPUT);
  pinMode(PIN_DIR2_B, OUTPUT);

  // konfigurace PWM
  ledcSetup(PIN_PWM_CH1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PIN_PWM_CH2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_PWM1, PIN_PWM_CH1);
  ledcAttachPin(PIN_PWM2, PIN_PWM_CH2);

  ledcWrite(PIN_PWM_CH1, 0);
  ledcWrite(PIN_PWM_CH2, 0);
}

/**
 * Poreseni wifi
 */
void initWiFi() {
  
  // AP
  WiFi.softAP("Pasove_vozidlo", "");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);
  return;

  // STA
  /*
  WiFi.mode(WIFI_STA);
  WiFi.begin("", "");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(250);
  }
  Serial.println();

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  */
}