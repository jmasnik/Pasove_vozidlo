#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <main.h>

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

#define PIN_SERVO 13

Servo servo;

char mac_address[100];

// AsyncWebServer on port 80
AsyncWebServer server(80);

typedef struct espnow_message_pasak {
  uint8_t type;
  int16_t motor_left;
  int16_t motor_right;
  uint8_t led;
  uint8_t servo;
} espnow_message_pasak;

espnow_message_pasak message_pasak;

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
uint8_t control_servo;

long act_servo;

IPAddress controller_ip;
uint8_t controller_ip_have;

unsigned long back_link_last_ms;
unsigned long serial_out_last_ms;

uint16_t tx_cnt;

void initWiFi();
void initPin();
void initServer();
void initESPNow();

void controlToAction();
void readMacAddress();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);


ESP32PWM pwm;

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);

  controller_ip_have = 0;
  back_link_last_ms = 0;
  serial_out_last_ms = 0;

  tx_cnt = 0;

  control_led = 0;
  control_left = 0;
  control_right = 0;
  control_servo = 50;

  ESP32PWM::allocateTimer(0);

  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO);

  initPin();
  initWiFi();
  //initServer();
  initESPNow();

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

  // Servo (aktualne rozsah 50 - 110 deg)
  act_servo = map(control_servo, 0, 100, 50, 110);
  servo.write(act_servo);

}

/**
 * Hlavni
 */
void loop() {
    //uint8_t i;
    //char *ch_s;
    //char *ch_pt1;
    //char *ch_pt2;
    //char str[20];
    char buff[100];
    unsigned long ms;

    ms = millis();

    /*
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
          for(i = 0; i < 5; i++){
            ch_pt1 = strchr(ch_s, '|');
            ch_pt2 = strchr(ch_pt1 + 1, '|');
            memcpy(str, ch_pt1 + 1, ch_pt2 - ch_pt1 - 1);
            str[ch_pt2 - ch_pt1 - 1] = '\0';
            ch_s = ch_pt2;

            if(i == 0) control_left = -atoi(str);
            if(i == 1) control_right = -atoi(str);
            if(i == 2) control_led = atoi(str);
            if(i == 4) control_servo = atoi(str);
          }
        }

        Serial.printf("UDP packet contents: %s\n", incomingPacket);
      }
      
    } 
    */

    // posilani dat z vozidla pres UDP (ted nic, nema IP toho co to ridi)
    if(ms - back_link_last_ms > 1000 && controller_ip_have == 1){
      tx_cnt++;
      sprintf(buff, "Ahoj z vozidla (%u)!", tx_cnt);

      Udp.beginPacket(controller_ip, 8889);
      Udp.print(buff);
      Udp.endPacket();

      back_link_last_ms = ms;
    }

    // pravidelny vypis stavu na seriak
    if(ms - serial_out_last_ms > 500){
      sprintf(buff, "CLeft: %d CRight: %d CLED: %u CServo: %u AServo: %ld", control_left, control_right, control_led, control_servo, act_servo);
      Serial.println(buff);
      
      serial_out_last_ms = ms;
    }

    // nastaveni motoru podle ovladacu
    controlToAction();
}

/**
 * Init serveru
 */
void initServer() {
  
  // Index
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", "Pasove_vozidlo");
  });

  // Start server
  server.begin();

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
  /*
  WiFi.softAP("Pasove_vozidlo", "");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);
  return;
  */

  // STA
  WiFi.mode(WIFI_STA);
  
  // Prenastavime kanal
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);


  /*
  WiFi.begin("mController", "heslo123");
  Serial.print("Connecting to WiFi mController");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200);
  }
  Serial.println();

  // IP
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Gateway IP
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  */

  // MAC
  readMacAddress();
}

/**
 * Nacte MAC adresu
 */
void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret;
  char buff[100];

  strcpy(mac_address, "");
  
  ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    sprintf(mac_address, "%02x %02x %02x %02x %02x %02x", baseMac[0], baseMac[1], baseMac[2],baseMac[3], baseMac[4], baseMac[5]);
    sprintf(buff, "MAC address: %s", mac_address);
    Serial.println(buff);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

/**
 * Inicializace ESPNow
 */
void initESPNow(){
  // inicializace
  if (esp_now_init() != ESP_OK) {
    log_d("Error initializing ESP-NOW");
    return;
  }
  
  // registrace prijimaciho callbacku
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

/**
 * Prijeti ESPNow
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //char buff[200];

  if(len == sizeof(message_pasak) && incomingData[0] == 0x02){
    memcpy(&message_pasak, incomingData, sizeof(message_pasak));

    //sprintf(buff, "ESPNow / Left: %d  Right: %d  Servo: %u  LED: %u", message_pasak.motor_left, message_pasak.motor_right, message_pasak.servo, message_pasak.led);
    //Serial.println(buff);

    control_left = message_pasak.motor_left;
    control_right = message_pasak.motor_right;
    control_led = message_pasak.led;
    control_servo = message_pasak.servo;

  } else {
    Serial.println("Received ESPNow");
  }

}
