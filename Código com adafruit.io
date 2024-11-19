#include <SimpleDHT.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Definição dos parâmetros do Wi-Fi
#define WLAN_SSID       "inserir seu login"
#define WLAN_PASS       "inserir sua senha"

// Definição dos parâmetros da Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "inserir seu login adafrui.io"
#define AIO_KEY         "inserir sua chave do adafruit.io"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Configura os canais de publicação MQTT
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish Distance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Distance");

// Feed de assinatura para controle do LED
Adafruit_MQTT_Subscribe LEDControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl");
Adafruit_MQTT_Subscribe LEDControl1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl1");

int pinDHT11 = 0;
SimpleDHT11 dht11(pinDHT11);
byte hum = 0;
byte temp = 0;

// Definição dos pinos do sensor ultrassônico
#define TRIG_PIN 4
#define ECHO_PIN 5


// Pino do LED (LED interno da placa ESP8266)
#define LED_PIN 2 
#define LED_PIN1 1

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED desligado inicialmente
  digitalWrite(LED_PIN1, LOW);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(F("\nWiFi connected"));
  Serial.println(WiFi.localIP());

  mqtt.subscribe(&LEDControl);  // Inscreve-se no feed de controle do LED
  mqtt.subscribe(&LEDControl1);
  connect();
}

void connect() {
  Serial.print(F("Connecting to Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    mqtt.disconnect();
    Serial.println(F("Retrying connection..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Connected!"));
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2;
  return distance;
}

// Função de callback para o controle do LED
void handleLEDControl(Adafruit_MQTT_Subscribe *subscription) {
  if (subscription == &LEDControl) {
    String value = (char *)LEDControl.lastread;  // Corrigido para 'lastread'
    if (value == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ligado via Adafruit IO");
    } else if (value == "OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED desligado via Adafruit IO");
    }
  }
}
void handleLEDControl1(Adafruit_MQTT_Subscribe *subscription) {
  if (subscription == &LEDControl1) {
    String value = (char *)LEDControl1.lastread;  // Corrigido para 'lastread'
    if (value == "ON") {
      digitalWrite(LED_PIN1, HIGH);
      Serial.println("LED ligado via Adafruit IO");
    } else if (value == "OFF") {
      digitalWrite(LED_PIN1, LOW);
      Serial.println("LED desligado via Adafruit IO");
    }
  }
}


void loop() {
  if (!mqtt.ping(3)) {
    if (!mqtt.connected())
      connect();
  }
 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(3000))) {
    if (subscription == &LEDControl) {
      handleLEDControl(subscription);
    }
    if (subscription == &LEDControl1) {
      handleLEDControl(subscription);
    }
  }

  dht11.read(&temp, &hum, NULL);
  Serial.print((int)temp); Serial.print(" *C, ");
  Serial.print((int)hum); Serial.print(" %H, ");
 
  float distance = getDistance();
  Serial.print(distance); Serial.println(" cm");

  if (!Temperature.publish(temp)) {
    Serial.println(F("Failed to send temperature"));
  }
  if (!Humidity.publish(hum)) {
    Serial.println(F("Failed to send humidity"));
  }
  if (!Distance.publish(distance)) {
    Serial.println(F("Failed to send distance"));
  } else {
    Serial.println(F("Data sent!"));
  }

  delay(5000);
}
