#include <SimpleDHT.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"



// Definição dos parâmetros do Wi-Fi
#define WLAN_SSID       "test"
#define WLAN_PASS       "12345678"

// Definição dos parâmetros da Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "Narlock"
#define AIO_KEY         "aio_HDfK80YBg0x39YE6JcOc0Cb8bzQP"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Configura os canais de publicação MQTT
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish Distance = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Distance");
Adafruit_MQTT_Publish LEDControl2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/LEDControl2");
Adafruit_MQTT_Publish LDR = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/LDR");

// Feeds de assinatura para controle dos LEDs
Adafruit_MQTT_Subscribe LEDControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl");
Adafruit_MQTT_Subscribe LEDControl3 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl3");
Adafruit_MQTT_Subscribe LEDControl4 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl4");
Adafruit_MQTT_Subscribe LEDControl5 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDControl5");

const unsigned long tempoLimite = 10000; // 10 segundos em milissegundos

int pinDHT11 = 0;
SimpleDHT11 dht11(pinDHT11);
byte hum = 0;
byte temp = 0;

// Definição dos pinos do sensor ultrassônico
#define TRIG_PIN 4
#define ECHO_PIN 5

// Definição dos pinos dos LEDs
#define LED_PIN 2
#define LED_PIN2 14
#define LED_PIN3 12
#define LED_PIN4 13
#define LED_PIN5 15


void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
  pinMode(LED_PIN5, OUTPUT);

  

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(F("\nWiFi connected"));
  Serial.println(WiFi.localIP());

  // Inscreve-se nos feeds de controle dos LEDs
  mqtt.subscribe(&LEDControl);  // Existing subscription
  
  mqtt.subscribe(&LEDControl3);
  mqtt.subscribe(&LEDControl4);
  mqtt.subscribe(&LEDControl5); 

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
  float Newdistance = 15 - distance;
  return Newdistance;
}

  int lerp(int start, int end, int step, int numSteps) {
          return start + (step * (end - start)) / numSteps;
  }

// Função de callback para o controle dos LEDs
void handleLEDControl(Adafruit_MQTT_Subscribe *subscription) {
    if (subscription == &LEDControl) {
         String value = String((char*)LEDControl.lastread); // Converter para String
        if (value.equals("ON")) {

        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED 1 ligado via Adafruit IO");
        delay(3000);

        !LEDControl2.publish("1");
        Serial.println(F("Failed to send LEDControl2 value"));
        digitalWrite(LED_PIN2, HIGH);
        Serial.println("LED 2 ligado via Adafruit IO");
          

        } else if (value == "OFF") {
            digitalWrite(LED_PIN, LOW);
            Serial.println("LED 1 desligado via Adafruit IO");
            delay(3000);

            !LEDControl2.publish("0");
            Serial.println(F("Failed to send LEDControl2 value"));
            digitalWrite(LED_PIN2, LOW);
            Serial.println("LED 2 desligado via Adafruit IO");
  

        }
    } /*else if (subscription == &LEDControl2) { // Check for second LED feed
         String value = String((char*)LEDControl.lastread); // Converter para String
        if (value.equals("ON")) {
            digitalWrite(LED_PIN2, HIGH);
            Serial.println("LED 2 ligado via Adafruit IO");
        } else if (value == "OFF") {
            digitalWrite(LED_PIN2, LOW);
            Serial.println("LED 2 desligado via Adafruit IO");
        }
    } */else if (subscription == &LEDControl3) {
          String value = String((char*)LEDControl3.lastread);
          int brightness = value.toInt();
           // Linear interpolation function
          if (brightness >= 10 && brightness <= 255) {
            !LDR.publish("10");
            // Rampa de subida/descida com tempo de transição (ajuste o valor de stepTime)
            int stepTime = 10; // Tempo em milissegundos para cada passo
            int currentBrightness = analogRead(LED_PIN3);
            int steps = abs(brightness - currentBrightness);
            for (int i = 0; i < steps; i++) {
            analogWrite(LED_PIN3, lerp(currentBrightness, brightness, i, steps));
            delay(stepTime);
            }
          } else{
            digitalWrite(LED_PIN3, LOW);
            Serial.println("LED 3 desligado via Adafruit IO");
            !LDR.publish("0");

          }
    } else if (subscription == &LEDControl4) { // Check for second LED feed
         String value = String((char*)LEDControl4.lastread); // Converter para String
        if (value.equals("ON")) {
            digitalWrite(LED_PIN4, HIGH);
            Serial.println("LED 4 ligado via Adafruit IO");
        } else if (value == "OFF") {
            digitalWrite(LED_PIN4, LOW);
            Serial.println("LED 4 desligado via Adafruit IO");
        }
    } else if (subscription == &LEDControl5) { // Check for second LED feed
         String value = String((char*)LEDControl5.lastread); // Converter para String
        if (value.equals("100")) {
            digitalWrite(LED_PIN5, LOW);
            Serial.println("LED 5 ligado via Adafruit IO");
        } else {
            digitalWrite(LED_PIN5, HIGH);
            Serial.println("LED 5 desligado via Adafruit IO");
        }
    } 
}



void loop() {
  if (!mqtt.ping(3)) {
    if (!mqtt.connected())
      connect();
  }

  static unsigned long tempoInicial = 0;
  static bool condicaoVerificada = false;
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    handleLEDControl(subscription);
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
