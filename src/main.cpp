#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include <UbidotsEsp32Mqtt.h>

#include "DHT.h"
#include <TFT_eSPI.h>
#include <SPI.h>

#include "data.h"
#include "Settings.h"

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

#define DHTPIN 27
#define DHTTYPE DHT11

#define LEDPin1 26
#define LEDPin2 25

DHT dht(DHTPIN, DHTTYPE);
TFT_eSPI tft = TFT_eSPI();

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

const char *UBIDOTS_TOKEN = "BBFF-qzKGCY0knHsGaELFUHuM0D16zyOLB6"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "TESP32";                               // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL1 = "temp";                              // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "hmdd";                              // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL3 = "SW1";
const char *VARIABLE_LABEL4 = "SW2";
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
unsigned long timer;

int bsw1 = 0;
int bsw2 = 0;
char str1[] = "/v2.0/devices/tesp32/sw1/lv";
Ubidots ubidots(UBIDOTS_TOKEN);

// Auxiliar Functions
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);

    if ((char)payload[0] == '1')
    {
      // digitalWrite(LED, HIGH);
      if (strcmp(str1, topic) == 0)
      {
        bsw1 = 1;
      }
      else
      {
        bsw2 = 1;
      }
    }
    else
    {
      // digitalWrite(LED, LOW);
      if (strcmp(str1, topic) == 0)
      {
        bsw1 = 0;
      }
      else
      {
        bsw2 = 0;
      }
    }
  }

  Serial.println();
}

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("Archive", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots

  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  timer = millis();

  pinMode(LEDPin1, OUTPUT);
  pinMode(LEDPin2, OUTPUT);

  Serial.println(F("DHTxx test!"));
  dht.begin();

  tft.init();
  tft.fillScreen(0x0000);

  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL3);
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL4);
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    delay(5000);
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL3);
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL4);
    }
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
      ubidots.publish(DEVICE_LABEL);

      ubidots.add(VARIABLE_LABEL2, h); // Insert your variable Labels and the value to be sent
      ubidots.publish(DEVICE_LABEL);
      timer = millis();
    }
    ubidots.loop();

    tft.drawString("Temp. (°C):", 0, 30, 4);
    tft.drawString(String(t), 0, 60, 7);

    tft.drawString("HMDD (%):", 0, 130, 4);
    tft.drawString(String(h), 0, 160, 7);

    Serial.print(F("% Humedad: "));
    Serial.println(h);
    Serial.print(F("Temperatura: "));
    Serial.print(t);
    Serial.println(F("°C "));
    Serial.println();

    // Switch 1
    if (bsw1 == 1)
    {
      tft.fillCircle(10, 10, 10, TFT_RED); //(X,Y,radio,color)
      digitalWrite(LEDPin1, HIGH);
    }
    else
    {
      tft.fillCircle(10, 10, 10, TFT_DARKGREY); //(X,Y,radio,color)
      digitalWrite(LEDPin1, LOW);
    }

    // Switch 2
    if (bsw2 == 1)
    {
      tft.fillCircle(120, 10, 10, TFT_GREEN); //(X,Y,radio,color)
      digitalWrite(LEDPin2, HIGH);
    }
    else
    {
      tft.fillCircle(120, 10, 10, TFT_DARKGREY); //(X,Y,radio,color)
      digitalWrite(LEDPin2, LOW);
    }
  }

  else // rutina para AP + WebServer
  {
    server.handleClient();

    delay(10);
    detect_long_press();
  }
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}