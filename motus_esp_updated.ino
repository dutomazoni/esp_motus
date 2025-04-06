#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Wire.h>

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>

#define FORMAT_LITTLEFS_IF_FAILED true
#define uS_TO_S_FACTOR 1000000ULL  //Fator de conversão de micro segundos para segundos

#include <WiFi.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson

//Import para DS1307
// #include <DS1307.h>
// DS1307 rtc;
#include <DS1307RTC.h>
#include <TimeLib.h>
#include "time.h"

//Import para SHT40
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// Dados Wifi
String ssid = "";
String senha = "";
int wifiPortalTimeout = 120;
String codigo = "";
const char *senhaAccessPoint = "9503572107";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -10800;
const int daylightOffset_sec = 0;

struct Settings {
  char codigo[50];
} sett;

HTTPClient httpClient;
SemaphoreHandle_t httpClientMutex;

// URL ou IP
const char *uriSaveData = "https://fancy-locrian-picture.glitch.me/dataTableNew";
const char *uriAlertWaterPump = "https://fancy-locrian-picture.glitch.me/alert/water-pump-status";
const char *uriChangeWaterPupmStatus = "https://fancy-locrian-picture.glitch.me/alert/water-pump-status?esp_id=123";

const char *firmwareUrl = "https://fancy-locrian-picture.glitch.me/asset-url";
const char *firmwareVersionUrl = "https://fancy-locrian-picture.glitch.me/firmware-version.txt";
String currentFirmwareVersion = "v1.0.2";

unsigned long lastOtaCheck = 0;
const unsigned long otaInterval = 60000;

//Arquivo de referencia RTC
const char *pathRTC = "/RefRTC.txt";

//Pinos Saidas
const int LedBlue = 2;
const int LedYellow = 4;
const int LedRed = 18;
const int LedGreen = 19;
const int ReleBomba = 26;

//Pinos Entradas
const int AutomaticoSwitch = 25;                //*
const int AlternarAutomaticoManualButton = 23;  //*

// Limites NPK
float limitMinNpk1 = 20.0;
float limitMinNpk2 = 20.0;
float limitMaxNpk1 = 23.0;
float limitMaxNpk2 = 23.0;

// D1 é na verdade TX-GPIO1
// D3 é na verdade RX-GPIO3
// Eles são o borne in out (reserva)

//Variáveis RTC DS1307
tmElements_t lastSavedTime;

//Variáveis conexão DESVON
const char *pathConfig = "/configparametros.json";

struct SensorReadings {
  float tempAmb;
  float umidAmb;
  float tempNPK1;
  float tempNPK2;
  float tempLeaf;
  float humNPK1;
  float humNPK2;
  float humLeaf;
  int tentativasDeEnvio;
};

float temperatura;
float umidade;
float molhamento;

QueueHandle_t httpQueue;
QueueHandle_t alertQueue;

bool bombaLigada = false;
bool deveTrocarStatusBomba = true;
bool pararControleAutomaticoBomba = false;
bool pararControleManualBomba = false;
bool estado_controle_bomba = false;
bool shouldTurnOffBomba = false;
bool wifiConectado = false;
int tentativasLerSensorTemperatura = 1;

//Tasks Handle
TaskHandle_t ReadInputsTaskHandle;
TaskHandle_t timeControlTaskHandle;
TaskHandle_t controleAutomaticoBombaTaskHandle;
TaskHandle_t controleManualBombaTaskHandle;
TaskHandle_t sendDataToServerTaskHandle;
TaskHandle_t sendAlertToServerTaskHandle;
TaskHandle_t blinkBlueLedTaskHandle;
TaskHandle_t checkWaterPumpStatusHandle;
bool controleAutomaticoBombaTaskStarted = false;
bool shouldStopBlinkingBlueLed = false;

void ReadInputsTask(void *pvParameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConectado) {
        wifiConectado = true;
        digitalWrite(LedBlue, HIGH);
      }
    } else {
      if (wifiConectado) {
        wifiConectado = false;
        digitalWrite(LedBlue, LOW);
      }
    }
    if (digitalRead(AlternarAutomaticoManualButton) == HIGH) {
      if (deveTrocarStatusBomba) {
        deveTrocarStatusBomba = false;
        if (digitalRead(AutomaticoSwitch) == LOW) {
          controleManualBomba();
        }
      }
    } else {
      deveTrocarStatusBomba = true;
    }
    if (digitalRead(AutomaticoSwitch) == HIGH) {
      if (digitalRead(LedYellow) == HIGH) {
        estado_controle_bomba = true;
        pararControleAutomaticoBomba = false;
        if (!controleAutomaticoBombaTaskStarted) {
          controleAutomaticoBombaTaskStarted = true;
          digitalWrite(ReleBomba, HIGH);
          digitalWrite(LedRed, HIGH);
          xTaskNotifyGive(controleAutomaticoBombaTaskHandle);
        }
      }
    } else {
      controleAutomaticoBombaTaskStarted = false;
      estado_controle_bomba = false;
      pararControleAutomaticoBomba = true;
    }

    vTaskDelay(50 / portTICK_RATE_MS);
  }
}

void getSavedRtcTimestamp() {
  if (!LittleFS.exists(pathRTC)) {
    Serial.println("Referência de tempo não existe, criando");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    writeFileInt(LittleFS, pathRTC, mktime(&timeinfo));
    Serial.print("Referência de tempo criada: ");
    int savedTimestamp = readFileInt(LittleFS, pathRTC);
    Serial.println(savedTimestamp);
    tmElements_t tm;
    breakTime(mktime(&timeinfo), tm);
    RTC.write(tm);
    // rtc.set(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year); //sec, min, hour, day, month, year
  } else {
    Serial.println("Referência de tempo existe, carregando");
    int savedTimestamp = readFileInt(LittleFS, pathRTC);
    Serial.println(savedTimestamp);
    breakTime(savedTimestamp, lastSavedTime);
  }
}

void setupPins() {
  pinMode(LedGreen, OUTPUT);
  pinMode(LedBlue, OUTPUT);
  pinMode(LedYellow, OUTPUT);
  pinMode(LedRed, OUTPUT);
  pinMode(ReleBomba, OUTPUT);

  pinMode(AutomaticoSwitch, INPUT_PULLDOWN);
  pinMode(AlternarAutomaticoManualButton, INPUT_PULLDOWN);

  digitalWrite(LedBlue, LOW);
  digitalWrite(LedYellow, LOW);
  digitalWrite(LedRed, LOW);
  digitalWrite(LedGreen, LOW);

  //Define a resolução das entradas analógicas, quanto maior, mais sensores vão ser compatíveis
  analogReadResolution(16);  // Valores -> 0-65535
}

bool setupWiFi() {
  WiFiManager wm;

  Serial.println("Iniciando configuração do WiFi");
  //Inicialização do WiFiManager - Access Point
  WiFi.mode(WIFI_STA);
  wm.resetSettings();
  wm.setConfigPortalTimeout(wifiPortalTimeout);
  const char *accessPoint = "AccessPoint-MotusBots";
  sett.codigo[49] = '\0';
  WiFiManagerParameter codigo("str", "Código", sett.codigo, 50);
  wm.addParameter(&codigo);

  int wifiConnectMaxTries = 5;
  int wifiConnectTries = 0;

  if (ssid != "") {
    Serial.println("Tentando conectar WiFi");
    WiFi.begin(ssid, senha);
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000 / portTICK_RATE_MS);
      wifiConnectTries += 1;
      if (wifiConnectTries == wifiConnectMaxTries) {
        break;
      }
    }
  }

  Serial.print("Status do WiFi: ");
  Serial.print(WiFi.status());
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    salvaNoJson("nomeDoWifi", wm.getWiFiSSID());
    salvaNoJson("senhaDoWifi", wm.getWiFiPass());
    salvaNoJson("codigo", codigo.getValue());

    Serial.print("Conectado ao WiFi");
    WiFi.softAPdisconnect(true);
    Serial.print(" | Rede: ");
    Serial.print(wm.getWiFiSSID());
    Serial.print(" | IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" | Força do sinal: ");
    Serial.print(WiFi.RSSI());
    Serial.println();
    return true;
  } else {
    if (!wm.startConfigPortal(accessPoint, senhaAccessPoint)) {
      Serial.println("  Falha ao conectar. Excedeu o tempo limite para conexão.");
      return false;
    } else {
      strncpy(sett.codigo, codigo.getValue(), 50);
      sett.codigo[49] = '\0';
      salvaNoJson("codigo", sett.codigo);
      Serial.print("Código do dispositivo: ");
      Serial.println(sett.codigo);
      salvaNoJson("nomeDoWifi", wm.getWiFiSSID());
      salvaNoJson("senhaDoWifi", wm.getWiFiPass());

      File file = LittleFS.open(pathConfig, FILE_READ);
      if (file) {
        Serial.println("Arquivo de configuracao aberto.");
        JsonDocument doc;

        DeserializationError error = deserializeJson(doc, file);
        if (error) {
          Serial.println("Failed to parse file");
        }
        Serial.println("Imprimindo JSON de parâmetros:");
        serializeJson(doc, Serial);
      } else {
        Serial.println("não abriu o arquivo por algum motivo");
      }
      file.close();
      return true;
    }
  }
}

void setupLittleFS() {
  Serial.println("Iniciando o SPIFSS (SPI Flash File System)");
  if (LittleFS.begin()) {
    if (LittleFS.exists(pathConfig)) {
      Serial.println("Abrindo o arquivo de configuracao WiFi");
      File file = LittleFS.open(pathConfig, FILE_READ);
      if (file) {
        Serial.println("Arquivo de configuracao aberto.");
        JsonDocument doc;

        DeserializationError error = deserializeJson(doc, file);
        if (error) {
          Serial.println("Failed to parse file");
        }
        Serial.println("Imprimindo JSON de parâmetros:");
        serializeJson(doc, Serial);
        codigo = doc["codigo"].as<String>();
        ssid = doc["nomeDoWifi"].as<String>();
        senha = doc["senhaDoWifi"].as<String>();
        file.close();
        return;
      } else {
        Serial.println("Falha ao ler as configuracoes do arquivo JSON.");
      }
    } else {
      Serial.println("Criando JSON para salvar parâmetros");
      JsonDocument doc;

      doc["codigo"] = "";
      doc["nomeDoWifi"] = "";
      doc["senhaDoWifi"] = "";
      File file = LittleFS.open(pathConfig, FILE_WRITE, true);
      if (!file) {
        Serial.println("Houve uma falha ao abrir o arquivo de configuração para incluir/alterar as configurações");
        return;
      }
      if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write to file");
      }
      file.close();
    }
  } else {
    Serial.println("  Falha no SPIFSS (SPI Flash File System)!");
    digitalWrite(LedRed, HIGH);
    return;
  }
}

void setup() {
  Serial.begin(115200);
  setupPins();

  Serial.println("=========================INICIANDO SISTEMA MOTUSBOTS=========================");

  httpClientMutex = xSemaphoreCreateMutex();
  if (httpClientMutex == NULL) {
    Serial.println("Failed to create HTTPClient mutex");
    while (1)
      ;  // Halt if the mutex creation fails
  }

  setupLittleFS();
  xTaskCreatePinnedToCore(
    blinkBlueLedTask,        /* Task function. */
    "blinkBlueLedTask",      /* name of task. */
    10000,                   /* Stack size of task */
    NULL,                    /* parameter of the task */
    1,                       /* priority of the task */
    &blinkBlueLedTaskHandle, /* Task handle to keep track of created task */
    1                        /* pin task to core 0 */
  );
  if (setupWiFi()) {
    shouldStopBlinkingBlueLed = true;
    vTaskDelay(2500 / portTICK_RATE_MS);
    digitalWrite(LedBlue, HIGH);
  } else {
    shouldStopBlinkingBlueLed = true;
    vTaskDelay(2500 / portTICK_RATE_MS);
    digitalWrite(LedBlue, LOW);
  }
  setupSensors();

  // rtc.begin();

  getSavedRtcTimestamp();

  Serial.println("=========================================================================================================");

  httpQueue = xQueueCreate(25, sizeof(SensorReadings *));
  alertQueue = xQueueCreate(25, sizeof(bool));

  xTaskCreatePinnedToCore(
    controleAutomaticoBombaTask,        /* Task function. */
    "controleAutomaticoBombaTask",      /* name of task. */
    20000,                              /* Stack size of task */
    NULL,                               /* parameter of the task */
    1,                                  /* priority of the task */
    &controleAutomaticoBombaTaskHandle, /* Task handle to keep track of created task */
    1                                   /* pin task to core 0 */
  );
  xTaskCreatePinnedToCore(
    controleManualBombaTask,        /* Task function. */
    "controleManualBombaTask",      /* name of task. */
    20000,                          /* Stack size of task */
    NULL,                           /* parameter of the task */
    1,                              /* priority of the task */
    &controleManualBombaTaskHandle, /* Task handle to keep track of created task */
    1                               /* pin task to core 0 */
  );
  xTaskCreatePinnedToCore(
    ReadInputsTask,        /* Task function. */
    "ReadInputsTask",      /* name of task. */
    10000,                 /* Stack size of task */
    NULL,                  /* parameter of the task */
    1,                     /* priority of the task */
    &ReadInputsTaskHandle, /* Task handle to keep track of created task */
    1                      /* pin task to core 0 */
  );
  xTaskCreatePinnedToCore(
    sendDataToServerTask,        /* Task function. */
    "sendDataToServerTask",      /* name of task. */
    10000,                       /* Stack size of task */
    NULL,                        /* parameter of the task */
    1,                           /* priority of the task */
    &sendDataToServerTaskHandle, /* Task handle to keep track of created task */
    0                            /* pin task to core 0 */
  );
  xTaskCreatePinnedToCore(
    sendAlertToServerTask,        /* Task function. */
    "sendAlertToServerTask",      /* name of task. */
    10000,                        /* Stack size of task */
    NULL,                         /* parameter of the task */
    2,                            /* priority of the task */
    &sendAlertToServerTaskHandle, /* Task handle to keep track of created task */
    0                             /* pin task to core 0 */
  );
  xTaskCreatePinnedToCore(
    timeControlTask,        /* Task function. */
    "timeControlTask",      /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &timeControlTaskHandle, /* Task handle to keep track of created task */
    1                       /* pin task to core 0 */
  );

  xTaskCreatePinnedToCore(
    checkWaterPumpStatusTask,    /* Task function */
    "checkWaterPumpStatusTask",  /* Name of task */
    10000,                       /* Stack size of task */
    NULL,                        /* Parameter of the task */
    1,                           /* Priority of the task */
    &checkWaterPumpStatusHandle, /* Task handle to keep track of created task */
    1                            /* Pin task to core 0 */
  );
}

void checkOtaUpdate() {
  if (xSemaphoreTake(httpClientMutex, 0) == pdTRUE) {
    Serial.println("[OTA] Checking firmware version...");

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    if (http.begin(client, firmwareVersionUrl)) {
      int httpCode = http.GET();
      Serial.printf("[OTA] Version check HTTP code: %d\n", httpCode);

      if (httpCode == 200) {
        String newVersion = http.getString();
        newVersion.trim();

        Serial.printf("[OTA] Current: %s | Remote: %s\n", currentFirmwareVersion.c_str(), newVersion.c_str());

        if (newVersion != currentFirmwareVersion) {
          Serial.println("[OTA] New version found. Fetching asset URL...");

          // Step 2: Get .bin URL
          HTTPClient assetHttp;
          if (assetHttp.begin(client, firmwareUrl)) {
            int assetHttpCode = assetHttp.GET();
            if (assetHttpCode == 200) {
              String binUrl = assetHttp.getString();
              binUrl.trim();
              Serial.printf("[OTA] Got bin URL: %s\n", binUrl.c_str());
              Serial.println("[OTA] New version found. Downloading...");

              HTTPClient binHttp;
              binHttp.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
              if (binHttp.begin(client, binUrl)) {
                binHttp.addHeader("Authorization", "token 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");
                binHttp.addHeader("Accept", "application/octet-stream");
                int binHttpCode = binHttp.GET();

                if (binHttpCode == 200) {
                  int contentLength = binHttp.getSize();
                  if (contentLength > 0 && Update.begin(contentLength)) {
                    WiFiClient *stream = binHttp.getStreamPtr();
                    size_t written = Update.writeStream(*stream);

                    if (Update.end() && Update.isFinished()) {
                      Serial.println("[OTA] Update successful, rebooting...");
                      currentFirmwareVersion = newVersion;
                      delay(2000);
                      ESP.restart();
                    } else {
                      Serial.printf("[OTA] Update failed. Error: %d\n", Update.getError());
                    }
                  } else {
                    Serial.println("[OTA] Not enough space or invalid content length.");
                  }
                } else {
                  Serial.printf("[OTA] Bin download failed: %d\n", binHttpCode);
                }
                binHttp.end();
              }
            } else {
              Serial.println("[OTA] Already running latest version.");
            }
          } else {
            Serial.printf("[OTA] Version check failed. HTTP: %d\n", httpCode);
          }
        }
      }

      http.end();
    } else {
      Serial.println("[OTA] Failed to begin version check.");
    }

    xSemaphoreGive(httpClientMutex);
  } else {
    Serial.println("[OTA] Skipped due to mutex busy");
  }
}

SensorReadings *createSensorReadings() {
  SensorReadings *sensorReadings = (SensorReadings *)malloc(sizeof(SensorReadings));
  if (sensorReadings == NULL) {
    Serial.println("Memory allocation failed");
    return NULL;
  }

  sensorReadings->tempAmb = 0.0;
  sensorReadings->umidAmb = 0.0;
  sensorReadings->tempNPK1 = 0.0;
  sensorReadings->tempNPK2 = 0.0;
  sensorReadings->tempLeaf = 0.0;
  sensorReadings->humNPK1 = 0.0;
  sensorReadings->humNPK2 = 0.0;
  sensorReadings->humLeaf = 0.0;
  sensorReadings->tentativasDeEnvio = 0;

  return sensorReadings;
}

void printReadings(SensorReadings *sensorReadings) {
  Serial.print("tempAmb: ");
  Serial.println(sensorReadings->tempAmb);
  Serial.print("umidAmb: ");
  Serial.println(sensorReadings->umidAmb);
  Serial.print("tempNPK1: ");
  Serial.println(sensorReadings->tempNPK1);
  Serial.print("tempNPK2: ");
  Serial.println(sensorReadings->tempNPK2);
  Serial.print("tempLeaf: ");
  Serial.println(sensorReadings->tempLeaf);
  Serial.print("humNPK1: ");
  Serial.println(sensorReadings->humNPK1);
  Serial.print("humNPK2: ");
  Serial.println(sensorReadings->humNPK2);
  Serial.print("humLeaf: ");
  Serial.println(sensorReadings->humLeaf);
}

void executeSensorsReading() {
  // Serial.print("Salvando tempo agora: ");
  // Serial.println(timeNowAsTimestamp);
  // writeFileInt(LittleFS, pathRTC, timeNowAsTimestamp);
  // lastSavedTime = timeNow;
  SensorReadings *sensorReadings = createSensorReadings();
  readSensors(sensorReadings);
  printReadings(sensorReadings);
  if (digitalRead(ReleBomba) == LOW) {
    if (sensorReadings->humNPK1 <= limitMinNpk1) {
      digitalWrite(LedYellow, HIGH);
      digitalWrite(LedGreen, LOW);
    }
    if (sensorReadings->humNPK1 > limitMinNpk1) {
      digitalWrite(LedGreen, HIGH);
      digitalWrite(LedYellow, LOW);
    }
  }
  if (xQueueSend(httpQueue, &sensorReadings, (TickType_t)10) != pdPASS) {
    Serial.println("Failed to send sensor data to queue");
    if (sensorReadings) free(sensorReadings);
  }
}

bool fixRTC() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return false;
  }
  tmElements_t tm;
  breakTime(mktime(&timeinfo), tm);
  RTC.write(tm);
  return true;
}

void timeControlTask(void *pvParameters) {
  // tmElements_t timeNow;
  int intervalBetweenReadSensors = 25;
  int secondsSinceLastReading = 0;
  // time_t timeNowAsTimestamp = makeTime(timeNow);
  Serial.println("executando primeira medição após ligar");
  executeSensorsReading();
  for (;;) {
    Serial.print("Segundos desde a última medição: ");
    Serial.println(secondsSinceLastReading);
    if (secondsSinceLastReading < intervalBetweenReadSensors) {
      secondsSinceLastReading++;
      vTaskDelay(1000 / portTICK_RATE_MS);
      continue;
    }
    secondsSinceLastReading = 0;
    executeSensorsReading();
    vTaskDelay(1000 / portTICK_RATE_MS);
    // if (RTC.read(timeNow)) {
    //   time_t timeNowAsTimestamp = makeTime(timeNow);
    //   time_t lastTimeSavedAsTimestamp = makeTime(lastSavedTime);
    //   int32_t timestampDifferenceInSeconds = timeNowAsTimestamp - lastTimeSavedAsTimestamp;
    //   Serial.print("Segundos desde a última medição: ");
    //   Serial.println(timestampDifferenceInSeconds);
    //   if (timestampDifferenceInSeconds < 0) {
    //     LittleFS.remove(pathRTC);
    //     esp_restart();
    //   }
    //   if (timestampDifferenceInSeconds >= intervalBetweenReadSensors) {
    //     executeSensorsReading(timeNow, timeNowAsTimestamp);
    //   }
    // } else {
    //   fixRTC();
    // }
  }
  vTaskDelete(NULL);
}

void blinkBlueLedTask(void *pvParameters) {
  for (;;) {
    if (shouldStopBlinkingBlueLed) {
      break;
    }
    digitalWrite(LedBlue, HIGH);
    vTaskDelay(1000 / portTICK_RATE_MS);
    digitalWrite(LedBlue, LOW);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

void controleManualBomba() {
  bool alerta = false;
  if (bombaLigada) {
    bombaLigada = false;
    digitalWrite(ReleBomba, LOW);
    digitalWrite(LedRed, LOW);
    pararControleManualBomba = true;
    xQueueSend(alertQueue, &alerta, (TickType_t)0);
  } else {
    bombaLigada = true;
    alerta = true;
    digitalWrite(ReleBomba, HIGH);
    digitalWrite(LedRed, HIGH);
    pararControleManualBomba = false;
    xTaskNotifyGive(controleManualBombaTaskHandle);
    xQueueSend(alertQueue, &alerta, (TickType_t)0);
  }
}

void controleManualBombaTask(void *pvParameters) {
  int loopCount = 0;       // contador de quantas vezes fez o delay
  int loopCounter = 0;     // contador de quantos segundos passou
  int loopFinish = 15;     // quantos segundos é pra ler novamente
  int loopDelay = 50;      // delay do loop
  bool shouldRead = true;  // deve executar a leitura?
  for (;;) {
    Serial.println("Controle manual da bomba no aguardo");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("Iniciando controle manual da bomba");
    SensorReadings *sensorReadings = createSensorReadings();
    for (;;) {
      if (pararControleManualBomba) {
        Serial.println("Parando controle manual da bomba");
        free(sensorReadings);
        break;
      }
      if (loopCount == (1000 / loopDelay)) {
        loopCount = 0;
        loopCounter += 1;
      }
      if (loopCounter == loopFinish) {
        loopCounter = 0;
        shouldRead = true;
      }
      if (shouldRead) {
        shouldRead = false;
        readSensors(sensorReadings);
        printReadings(sensorReadings);
        if (sensorReadings->humNPK1 >= limitMaxNpk1) {
          digitalWrite(ReleBomba, LOW);
          digitalWrite(LedRed, LOW);
          digitalWrite(LedYellow, LOW);
          digitalWrite(LedGreen, HIGH);
          free(sensorReadings);
          Serial.println("Parando bomba manual");
          break;
        }
      }
      loopCount += 1;
      vTaskDelay(loopDelay / portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void controleAutomaticoBombaTask(void *pvParameters) {
  bool alerta = true;
  int loopCount = 0;       // contador de quantas vezes fez o delay
  int loopCounter = 0;     // contador de quantos segundos passou
  int loopFinish = 15;     // quantos segundos é pra ler novamente
  int loopDelay = 50;      // delay do loop
  bool shouldRead = true;  // deve executar a leitura?
  for (;;) {
    controleAutomaticoBombaTaskStarted = false;
    Serial.println("Controle automático da bomba no aguardo");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("Iniciando controle automático da bomba");
    xQueueSend(alertQueue, &alerta, (TickType_t)0);
    SensorReadings *sensorReadings = createSensorReadings();
    for (;;) {
      if (pararControleAutomaticoBomba) {
        Serial.println("Parando controle automático da bomba");
        digitalWrite(ReleBomba, LOW);
        digitalWrite(LedRed, LOW);
        alerta = false;
        xQueueSend(alertQueue, &alerta, (TickType_t)0);
        free(sensorReadings);
        break;
      }
      if (loopCount == (1000 / loopDelay)) {
        loopCount = 0;
        loopCounter += 1;
      }
      if (loopCounter == loopFinish) {
        loopCounter = 0;
        shouldRead = true;
      }
      if (shouldRead) {
        shouldRead = false;
        readSensors(sensorReadings);
        printReadings(sensorReadings);
        if (sensorReadings->humNPK1 >= limitMaxNpk1) {
          digitalWrite(ReleBomba, LOW);
          digitalWrite(LedRed, LOW);
          digitalWrite(LedYellow, LOW);
          digitalWrite(LedGreen, HIGH);
          alerta = false;
          xQueueSend(alertQueue, &alerta, (TickType_t)0);
          free(sensorReadings);
          Serial.println("Parando bomba automatico");
          break;
        }
      }
      loopCount += 1;
      vTaskDelay(loopDelay / portTICK_RATE_MS);
    }
  }
  vTaskDelete(NULL);
}

void readSensors(SensorReadings *sensorReadings) {
  sensors_event_t humiditySHT40, tempSHT40;
  sht4.getEvent(&humiditySHT40, &tempSHT40);
  for (size_t i = 0; i <= 5; i++) {
    if (tentativasLerSensorTemperatura == 5) { break; }
    sht4.getEvent(&humiditySHT40, &tempSHT40);
    if (tempSHT40.temperature <= 0.0) {
      tentativasLerSensorTemperatura += 1;
      continue;
    } else {
      tentativasLerSensorTemperatura = 1;
      break;
    }
  }
  sensorReadings->tempAmb = getTempAmb(tempSHT40.temperature);
  sensorReadings->umidAmb = getUmidAmb(humiditySHT40.relative_humidity);
  sensorReadings->tempNPK1 = getTempNPK1();
  sensorReadings->tempNPK2 = getTempNPK2();
  sensorReadings->tempLeaf = getTempLeaf();
  sensorReadings->humNPK1 = getHumNPK1();
  sensorReadings->humNPK2 = getHumNPK2();
  sensorReadings->humLeaf = getHumLeaf();
}

//TimeRTC --------------------------------------------------------------------
String formatTimestamp(uint8_t sec, uint8_t minute, uint8_t hour, uint8_t day, uint8_t month, uint16_t year) {
  return String(year) + "-" + formatDigits(month) + "-" + formatDigits(day) + " " + formatDigits(hour) + ":" + formatDigits(minute) + ":" + formatDigits(sec);
}

String formatDigits(uint8_t value) {
  if (value < 10) {
    return "0" + String(value);
  } else {
    return String(value);
  }
}

//LitleFS --------------------------------------------------------------------
void writeFileInt(fs::FS &fs, const char *fileName, int value) {
  File file = fs.open(fileName, FILE_WRITE);
  if (!file) {
    Serial.println("falha ao abrir arquivo para escrever");
    return;
  }

  file.write((uint8_t *)&value, sizeof(value));
  file.close();
}

int readFileInt(fs::FS &fs, const char *fileName) {
  File file = fs.open(fileName);
  if (!file || file.isDirectory()) {
    Serial.println(" - falha ao abrir arquivo para ler");
    return 0;
  }

  int value;
  file.read((uint8_t *)&value, sizeof(value));
  Serial.println(" - arquivo lido");
  file.close();

  return value;
}

String readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void salvaNoJson(String chave, String valor) {
  Serial.println("Salvando valores no arquivo de configuração");
  if (LittleFS.begin()) {
    if (LittleFS.exists(pathConfig)) {
      File file = LittleFS.open(pathConfig, FILE_READ);
      if (file) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
          Serial.println("Failed to open file");
        }

        file.close();

        doc[chave] = valor;

        file = LittleFS.open(pathConfig, FILE_WRITE);
        if (!file) {
          Serial.println("Houve uma falha ao abrir o arquivo de configuração para incluir/alterar as configurações");
        }
        if (serializeJson(doc, file) == 0) {
          Serial.println(F("Failed to write to file"));
        }
      }
      file.close();
    } else {
      Serial.println("Arquivo não existe");
    }
  }
}

//Sensores -------------------------------------------------------------------
void setupSensors() {
  defSHT40();
  sht4.begin();
}

void defSHT40() {
  //Definições SHT40
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
    case SHT4X_HIGH_PRECISION:
      Serial.println("High precision");
      break;
    case SHT4X_MED_PRECISION:
      Serial.println("Med precision");
      break;
    case SHT4X_LOW_PRECISION:
      Serial.println("Low precision");
      break;
  }

  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
    case SHT4X_NO_HEATER:
      Serial.println("No heater");
      break;
    case SHT4X_HIGH_HEATER_1S:
      Serial.println("High heat for 1 second");
      break;
    case SHT4X_HIGH_HEATER_100MS:
      Serial.println("High heat for 0.1 second");
      break;
    case SHT4X_MED_HEATER_1S:
      Serial.println("Medium heat for 1 second");
      break;
    case SHT4X_MED_HEATER_100MS:
      Serial.println("Medium heat for 0.1 second");
      break;
    case SHT4X_LOW_HEATER_1S:
      Serial.println("Low heat for 1 second");
      break;
    case SHT4X_LOW_HEATER_100MS:
      Serial.println("Low heat for 0.1 second");
      break;
  }
}

float getTempAmb(float tempSHT40) {
  float tempAmb = tempSHT40 + 0.03;
  return tempAmb;
}

float getUmidAmb(float humiditySHT40) {
  float umidAmb = humiditySHT40 + 5.13;
  umidAmb = umidAmb + (umidAmb * 0.232) - 28.27;
  return umidAmb;
}

float getTempLeaf() {
  int analogValueT = analogRead(32);
  //float analogVoltsT = analogReadMilliVolts(34);

  return analogValueT;
}

float getHumLeaf() {
  int valorAnalogico = analogRead(33);  // analog maximo 65520
  float humLeafPercentage = map(valorAnalogico, 0, 65520, 0, 100);
  // float tensao = (valorAnalogico / 4095.0) * 3.3 * 1.192446043165468 * 1.974164133738;  // Ajuste a tensão de acordo com a resolução e a tensão de referência
  // float calculado = ((100 / (5 - 0)) * tensao);

  return humLeafPercentage;
}

float getHumNPK1() {
  int valorAnalogico = analogRead(34);
  //float tensao = (valorAnalogico / 4095.0) * 3.3 * 1.192446043165468 * 1.974164133738;
  float calculado = map(valorAnalogico, 0, 65535, 0, 100);

  return calculado;
}

float getTempNPK1() {
  int valorAnalogico = analogRead(35);
  float calculado = map(valorAnalogico, 0, 65535, -40, 80) - 30;

  return calculado;
}

float getHumNPK2() {
  int valorAnalogico = analogRead(36);
  float calculado = map(valorAnalogico, 0, 65535, 0, 100);

  return calculado;
}

float getTempNPK2() {
  int valorAnalogico = analogRead(39);
  float calculado = map(valorAnalogico, 0, 65535, -40, 80) - 30;

  return calculado;
}

//HTTP -------------------------------------------------------------------
void sendDataToServerTask(void *pvParameters) {
  for (;;) {
    SensorReadings *sensorReadings = NULL;
    if (xQueueReceive(httpQueue, &sensorReadings, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Tentando conectar WiFi");
        WiFi.begin(ssid, senha);
        vTaskDelay(5000 / portTICK_RATE_MS);

        Serial.print("Status do WiFi: ");
        Serial.print(WiFi.status());
        Serial.println();
        if (WiFi.status() != WL_CONNECTED) {
          digitalWrite(LedBlue, LOW);
          free(sensorReadings);
          continue;
        }
        digitalWrite(LedBlue, HIGH);
      }
      if (sendDataToServer(sensorReadings)) {
        free(sensorReadings);
      } else {
        Serial.println("falha ao enviar dados");
        Serial.print("tentativas de envio: ");
        Serial.println(sensorReadings->tentativasDeEnvio);
        if (sensorReadings->tentativasDeEnvio >= 3) {
          Serial.println("3 tentativas de envio esgotadas");
          free(sensorReadings);
        } else {
          if (xQueueSend(httpQueue, &sensorReadings, (TickType_t)10) != pdPASS) {
            Serial.println("Failed to send sensor data to queue");
          }
        }
      }
    }
  }
  vTaskDelete(NULL);
}

void sendAlertToServerTask(void *pvParameters) {
  for (;;) {
    bool alert = false;
    if (xQueueReceive(alertQueue, &alert, portMAX_DELAY) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Tentando conectar WiFi");
        WiFi.begin(ssid, senha);
        vTaskDelay(5000 / portTICK_RATE_MS);

        Serial.print("Status do WiFi: ");
        Serial.print(WiFi.status());
        Serial.println();
        if (WiFi.status() != WL_CONNECTED) {
          digitalWrite(LedBlue, LOW);
          continue;
        }
        digitalWrite(LedBlue, HIGH);
      }
      sendAlertToServer(alert);
    }
  }
  vTaskDelete(NULL);
}

bool sendDataToServer(SensorReadings *sensorReadings) {
  if (xSemaphoreTake(httpClientMutex, portMAX_DELAY) == pdTRUE) {
    httpClient.setReuse(false);
    httpClient.setTimeout(5000);
    httpClient.begin(uriSaveData);
    httpClient.addHeader("Content-Type", "application/json");

    String data;
    JsonDocument doc;
    tmElements_t timeNow;
    if (RTC.read(timeNow)) {
      time_t timeNowAsTimestamp = makeTime(timeNow);
      doc["timestamp"] = timeNowAsTimestamp;
    } else {
      Serial.println("rtc bugado");
      doc["timestamp"] = 0;
    }

    doc["esp_id"] = "123";
    doc["umidade_ambiente"] = sensorReadings->umidAmb;
    doc["temperatura_ambiente"] = sensorReadings->tempAmb;
    doc["temperatura_sensor_1"] = sensorReadings->tempNPK1;
    doc["temperatura_sensor_2"] = sensorReadings->tempNPK2;
    doc["temperatura_folha"] = sensorReadings->tempLeaf;
    doc["umidade_sensor_1"] = sensorReadings->humNPK1;
    doc["umidade_sensor_2"] = sensorReadings->humNPK2;
    doc["umidade_folha"] = sensorReadings->humLeaf;
    doc["sensor_umidade_1_min"] = limitMinNpk1;
    doc["sensor_umidade_2_min"] = limitMinNpk2;
    doc["sensor_umidade_1_max"] = limitMaxNpk1;
    doc["sensor_umidade_2_max"] = limitMaxNpk2;

    serializeJson(doc, data);
    Serial.println("enviando dados para o servidor");
    int statusCode = httpClient.POST(data);
    Serial.printf("send data response status code: %d\n", statusCode);

    String responseBody = httpClient.getString();
    Serial.println(responseBody);

    httpClient.end();
    xSemaphoreGive(httpClientMutex);

    sensorReadings->tentativasDeEnvio += 1;

    if (statusCode == 200) {
      Serial.println("dados enviados");
      return true;
    } else {
      return false;
    }
  } else {
    Serial.println("Failed to take HTTPClient mutex");
    return false;
  }
}


void sendAlertToServer(bool bombaLigada) {
  if (xSemaphoreTake(httpClientMutex, portMAX_DELAY) == pdTRUE) {
    httpClient.setReuse(false);
    httpClient.setTimeout(5000);
    httpClient.begin(uriAlertWaterPump);
    httpClient.addHeader("Content-Type", "application/json");

    String data;
    JsonDocument doc;
    doc["esp_id"] = "123";
    doc["bomba_ligada"] = bombaLigada;

    if (estado_controle_bomba) {
      doc["estado_controle_bomba"] = "automatico";
    } else {
      doc["estado_controle_bomba"] = "manual";
    }

    serializeJson(doc, data);
    int statusCode = httpClient.POST(data);
    Serial.printf("send alert response status code: %d\n", statusCode);

    String responseBody = httpClient.getString();
    Serial.println(responseBody);

    if (statusCode == 200) {
      Serial.println("alerta enviado");
    } else {
      Serial.println("falha ao enviar alerta");
    }

    httpClient.end();

    xSemaphoreGive(httpClientMutex);
  } else {
    Serial.println("Failed to take HTTPClient mutex");
  }
}

void checkWaterPumpStatusTask(void *parameter) {
  Serial.println("Starting checkWaterPumpStatusTask...");

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (xSemaphoreTake(httpClientMutex, portMAX_DELAY) == pdTRUE) {
        httpClient.setReuse(false);
        httpClient.setTimeout(5000);
        httpClient.begin(uriChangeWaterPupmStatus);  // Use global client and URI

        int httpCode = httpClient.GET();
        Serial.printf("check pump status response status code: %d\n", httpCode);

        if (httpCode == HTTP_CODE_OK) {
          String payload = httpClient.getString();
          Serial.println("Received payload: " + payload);

          DynamicJsonDocument doc(1024);
          DeserializationError error = deserializeJson(doc, payload);
          if (!error) {
            bool bombaLigadaServer = doc["status"]["bomba_ligada"];
            digitalWrite(ReleBomba, bombaLigadaServer ? HIGH : LOW);
            digitalWrite(LedRed, bombaLigadaServer ? HIGH : LOW);
            bombaLigada = bombaLigadaServer;
            Serial.printf("Bomba status updated: %s\n", bombaLigada ? "ON" : "OFF");
          } else {
            Serial.println("Failed to parse JSON");
          }
        } else {
          Serial.println("Failed to fetch pump status");
        }

        httpClient.end();
        xSemaphoreGive(httpClientMutex);
      } else {
        Serial.println("Failed to take HTTPClient mutex");
      }
    }

    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Delay 30s before next check
  }
}


//HTTP------------------------------------------------------------------------
void interpretarCodigoHTTP(int codigoHTTP) {
  switch (codigoHTTP) {
    case HTTP_CODE_OK:
      Serial.println("200 OK - Requisição bem-sucedida");
      break;
    case HTTP_CODE_CREATED:
      Serial.println("201 Created - Recurso criado com sucesso");
      break;
    case HTTP_CODE_NO_CONTENT:
      Serial.println("204 No Content - Requisição bem-sucedida, mas sem conteúdo para retornar");
      break;
    case HTTP_CODE_MOVED_PERMANENTLY:
      Serial.println("301 Moved Permanently - Recurso movido permanentemente");
      break;
    case HTTP_CODE_FOUND:
      Serial.println("302 Found - Recurso encontrado, mas redirecionado temporariamente");
      break;
    case HTTP_CODE_BAD_REQUEST:
      Serial.println("400 Bad Request - Requisição inválida");
      break;
    case HTTP_CODE_UNAUTHORIZED:
      Serial.println("401 Unauthorized - Autenticação necessária ou falha na autenticação");
      break;
    case HTTP_CODE_FORBIDDEN:
      Serial.println("403 Forbidden - Acesso proibido ao recurso");
      break;
    case HTTP_CODE_NOT_FOUND:
      Serial.println("404 Not Found - Recurso não encontrado");
      break;
    case HTTP_CODE_METHOD_NOT_ALLOWED:
      Serial.println("405 Method Not Allowed - Método de requisição não permitido para o recurso");
      break;
    case HTTP_CODE_INTERNAL_SERVER_ERROR:
      Serial.println("500 Internal Server Error - Erro interno do servidor");
      break;
    case HTTP_CODE_SERVICE_UNAVAILABLE:
      Serial.println("503 Service Unavailable - Serviço não disponível");
      break;
    default:
      Serial.print("Código HTTP desconhecido: ");
      Serial.println(codigoHTTP);
      break;
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastOtaCheck > otaInterval) {
    lastOtaCheck = now;
    checkOtaUpdate();
  }
}
