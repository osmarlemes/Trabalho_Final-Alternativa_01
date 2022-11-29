/*
* Trabalho Final 
* Alternativa 01
* 
* Placa DOIT ESP32 DEVKIT V1 utilizando os sensores TEMP6000 (luminosidade), 
*       MPU6050(acelerômetro) e um buzzer.
*
* Monitoramento e controle via MQTT e apresentação no MQTT-DASH
*/

/*******************************
| Includes                     
*******************************/
#include <Arduino.h>

#include "esp_wifi.h"
#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/*******************************
| Constantes                     
*******************************/
#define ADC1_0_PIN (36) /*ADC1_0*/
#define GPIO_BUZZER (4) /*Pinout Buzzer*/
#define LED (2) /*Led */

/*Configurações para acesso à rede Wifi*/
const char* WIFI_SSID = "Larissa_2.4G";          
const char* WIFI_PASSWORD = "L@ry_2.4G_1234";

/*Configurações para acesso ao broker MQTT*/
const char* MQTT_BROKER = "test.mosquitto.org";    /*Servido MQTT*/
const int MQTT_PORT = 1883;                        /*porta do servidor*/
const char* MQTT_TOPIC_WARNING= "it012/warnings/"; /*topico para enviar os alarmes*/
const char* MQTT_TOPIC_PUB= "it012/sensores/";     /*topico para enviar os dados dos sensores em a cada 2s*/
const char* MQTT_TOPIC_CTRL = "it012/ctrl/";       /*topico para receber os dados do controle MQTT*/

AsyncMqttClient mqttClient;
Adafruit_MPU6050 sensor_mpu;
sensors_event_t g_aceleracao, g_giroscopio, g_temperatura;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

struct STRUCT_SENSORES
{
  float X;
  float Y;
  float Z;
  float roll;
  float pitch;
  float yaw;
  float luminancia;
};

typedef struct STRUCT_SENSORES struct_sensor;

/*******************************
| Variaveis                     
*******************************/
unsigned long timer_loop = millis();      /*timer do loop principal do programa*/
unsigned long timer_broker = millis();    /*timer para o envio das mensagens para o broker*/
unsigned long timer_led = millis();       /*timer para o led interna da placa informar se a aplicação esta rodando*/
struct_sensor sensor_meas;                /*struct com os valores dos sensores */ 
struct_sensor sensor_limiar;              /*struct para armazenar os valores de limite */ 
char string_alarm [100];                  /*vetor para pegar os alarmes */
uint8_t ledState = 0;                     /*variavel para alternar o status do led*/

/*******************************
| Implementação                     
*******************************/
// Funções para iniciar conexão à rede Wifi
void connectToWifi()
{
    Serial.printf("\r\n[connectToWifi] conectando à rede %s...", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/**
 * Função para iniciar conexão ao broker MQTT
*/
void connectToMqtt()
{
    Serial.printf("\r\n[connectToMqtt] conectando ao broker MQTT %s...", MQTT_BROKER);
    mqttClient.clearQueue();
    mqttClient.connect();
}

/**
 * Função responsavel para assinar o topico
*/
void constrolSubscribe()
{
    // Configura um tópico específico para este dispositivo, cujo nome será
    // gerado concatenando o MAC lido do ESP32.
    uint8_t mac[6];
    char topic[60] = { 0 };

    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(topic, sizeof(topic), "%s%02X%02X%02X%02X%02X%02X", MQTT_TOPIC_CTRL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Assina um tópico para controle com nome definido para este dispositivo
    Serial.printf("\r\n[constrolSubscribe] Assinando tópico %s...", topic);
    mqttClient.subscribe(topic, 0);
}

/**
 * Callback para tratar eventos relacionados à conexão Wifi
 */
void WiFiEvent(WiFiEvent_t event)
{
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.printf("\r\n[WiFiEvent] IP obtido: %s\n",
            WiFi.localIP().toString().c_str());

        // Se há conexão Wifi, inicia agora conexão ao broker MQTT:
        connectToMqtt();
        break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("\r\n[WiFiEvent] Conexão perdida...");

        // "mqttReconnectTimer" é utilizado para garantir que não haverá
        // tentativa de reconexão ao broker se a conexão ao falhou.
        mqttClient.disconnect();
        WiFi.disconnect();
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;

    default:
        break;
    }
}

// Callbacks específicos da lib AsyncMqttClient para eventos do MQTT
void onMqttConnect(bool sessionPresent)
{
    Serial.printf("\r\n[connectToMqtt] conectado ao broker MQTT %s. ", MQTT_BROKER);

    // Se há conexão ao broker, assina o tópico de controle
    constrolSubscribe();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.printf("\r\n[onMqttDisconnect] conexão perdida...");

    // se há conexão à uma rede Wifi, é possível disparar um timer para
    // reconexão ao broker
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

/**
 * Função responsavel por tratar as mensagens vindas do MQTT
*/
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
    StaticJsonDocument<150> doc_receive_data; /*documento json para receber dados dados*/

    /*Retira os dados do payload recebido e atualiza os limites*/
    deserializeJson(doc_receive_data, payload);
    sensor_limiar.X = doc_receive_data["MPU6050"]["x"];
    sensor_limiar.Y = doc_receive_data["MPU6050"]["y"];
    sensor_limiar.Z = doc_receive_data["MPU6050"]["z"];
    sensor_limiar.roll = doc_receive_data["MPU6050"]["roll"];
    sensor_limiar.pitch = doc_receive_data["MPU6050"]["pitch"];
    sensor_limiar.yaw = doc_receive_data["MPU6050"]["yaw"];
    sensor_limiar.luminancia = doc_receive_data["TEMT6000"]["luminosidade"];
}

/**
 * Funcao responsavel por fazer a leitura ADC do sensor TEMT6000
*/
static void read_TEMT6000(void)
{
    int adc_input = 0;

    /*faz a leitura adc do sensor TEMT6000*/
    adc_input = analogRead(ADC1_0_PIN);
    sensor_meas.luminancia = (adc_input / 4095.0f) * 100;
}

/**
 * Funcao responsavel em ler os dados do sensor MPU6050
*/
static void read_MPU6050(void)
{
  if(sensor_mpu.getEvent(&g_aceleracao, &g_giroscopio, &g_temperatura))
  {
    sensor_meas.X = g_aceleracao.acceleration.x;
    sensor_meas.Y = g_aceleracao.acceleration.y;
    sensor_meas.Z = g_aceleracao.acceleration.z;
    sensor_meas.roll = g_giroscopio.gyro.x ;
    sensor_meas.pitch = g_giroscopio.gyro.y ;
    sensor_meas.yaw = g_giroscopio.gyro.z ;
  }
  else{
    sensor_meas.X = 0.0f;
    sensor_meas.Y = 0.0f;
    sensor_meas.Z = 0.0f;
    sensor_meas.roll = 0.0f;
    sensor_meas.pitch = 0.0f;
    sensor_meas.yaw = 0.0f;
  }

}

/**
 * Funcao responsavel por fazer a publicação no broker
*/
static void sensorPublish()
{
    StaticJsonDocument<150> doc_send_data;    /*documento json para enviar dados*/

    // Adiciona campos do sensor DHT11 ao objeto JSON doc.
    doc_send_data["x"] = (sensor_meas.X != 0) ? (floorf(sensor_meas.X * 100) / 100.0) : 0.0;
    doc_send_data["y"] = (sensor_meas.Y != 0) ? (floorf(sensor_meas.Y * 100) / 100.0) : 0.0;
    doc_send_data["z"] = (sensor_meas.Z!= 0) ? (floorf(sensor_meas.Z * 100) / 100.0) : 0.0;
    doc_send_data["roll"] = (sensor_meas.roll != 0) ? (floorf(sensor_meas.roll * 100) / 100.0) : 0.0;
    doc_send_data["pitch"] = (sensor_meas.pitch != 0) ? (floorf(sensor_meas.pitch * 100) / 100.0) : 0.0;
    doc_send_data["yaw"] = (sensor_meas.yaw != 0) ? (floorf(sensor_meas.yaw * 100) / 100.0) : 0.0;
    doc_send_data["luminosidade"] = (sensor_meas.luminancia != 0) ? (floorf(sensor_meas.luminancia * 100) / 100.0) : 0.0;
    char buffer[256];
    serializeJson(doc_send_data, buffer);

    // Configura um tópico específico
    uint8_t mac[6];
    char topic[60] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(topic, sizeof(topic), "%s%02X%02X%02X%02X%02X%02X", MQTT_TOPIC_PUB, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Publica uma mensagem MQTT no tópico especificado
    mqttClient.publish(topic, 2, false, buffer);

}

/**
 * Funcao responsavel por fazer a publicação no broker
*/
static void warning_Publish()
{
    StaticJsonDocument<150> doc_send_warning; /*documento json para enviar os alarmes*/

    // Adiciona o alarme ao objeto JSON doc.
    doc_send_warning["alarme"] = string_alarm;

    char buffer[150];
    serializeJson(doc_send_warning, buffer);

    // Configura um tópico específico
    uint8_t mac[6];
    char topic[60] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(topic, sizeof(topic), "%s%02X%02X%02X%02X%02X%02X", MQTT_TOPIC_WARNING, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Publica uma mensagem MQTT no tópico especificado
    mqttClient.publish(topic, 2, false, buffer);

}

/**
 * funcao responsavel por fazer a checagem dos limites e chamar a funcao para enviar o alarme via mqtt
*/
static uint8_t check_limits()
{   
    uint8_t ret = 0;

    memset(string_alarm,0,sizeof(string_alarm));

    if(sensor_meas.X >= sensor_limiar.X)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "X = %.2f", sensor_meas.X); 
        Serial.printf("\nLimite excedido: %s\n",string_alarm);

        /*chama a funcao para publicar o alarme*/
        warning_Publish();

        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.Y >= sensor_limiar.Y)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Y = %.2f", sensor_meas.X); 
        Serial.printf("\nLimite excedido: %s\n",string_alarm);
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.Z >= sensor_limiar.Z)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Z = %.2f", sensor_meas.Z);
        Serial.printf("\nLimite excedido: %s\n",string_alarm); 
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.roll >= sensor_limiar.roll)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Roll = %.2f", sensor_meas.roll); 
        Serial.printf("\nLimite excedido: %s\n",string_alarm);
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.pitch >= sensor_limiar.pitch)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Pitch = %.2f", sensor_meas.pitch);
        Serial.printf("\nLimite excedido: %s\n",string_alarm); 
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.yaw >= sensor_limiar.yaw)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Yaw = %.2f", sensor_meas.yaw); 
        Serial.printf("\nLimite excedido: %s\n",string_alarm);
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    if(sensor_meas.luminancia >= sensor_limiar.luminancia)
    {
        /*envia mensagem paara a serial*/
        sprintf(string_alarm, "Luminancia = %.2f", sensor_meas.luminancia); 
        Serial.printf("\nLimite excedido: %s\n",string_alarm);
        
        /*chama a funcao para publicar o alarme*/
        warning_Publish();
        
        /*indica que houve alarme*/
        ret = 1;
    }

    return ret;

}

/**
 * Faz a configuração inicial do dispositivo
*/
void setup() {
    
    /*configura a serial da placa*/
    Serial.begin(115200);
    Serial.print("\r\n --- TRABALHO FINAL - ALTERNATIVA 01 - OSMAR LEMES--- \n");

    // Configura LED
    pinMode(LED, OUTPUT);

    /*configura o sensor de movimento*/
    if (!sensor_mpu.begin()) {
        Serial.println("Falha ao iniciar o sendor MPU6050!!!");
        while (1)
            yield();
    }

    /*configura o pino do buzzer como saida*/
     pinMode(GPIO_BUZZER, OUTPUT);

    // Criação de timers (utilizando a API do FreeRTOS) para controle do tempo
    // de reconexão para a rede Wifi e para o acesso ao broker MQTT
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, (TimerCallbackFunction_t)(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, (TimerCallbackFunction_t)(connectToWifi));

    // Define callback genérico para os eventos relacionados à conexão wifi
    WiFi.onEvent(WiFiEvent);

        // Definição de callbacks para eventos específicos do MQTT
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);

    // Seta o servidor do broker MQTT e inicia conexão Wifi
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    connectToWifi();

    /*inicializa os valores dos sensores zerados*/
    sensor_meas.X = 0.0f;
    sensor_meas.Y = 0.0f;
    sensor_meas.Z = 0.0f;
    sensor_meas.roll = 0.0f;
    sensor_meas.pitch = 0.0f;
    sensor_meas.yaw = 0.0f;
    sensor_meas.luminancia = 0.0f;

    /*inicializa os valores dos limiares default*/
    sensor_limiar.X = 20.0f;
    sensor_limiar.Y = 20.0f;
    sensor_limiar.Z = 20.0f;
    sensor_limiar.roll = 20.0f;
    sensor_limiar.pitch = 20.0f;
    sensor_limiar.yaw = 20.0f;
    sensor_limiar.luminancia = 100.0f;
}

/**
 * Loop principal do programa
*/
void loop() {
  
  /*Faz coleta dos dados a a cada 10ms*/
  if((millis() - timer_loop) > 5)
  {
    /*faz a leitura dos valores do sensor TEMT6000*/
    read_TEMT6000();

    /*faz a leitura dos valores do sensor MPU*/
    read_MPU6050();

    /*faz a checagem dos limiares*/
    if(check_limits())
    {
        /*aciona o buzzer e envia uma mensagem para a serial e uma mensagem para o broker mqtt*/
        digitalWrite(GPIO_BUZZER, HIGH);
        delay(1000);
        digitalWrite(GPIO_BUZZER, LOW);
    }

    timer_loop = millis();
  }

  /*Faz a publicação no broker a cada 2000ms*/
  if((millis() - timer_broker) > 2000)
  {
    sensorPublish();

    timer_broker = millis();
  }

  if((millis() - timer_led) > 100)
  {
    digitalWrite(LED, !ledState);
    ledState = !ledState;
    timer_led = millis();
  }
}
