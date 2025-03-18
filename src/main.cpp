
// Import required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "DHT20.h"
#include "ThingsBoard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Import supported libraries
#include <Arduino_MQTT_Client.h>


// Replace with your network credentials
// const char* ssid = PROJECT_WIFI_SSID;
// const char* password = PROJECT_WIFI_PASSWORD;

const char* ssid = "ACLAB-IOT";
const char* password = "12345678";


/* Server object ---------------------------------------------*/
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size);

// Set up the device properties on server
constexpr char DEVICE_TOKEN[] = "";
constexpr char TEMPERATURE_KEY[] = "";
constexpr char HUMIDITY_KEY[] = "";


/* Sensor object ---------------------------------------------*/
DHT20 DHT;
typedef struct {
  float Temperature = 0.0;
  float Humidity = 0.0;
} DHT20_Data_t;

DHT20_Data_t DHT20_Data;

// Set LED GPIO
const int ledPin = 13;
// Stores LED state
String ledState;

// Task object
TaskHandle_t WifiTask_handle;
TaskHandle_t SensorTask_handle;
TaskHandle_t PublishData_handle;


// Task to handle Wi-Fi connection
void wifiTask(void *pvParameters) 
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  vTaskDelete(NULL);  // Delete the task when done
}

// Task to read value from DHT20
void sensorTask(void* pvParameters) 
{
  Wire.begin();   // set up I2C bus
  DHT.begin();    // Start sensor

  while(1){
    // get temperature value
    DHT20_Data.Temperature = DHT.getTemperature(); 
    // get humidity value
    DHT20_Data.Humidity = DHT.getHumidity(); 

    // get sensor data periodly
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}


// Task to publish data to coreiot server
void publishdataTask(void* pvParameters)
{

  while(1){
    if (!tb.connected())
    {
        Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, DEVICE_TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, DEVICE_TOKEN, THINGSBOARD_PORT))
        {
            Serial.println("Failed to connect");
        }
        else
        {
            Serial.println("Connected");
        }
    }

    tb.sendTelemetryData(TEMPERATURE_KEY, DHT20_Data.Temperature);
    tb.sendTelemetryData(HUMIDITY_KEY, DHT20_Data.Humidity);

    tb.loop();

    // publish data periodly
    vTaskDelay(5000 / portTICK_PERIOD_MS); 
  }

}

void setup(){
  pinMode(ledPin, OUTPUT);

  // Create tasks for Wi-Fi and server
  xTaskCreate(sensorTask, "SensorTask", 1024 * 4, NULL, 3, &SensorTask_handle);  
  xTaskCreate(publishdataTask, "PublishDataTask", 1024 * 4, NULL, 2, &PublishData_handle);
  xTaskCreate(wifiTask, "WiFiTask", 1024 * 4, NULL, 1, &WifiTask_handle);    
}
 
void loop(){
  // Nothing to do here, FreeRTOS tasks handle the work
  // Push the main loop to the idle task to save the energy
  vTaskDelay(portMAX_DELAY);
}