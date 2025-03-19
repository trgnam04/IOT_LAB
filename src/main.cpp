

// Import required libraries
#include "WiFi.h"
#include "ThingsBoard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Import supported libraries
#include <Arduino_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>



#define DEBUG 1

// Replace with your network credentials
const char* ssid = "271104E";
const char* password = "1234567890";


/* Server object ---------------------------------------------*/
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size);

// Set up the device properties on server
constexpr char DEVICE_TOKEN[] = "Lab1_IOT";
constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";


/* Sensor object ---------------------------------------------*/
#define DHTPIN 6
#define DHTTYPE    DHT11 
DHT_Unified dht(DHTPIN, DHTTYPE);
typedef struct {
  float Temperature = 0.0;
  float Humidity = 0.0;
} DHT20_Data_t;

DHT20_Data_t DHT20_Data;

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
#ifdef DEBUG
    Serial.println("Connecting to WiFi..");
#endif
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  vTaskResume(PublishData_handle);
  vTaskDelete(NULL);  // Delete the task when done
}

// Task to read value from DHT20
void sensorTask(void* pvParameters) 
{  
  dht.begin();    // Start sensor
  sensors_event_t event;

  while(1){
    dht.temperature().getEvent(&event);
    // get temperature value    
    DHT20_Data.Temperature = event.temperature;
    vTaskDelay(5);
    dht.humidity().getEvent(&event);    
    // get humidity value
    DHT20_Data.Humidity = event.relative_humidity;
#ifdef DEBUG
    Serial.printf("Temperature: %.3f | Humidity: %.3f \n", DHT20_Data.Temperature, DHT20_Data.Humidity);
#endif

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
#ifdef  DEBUG    
        Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, DEVICE_TOKEN);
#endif
        if (!tb.connect(THINGSBOARD_SERVER, DEVICE_TOKEN, THINGSBOARD_PORT))
        {          
#ifdef  DEBUG        
          Serial.println("Failed to connect");
#endif          
        }
        else
        {
          vTaskResume(SensorTask_handle);
#ifdef  DEBUG                  
          Serial.println("Connected");
#endif          
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

  // Create tasks for Wi-Fi and server
  xTaskCreate(sensorTask, "SensorTask", 1024 * 4, NULL, 3, &SensorTask_handle);  
  vTaskSuspend(SensorTask_handle);
  xTaskCreate(publishdataTask, "PublishDataTask", 1024 * 4, NULL, 2, &PublishData_handle);
  vTaskSuspend(PublishData_handle);

  xTaskCreate(wifiTask, "WiFiTask", 1024 * 4, NULL, 1, &WifiTask_handle);    
}
 
void loop(){
  // Nothing to do here, FreeRTOS tasks handle the work
  // Push the main loop to the idle task to save the energy
  vTaskDelay(portMAX_DELAY);
}