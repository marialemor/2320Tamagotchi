#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "queue.h"

#include "Adafruit_MPU6050.h"

#include <WiFi.h>
#include "time.h"

//botones con el numero de pin 
const int pinBotonWalk = 2;
const int pinBotonBath = 3;
const int pinBotonEat = 3;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
Adafruit_MPU6050 mpu;

int pasos = 0;
int iniciar = 0; 
int contar = 0;

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

const char* ssid       = "Mariayo";
const char* password   = "M4R14L3J4";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;


void walk(void *parameter) {
  while(1) {
  if (digitalRead(pinBotonWalk) == LOW) {
  
  //Realizar tarea walk    

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (a.acceleration.y < -8.0){
    iniciar = 0;
    pasos = 0;
  }
  if (a.acceleration.x > 8.0){
    if(iniciar==0){
    }
    iniciar = 1;
    
  }
  if (iniciar == 1){ 
    if (contar == 0 && a.acceleration.y > 9){
        pasos = pasos + 1;
        contar=1;    
    }
    if (contar==1 && (a.acceleration.y) < 9 ){
        contar = 0;    
    }
    
  }
  display.print("Steps:\n ");
  display.println(pasos);


  vTaskDelay(500 / portTICK_PERIOD_MS); // Debouncing para evitar lecturas erróneas debido a rebotes
    }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  }

}


void InitPant(void *pvParameters) {
  (void)pvParameters;
  while(1) {
    
  }
}

void Bath(void *pvParameters) {
  (void)pvParameters;
  while(1) {

  if (digitalRead(pinBotonBath) == LOW) {

  //Realizar tarea Bath
    

  
  vTaskDelay(500 / portTICK_PERIOD_MS); // Debouncing para evitar lecturas erróneas debido a rebotes
    }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Eat(void *pvParameters) {
  (void)pvParameters;
  while(1) {

  if (digitalRead(pinBotonEat) == LOW) {

  //Realizar tarea Eat 

  
    
  vTaskDelay(500 / portTICK_PERIOD_MS); // Debouncing para evitar lecturas erróneas debido a rebotes
    }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Botton(){
  while(1) {
    if (digitalRead(pinBotonBath) == LOW) {


    if (digitalRead(pinBotonEat) == LOW) {

    if (digitalRead(pinBotonWalk) == LOW) {

    }
    }
}
  }
}


void time_count(void *pvParameters) {
  (void)pvParameters;
  while(1) {
    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      return;
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    
  }
}

void setup() {
  Serial.begin(9600);


  queueBotton= xQueueCreate(1,sizeof(unit32_t));


  // Initialize the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  // Clear the display buffer
  display.clearDisplay();
  
  display.display();
  delay(500);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setRotation(2);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              InitPant,  // Function to be called
              "Inicializar Pantalla",   // Name of task
              4096,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              NULL);     // Run on one core for demo purposes (ESP32 only)
  
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              walk,  // Function to be called
              "Walk",   // Name of task
              4096,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              NULL);     // Run on one core for demo purposes (ESP32 only)

xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Eat,  // Function to be called
              "Eat",   // Name of task
              4096,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              NULL);     // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Bath,  // Function to be called
              "Bath",   // Name of task
              4096,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              NULL);     // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Botones,  // Function to be called
              "Botones",   // Name of task
              4096,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              NULL);     // Run on one core for demo purposes (ESP32 only)


  // If this was vanilla FreeRTOS, you'd want to call vTaskStartScheduler() in
  // main after setting up your tasks.
}

void loop() {
  // Do nothing
  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
}