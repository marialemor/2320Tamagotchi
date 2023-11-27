#include <Arduino.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

QueueHandle_t queueBotton;

//parametros de la pantalla
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64 

#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//parametros del MPU
Adafruit_MPU6050 mpu;

//pines de los botones
const int BathButton = 18;
const int EatButton = 5;
const int WalkButton = 19;

void Pant(void *pvParameters);
void Eat(void *pvParameters);
void walk(void *pvParameters);
void Bath(void *pvParameters);
void time_count(void *pvParameters);
void Barras(void *pvParameters);

#define WIRE Wire

//parametros para caminar
int pasos = 0;
int iniciar = 0; 
int contar = 0;

// Stats de la mascota
float hunger = 100;
float fun = 100;
float bath = 100;

void Pant(void *pvParameters){

  while(1) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("MENU");
    display.display();
    delay(2000);

  }
}

void walk(void *parameter) {
  int buttonReceived = 0;

  while(1) {

    if (xQueueReceive(queueBotton,&buttonReceived,portMAX_DELAY)){
      if (buttonReceived == 1){
        
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
      }
    }

}}

void Eat(void *pvParameters){
  int buttonReceived = 0;

  while(1) {

    if (xQueueReceive(queueBotton,&buttonReceived,portMAX_DELAY)){
      if (buttonReceived == 2){
        display.setCursor(0,0);
        display.println("eat");
        display.display();
        display.clearDisplay();
        //task
      }
    }
  }
}

void Bath(void *pvParameters){
  int buttonReceived = 0;

  while(1) {
    if (xQueueReceive(queueBotton,&buttonReceived,portMAX_DELAY)){
      if (buttonReceived == 3){
        display.setCursor(0,0);
        display.println("Bath");
        display.display();
        display.clearDisplay();
      }
    }

  }
}

void Eat_Interrupt()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
      int dataEat = 1;
      xQueueSend(queueBotton, &dataEat, portMAX_DELAY);
  }
  last_interrupt_time = interrupt_time;
}

void Walk_Interrupt()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
      int dataWalk = 2;
      xQueueSend(queueBotton, &dataWalk, portMAX_DELAY);
    
  }
  last_interrupt_time = interrupt_time;
}

void Bath_Interrupt()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
      int dataBath = 3;
      xQueueSend(queueBotton, &dataBath, portMAX_DELAY);
    
  }
  last_interrupt_time = interrupt_time;
}


void time_count(void *pvParameters) {
  
  while(1) {
    Serial.print("time");
  }
}

void Barras(void *pvParameters) {
  
  while(1) {
    //task
  }
}

void setup() {
  Serial.begin(9600);

  // Initialize the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(200);
  display.setRotation(1);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("INIT");
  display.display();
  delay(200);

  //pines de los botones

  pinMode(WalkButton, INPUT_PULLUP);
  pinMode(BathButton, INPUT_PULLUP);
  pinMode(EatButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(WalkButton), Walk_Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BathButton), Bath_Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(EatButton), Eat_Interrupt, FALLING);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  //Cola para botones
  queueBotton = xQueueCreate(50,sizeof(int));

  //Creacion de los tasks
  xTaskCreate(Pant," Pantalla", 4096, NULL, 1, NULL);  
  xTaskCreate(Eat," Eat", 4096, NULL, 1, NULL); 
  xTaskCreate(walk," walk", 4096, NULL, 1, NULL); 
  xTaskCreate(Bath," Bath", 4096, NULL, 1, NULL); 
  xTaskCreate(Barras," Barras", 4096, NULL, 1, NULL); 

  vTaskStartScheduler();
}

void loop() {
}