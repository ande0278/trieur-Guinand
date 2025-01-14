#include <Arduino.h>
#include "rgb_lcd.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include "ESP32Encoder.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <CAN.h>


ESP32Encoder encoder;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

uint16_t r, g, b, c, colorTemp, lux;

rgb_lcd lcd;

// définition des numéros des PINS
int pwm = 27;
int sens = 26;
int bp_bleu = 0;
int bp_jaune = 2;
int bp_vert = 12;
int potar = 33;
int vts = 0;
int consigne = 0;
float kp = 30, ki = 5;
int etat = 0;
int servo = 13;
int pote;

// déclaration des variables pour lire les entrées
int val_bp_bleu;
int val_bp_jaune;
int val_bp_vert;
int val_potar;

int frequence = 25000;
int canal = 0;
int canal1 = 2;
int resolution = 11;

void vTaskPeriodic(void *pvParameters)
{
  int a, aprecedent = 0, err, somme=0, delta;
  TickType_t xLastWakeTime;
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();

  float speed;
  while (1)
  {
    a = encoder.getCount();
    delta = a - aprecedent;
    err = consigne - delta;
    somme += err;
    if (somme>204) somme=204; else if (somme<-204) somme=-204;
    speed = ki * somme + kp * err;
    aprecedent = a;
    if (speed > 0)
    {
      if (speed > 2047)
        speed = 2047;
      digitalWrite(sens, LOW);
      ledcWrite(canal, speed);
    }
    else
    {
      if (speed <= -2047)
        speed = -2047;
      digitalWrite(sens, HIGH);
      ledcWrite(canal, -speed);
    }
    //Serial.printf("%d %f %d\n", err, speed, somme);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void setup()
{
  // Initialise la liaison avec le terminal

  Serial.begin(115200);
  while (!Serial);

  Serial.println("CAN Sender");

  CAN.setPins(4, 18);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

  encoder.attachFullQuad(23, 19);

  encoder.setCount(0);

  // initialise la couleur de l'écran
  lcd.setRGB(255, 0, 70);

  // configuration des entées
  pinMode(sens, OUTPUT);
  pinMode(bp_bleu, INPUT_PULLUP);
  pinMode(bp_jaune, INPUT_PULLUP);
  pinMode(bp_vert, INPUT_PULLUP);
  pinMode(potar, INPUT);

  ledcSetup(canal, frequence, resolution);
  ledcSetup(canal1, 50, 16);

  ledcAttachPin(pwm, canal);
  ledcAttachPin(servo, canal1);

  // I2C
  Wire.begin();

  if (tcs.begin())
  {
    Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
      ;
  }

  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 2, NULL);
}

void loop()
{

  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x12);
  CAN.write('h');
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);

  // Lecture des entrées
  val_bp_bleu = digitalRead(bp_bleu);
  val_bp_jaune = digitalRead(bp_jaune);
  val_bp_vert = digitalRead(bp_vert);
  val_potar = analogRead(potar);
  lcd.setCursor(0, 1);
  lcd.printf("pot=%4d", val_potar);
  consigne = (val_potar-2047) / 50;

// code pour lire les couleur du capteur de couleur

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

 //Serial.print("Color Temp: ");
 //Serial.print(colorTemp, DEC);
 //Serial.print(" K - ");
 //Serial.print("Lux: ");
 //Serial.print(lux, DEC);
 //Serial.print(" - ");
 //Serial.print("R: ");
 //Serial.print(r, DEC);
 //Serial.print(" ");
 //Serial.print("G: ");
 //Serial.print(g, DEC);
 //Serial.print(" ");
 //Serial.print("B: ");
 //Serial.print(b, DEC);
 //Serial.print(" ");
 //Serial.print("C: ");
 //Serial.print(c, DEC);
 //Serial.print(" ");
 //Serial.println(" ");


// pour etre au milleux 4698, pou etre a gauche 3650, pour etre a droite 5746
  if (val_bp_vert == false)
  {
    ledcWrite(canal1, 5746);
  }
  else if (val_bp_jaune == false)
  {
    ledcWrite(canal1, 3650);
  }
  else
  {
    ledcWrite(canal1, 4689);
  }
}