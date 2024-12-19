#include <Arduino.h>
#include "rgb_lcd.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include "ESP32Encoder.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

ESP32Encoder encoder;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

uint16_t r, g, b, c, colorTemp, lux;

rgb_lcd lcd;

// définition des numéros des PINS
int pwm = 27;
int sens = 26;
int BP_BLEU = 0;
int BP_JAUNE = 2;
int BP_VERT = 12;
int POTAR = 33;
int Vts = 0;
int Consigne = 10;
float Kp = 100, Ki = 10;
int etat = 0;
int servo = 13;
int pote;

// déclaration des variables pour lire les entrées
int VAL_BP_BLEU;
int VAL_BP_JAUNE;
int VAL_BP_VERT;
int VAL_POTAR;

int frequence = 25000;
int canal = 0;
int canal1 = 1;
int resolution = 11;

void vTaskPeriodic(void *pvParameters)
{
  int a, aprecedent = 0, err, Somme=0, delta;
  TickType_t xLastWakeTime;
  float speed;
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    a = encoder.getCount();
    delta = a - aprecedent;
    err = Consigne - delta;
    Somme += err;
    if (Somme>204) Somme=204; else if (Somme<-204) Somme=-204;
    speed = Ki * Somme + Kp * err;
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
    Serial.printf("%d %f %d\n", err, speed, Somme);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void setup()
{
  init();

  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

  encoder.attachFullQuad(23, 19);

  encoder.setCount(0);

  // initialise la couleur de l'écran
  lcd.setRGB(255, 0, 70);

  // configuration des entées
  pinMode(sens, OUTPUT);
  pinMode(BP_BLEU, INPUT_PULLUP);
  pinMode(BP_JAUNE, INPUT_PULLUP);
  pinMode(BP_VERT, INPUT_PULLUP);
  pinMode(POTAR, INPUT);

  ledcSetup(canal, frequence, resolution);
  // ledcSetup(canal1, 50, 16);

  ledcAttachPin(pwm, canal);
  // ledcAttachPin(servo, canal1);

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

  // Lecture des entrées
  VAL_BP_BLEU = digitalRead(BP_BLEU);
  VAL_BP_JAUNE = digitalRead(BP_JAUNE);
  VAL_BP_VERT = digitalRead(BP_VERT);
  VAL_POTAR = analogRead(POTAR);
  lcd.setCursor(0, 1);
  lcd.printf("pot=%4d", VAL_POTAR);
  Consigne = (VAL_POTAR-2047) / 50;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  // Serial.print("Color Temp: ");
  // Serial.print(colorTemp, DEC);
  // Serial.print(" K - ");
  // Serial.print("Lux: ");
  // Serial.print(lux, DEC);
  // Serial.print(" - ");
  // Serial.print("R: ");
  // Serial.print(r, DEC);
  // Serial.print(" ");
  // Serial.print("G: ");
  // Serial.print(g, DEC);
  // Serial.print(" ");
  // Serial.print("B: ");
  // Serial.print(b, DEC);
  // Serial.print(" ");
  // Serial.print("C: ");
  // Serial.print(c, DEC);
  // Serial.print(" ");
  // Serial.println(" ");

  if (VAL_BP_VERT == false)
  {
    // ledcWrite(canal1, 5746);
  }
  else if (VAL_BP_JAUNE == false)
  {
    //  ledcWrite(canal1, 3650);
  }
  else
  {
    // pour etre au milleux 4698, pou etre a gauche 3650, pour etre a droite 5746
    // ledcWrite(canal1, 4698);
  }

  /*if(speed < Consigne)
  {
    speed = Consigne;
  }
  else if(speed > Consigne)
  {
    speed = Consigne;
  }*/

  delay(1000);
}