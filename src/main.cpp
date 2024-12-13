#include <Arduino.h>
#include "rgb_lcd.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

uint16_t r, g, b, c, colorTemp, lux;

rgb_lcd lcd;

// définition des numéros des PINS
int pwm = 27;
int SENS = 26;
int BP_BLEU = 0;
int BP_JAUNE = 2;
int BP_VERT = 12;
int POTAR = 33;

// déclaration des variables pour lire les entrées
int VAL_BP_BLEU;
int VAL_BP_JAUNE;
int VAL_BP_VERT;
int VAL_POTAR;

int frequence = 25000;
int canal = 0;
int resolution = 11;

void setup()
{
  init();

  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);

  // lcd.printf("Trieur de balles");

  // initialise la couleur de l'écran
  const int colorR = 255;
  const int colorG = 0;
  const int colorB = 70;

  lcd.setRGB(colorR, colorG, colorB);

  // configuration des entées
  pinMode(SENS, OUTPUT);
  pinMode(BP_BLEU, INPUT_PULLUP);
  pinMode(BP_JAUNE, INPUT_PULLUP);
  pinMode(BP_VERT, INPUT_PULLUP);
  pinMode(POTAR, INPUT);

  ledcSetup(canal, frequence, resolution);

  ledcAttachPin(pwm, canal);

  //I2C
  Wire.begin();

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

}

void loop()
{
  // Lecture des entrées
  VAL_BP_BLEU = digitalRead(BP_BLEU);
  VAL_BP_JAUNE = digitalRead(BP_JAUNE);
  VAL_BP_VERT = digitalRead(BP_VERT);

  if (VAL_BP_BLEU==false)
  {
    digitalWrite(SENS, HIGH);
    ledcWrite(canal, 511);
  }
  else if (VAL_BP_JAUNE==false)
  {
    digitalWrite(SENS, LOW);
    ledcWrite(canal, 511);
  }  
  else if (VAL_BP_VERT==false)
  {
    digitalWrite(SENS, LOW);
    ledcWrite(canal, 0);
  }

  delay(100);
  //VAL_POTAR = analogRead(33);
  lcd.setCursor(0, 1);
  lcd.printf("pot=%4d", VAL_POTAR);

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  /*delay(100);
  Serial.printf("Test\n");

  Wire.requestFrom(22, 2);
  Wire.setClock(21);
  while (Wire.available())
  {
    char c = Wire.read();
    Serial.print(c & 0xFF, BIN);
    Serial.print(", ");
  }
  Serial.println();
  delay(200);*/

}
