#include <Arduino.h>
#include "rgb_lcd.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <SPI.h>
#include "ESP32Encoder.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


ESP32Encoder encoder;
ESP32Encoder encoder2;



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
int NewVal = encoder.getCount();
int Vitesse = 0;
int OldVal = 0;
int speed = 0;
int Erreur = 0, Consigne = 511, Kp=1, Ki=2, Somme=Consigne;
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

void setup()
{
  init();


  // Initialise la liaison avec le terminal
  Serial.begin(115200);


  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);


  encoder.attachHalfQuad(19, 18);

  encoder.setCount(0);


  // initialise la couleur de l'écran
  const int colorR = 255;
  const int colorG = 0;
  const int colorB = 70;

  lcd.setRGB(colorR, colorG, colorB);


  // configuration des entées
  pinMode(sens, OUTPUT);
  pinMode(BP_BLEU, INPUT_PULLUP);
  pinMode(BP_JAUNE, INPUT_PULLUP);
  pinMode(BP_VERT, INPUT_PULLUP);
  pinMode(POTAR, INPUT);

  ledcSetup(canal, frequence, resolution);
  ledcSetup(canal1, frequence, resolution);

  ledcAttachPin(pwm, canal);
  ledcAttachPin(servo, canal1);


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
  

  /*if(Erreur > 30)
  {
    Erreur = Erreur - 10;
  }

  switch(etat)
  {
    case 0:
      digitalWrite(sens, LOW);
      ledcWrite(canal, 0);
      Serial.println("etat0");
      if (VAL_BP_BLEU==false)
      {
        etat = 1;
      }
      else if (VAL_BP_VERT==false)
      {
        lcd.clear();
        etat = 2;
      }
      break;

    case 1:
      digitalWrite(sens, HIGH);
      if (speed >= 2047)
      {
        speed = 2047;
      }
      ledcWrite(canal, 511);
      Serial.println("etat1");
      if (VAL_BP_JAUNE==false)
      {
        etat = 0;
      }  
      else if (VAL_BP_VERT==false)
      {
        etat = 2;
      }
      break;

    case 2:
        /*NewVal = (NewVal/2^(-20));

        Vitesse = NewVal - OldVal;

        Somme = Somme  + Vitesse;

        Erreur = ((Consigne - Vitesse)/Consigne);

        speed = Kp*Erreur + Ki*Somme;

        OldVal = NewVal;*/

      /*
      digitalWrite(sens, LOW);
      Serial.printf("val=%4d", Vitesse);
      if(speed > 0)
      {
          if (speed > 2047)
      {
        speed = 2047;
      }
      ledcWrite(canal, 511);
      Serial.println("etat2");
      }
      if (VAL_BP_BLEU==false)
      {
        etat = 1;
      }
      else if (VAL_BP_JAUNE==false)
      {
        etat = 0;
      }  
      break;
      
  } 
  */
  VAL_POTAR = analogRead(POTAR);
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


  if(VAL_BP_VERT==false)
  {
    pote = 75;
  }
  if(VAL_BP_JAUNE==false)
  {
    pote = 60;
  }
  else
  {
    pote = 90;
  }

  ledcWrite(canal1, pote);
  lcd.setCursor(0, 0);
  lcd.printf("pote=%4d", pote);
}
