#include <Arduino.h>
#include "rgb_lcd.h"

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
//  VAL_POTAR = analogRead(33);
  lcd.setCursor(0, 1);
  lcd.printf("pot=%4d", VAL_POTAR);

  delay(100);
  Serial.printf("Test\n");
}
