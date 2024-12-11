#include <Arduino.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

  //définition des numéros des PINS
  int BP_BLEU=0;
  int BP_JAUNE=2;
  int BP_VERT=12;
  int POTAR = 33;

  //déclaration des variables pour lire les entrées
  int VAL_BP_BLEU;
  int VAL_BP_JAUNE;
  int VAL_BP_VERT;
  int VAL_POTAR;

void setup() {
  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.printf("Trieur de balles");

  //initialise la couleur de l'écran
  const int colorR = 255;
  const int colorG = 0;
  const int colorB = 70;

  lcd.setRGB(colorR, colorG, colorB);

  //configuration des entées
  pinMode(BP_BLEU,INPUT_PULLDOWN);
  pinMode(BP_JAUNE,INPUT_PULLUP);
  pinMode(BP_VERT,INPUT_PULLUP);

  //confi de la liasion série
  Serial.begin(9600);
}

void loop() {


//Lecture des entrées
VAL_BP_BLEU=digitalRead(BP_BLEU);
VAL_BP_JAUNE=digitalRead(BP_JAUNE);
VAL_BP_VERT=digitalRead(BP_VERT);
VAL_POTAR=analogRead(POTAR);

//change de couleur tout les X temps
/*delay(3000);//temps en ms
lcd.setRGB(200, 150, 40);
delay(2000);
lcd.setRGB(20, 50, 200);
delay(1000);
lcd.setRGB(255, 0, 120);*/


if (VAL_BP_BLEU==false)
{
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.printf("C'est BLEU");
}
else if (VAL_BP_JAUNE==false)
{
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.printf("C'est JAUNE");
}
else if (VAL_BP_VERT==false)
{
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.printf("C'est VERT");
}
else
{
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.printf("pot=%d", VAL_POTAR);
}
}
