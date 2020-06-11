#include <Arduino.h>
#include "Pinmap.h"
#include <avr/wdt.h>
#include "AccelStepper.h"

#define DEBUG
//#define RELEASE

void pinConfig();
void wdtConfig();

AccelStepper motor(AccelStepper::DRIVER, PIN_MTR_STP, PIN_MTR_DIR);
int state = 0;
unsigned long timerStart = 0;
unsigned long fireTime = 0;           //optimalizace typu

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("START");
#endif

  pinConfig();
  wdtConfig();

  motor.setPinsInverted(false, false, false); //dir, step, en
  motor.setAcceleration(1000.0);
  motor.setMaxSpeed(2500.0);
}

void loop()
{
  fireTime = map(analogRead(PIN_POT), 0, 1023, 2000, 30000);

#ifdef DEBUG
  Serial.print("Spinac pist: ");
  Serial.println(digitalRead(PIN_ENS_PISTON));
  Serial.print("Spinac retez: ");
  Serial.println(digitalRead(PIN_ENS_CHAIN));
  Serial.print("Analog: ");
  Serial.println(analogRead(PIN_POT));
  Serial.print("Potenciometr: ");
  Serial.println(fireTime);
  Serial.print("Case: ");
  Serial.println(state);
#endif

  switch (state)
  {
    case 0:
      //pokud je píst venku vypni relé jinak přeskoč
      if (digitalRead(PIN_ENS_PISTON) == 0) {
        digitalWrite(PIN_REL, LOW);
        delay(500); //počkej na píst
      }
      else
        state++;
      break;
    case 1:
      //pokud není sepnut spínač řetězu popojeď
      motor.move(5500);
      while (motor.currentPosition() < 1000)
        motor.run();
      while (digitalRead(PIN_ENS_CHAIN) == 0)
        motor.run();
      motor.setCurrentPosition(0);
      state++;
      break;
    case 2:
      //set timer
      digitalWrite(PIN_REL, HIGH);
      timerStart = millis();
      state++;
      break;
    case 3:
      //wait for timer
      if ((millis() - timerStart) >= fireTime)
      {
        digitalWrite(PIN_REL, LOW);
        state++;
      }
      break;
    default:
      state = 0;
      break;
  }
  delay(200);
  wdt_reset();
}

void pinConfig()
{
  pinMode(PIN_ENS_CHAIN, INPUT_PULLUP);
  digitalWrite(PIN_ENS_CHAIN, HIGH);
  pinMode(PIN_ENS_PISTON, INPUT_PULLUP);
  digitalWrite(PIN_ENS_PISTON, HIGH);

  pinMode(PIN_MTR_DIR, OUTPUT);
  digitalWrite(PIN_MTR_DIR, LOW);
  pinMode(PIN_MTR_STP, OUTPUT);
  digitalWrite(PIN_MTR_STP, LOW);

  pinMode(PIN_REL, OUTPUT);
  digitalWrite(PIN_REL, LOW);
}

void wdtConfig()
{
  wdt_enable(WDTO_8S);
}

/*
  enableOutputs()/disableOutputs() zapne vypne piny (přepne do low)
  currentPosition() vrací aktuální pozici
  targetPosition()  vrací hodnotu cílové pozice
  maxSpeed()        vrací hodnotu max rychlosti
  speed()           vrací hodnotu nastavené rychlosti
  distanceToGo()    vrací rozdíl aktuální a cílové hodnoty
  isRunning()       vrací informaci zda motor zrovna běží

  stop()            nastaví cílovou pozici tak aby motor zastavil
  move()            nastaví cílovou pozici relativně k současné
  moveTo()          nastaví cílovou pozici absolutne
  run()             udělá krok k cílové pozici, s akcelerací
  runSpeed()        udělá krok k cílové pozici, bez akcelerace
  runToPosition()      běž na pozici, s akcelerací, blokuje
  runSpeedToPosition() běž na pozici, bez akcelerace, blokuje
  runToNewPosition()   běž na novou pozici, s akcelerací, (absolutně?) blokuje

  setAcceleration()    nastaví zrychlení, krok za sekundu na druhou
  setSpeed()           nastaví rychlost, kroků za sekundu (max 1000), 0.00027777 = krok za hodinu
  setCurrentPosition() nastaví současnou pozici na 0 (home)
  setMaxSpeed()        nastaví maximální rychlost po akceleraci
*/
